#define _GNU_SOURCE
// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * can-bridge - put a SocketCAN interface on the network
 *
 * Reads frames from a CAN interface and sends them to a UDP peer; optionally
 * takes frames from that peer and puts them on the bus. Also keeps a decoded
 * snapshot of selected IDs in a JSON status file, so a dashboard can show
 * telemetry without anything else parsing CAN.
 *
 * READ-ONLY BY DEFAULT, and that is deliberate. Reading a vehicle bus over a
 * wireless link is harmless. Writing to one is a motion command with a WiFi
 * hop and a router's scheduler in the path, and if the link stalls the last
 * command keeps being acted on. Injection has to be asked for explicitly with
 * --allow-inject, and even then it belongs to bench work, not to a control
 * loop - see doc/CAN.md.
 */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include "agilex.h"

#define MAGIC		0x4e414342u	/* "BCAN" little-endian */
#define VERSION		1
#define MAX_BATCH	24
#define MAX_TRACK	32
#define HDR		8
#define REC		16		/* id(4) dlc(1) pad(3) data(8) */

struct tracked {
	uint32_t id;
	uint8_t len;
	uint8_t data[8];
	uint64_t count;
	uint64_t last_ms;
	bool seen;
};

static struct {
	char *ifname;
	struct sockaddr_in peer;
	bool have_peer;
	bool allow_inject;
	int listen_port;
	char *status_path;
	int status_ms;
	bool foreground;

	struct tracked track[MAX_TRACK];
	int ntrack;

	uint64_t rx, tx, injected, rejected, dropped;

	bool discover;
	uint32_t seen_ids[MAX_TRACK];
	int nseen;

	/*
	 * Which generation of the AgileX protocol this bus is speaking.
	 *
	 * Not a guess: this is how the vendor's own ugv_sdk decides, in
	 * src/utilities/protocol_detector.cpp. Frame 0x151 exists only in v1 and
	 * frames 0x221/0x241 only in v2, so hearing one of them settles it, and
	 * hearing both means something is wrong rather than something is new.
	 *
	 * It matters because the two generations reuse identifiers for unrelated
	 * things - 0x131 is a brake command in v2 and the motion state in v1 - so
	 * there is one decoder per generation and this picks which one runs. The
	 * official demo for the Scout Mini Omni detects at runtime rather than
	 * assuming, so this vehicle can be either and neither answer is hardcoded.
	 *
	 * Listening only. Detection never puts a frame on the bus.
	 */
	bool proto_v1_seen;
	bool proto_v2_seen;
	bool proto_reported;
	bool proto_conflict_reported;
	uint64_t proto_undecided;	/* frames seen before the generation was */

	/* AgileX protocol v2 decoding, off unless asked for: on a bus that is not
	 * an AgileX vehicle, these ids mean something else entirely and named
	 * fields would be confident nonsense. */
	bool agilex;
	struct agx_state agx;
} g;

static volatile sig_atomic_t stop_requested;

static void on_signal(int sig)
{
	(void)sig;
	stop_requested = 1;
}

static void logmsg(int prio, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	if (g.foreground) {
		vfprintf(stderr, fmt, ap);
		fputc('\n', stderr);
	} else {
		vsyslog(prio, fmt, ap);
	}
	va_end(ap);
}

static uint64_t now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000);
}

static void wr32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xff);
	p[1] = (uint8_t)((v >> 8) & 0xff);
	p[2] = (uint8_t)((v >> 16) & 0xff);
	p[3] = (uint8_t)((v >> 24) & 0xff);
}

static uint32_t rd32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static struct tracked *track_find(uint32_t id)
{
	int i;

	for (i = 0; i < g.ntrack; i++)
		if (g.track[i].id == id)
			return &g.track[i];
	return NULL;
}

static void discover_note(uint32_t id)
{
	int i;

	for (i = 0; i < g.nseen; i++)
		if (g.seen_ids[i] == id)
			return;
	if (g.nseen >= MAX_TRACK)
		return;
	g.seen_ids[g.nseen++] = id;
	logmsg(LOG_NOTICE, "discovered CAN id 0x%03X (%d distinct so far)",
	       id, g.nseen);
}

/* The vendor's discriminators, and nothing else. Reporting once keeps this to a
 * single line in the log whatever the frame rate is. */
static void proto_note(uint32_t id)
{
	if (id == 0x151)
		g.proto_v1_seen = true;
	else if (id == 0x221 || id == 0x241)
		g.proto_v2_seen = true;
	else
		return;

	/* Both markers on one bus is the vendor's UNKNOWN case. Worth its own
	 * line, because it means the assumption behind either answer is broken
	 * and not that the bus is somehow both generations. */
	if (g.proto_v1_seen && g.proto_v2_seen) {
		if (!g.proto_conflict_reported) {
			g.proto_conflict_reported = true;
			logmsg(LOG_ERR,
			       "both AgileX protocol markers seen (0x151 and "
			       "0x221/0x241). ugv_sdk calls this UNKNOWN. Do not "
			       "enable inject until this is understood.");
		}
		return;
	}

	if (g.proto_reported)
		return;
	g.proto_reported = true;

	if (g.proto_v1_seen)
		logmsg(LOG_NOTICE,
		       "AgileX protocol v1 detected (0x151). Decoding as v1: "
		       "checksummed frames, and commands are percentages of the "
		       "SCOUT MINI maxima rather than mm/s. agx-cmd must be told "
		       "the same generation - it reads it from this status file.");
	else
		logmsg(LOG_NOTICE,
		       "AgileX protocol v2 detected (0x%03X). Decoding as v2.",
		       id);
}

static void track_update(const struct can_frame *f)
{
	struct tracked *t;

	proto_note(f->can_id & CAN_EFF_MASK);

	if (g.discover)
		discover_note(f->can_id & CAN_EFF_MASK);

	/*
	 * One decoder per generation, chosen by what the bus said.
	 *
	 * Nothing is decoded until the generation is known, which costs a handful
	 * of frames: 0x151 (v1) and 0x221 (v2) are both periodic at 50 Hz, so the
	 * wait is measured in tens of milliseconds. Guessing instead would be
	 * worse than waiting - on a v1 bus the v2 decoder reads the motion state
	 * as a brake command and reports velocities that were never sent, which
	 * looks like data rather than like an error.
	 */
	if (g.agilex) {
		uint32_t id = f->can_id & CAN_EFF_MASK;

		if (g.proto_v1_seen && !g.proto_v2_seen)
			agx1_decode(&g.agx, id, f->data, f->can_dlc);
		else if (g.proto_v2_seen && !g.proto_v1_seen)
			agx_decode(&g.agx, id, f->data, f->can_dlc);
		else
			g.proto_undecided++;
	}

	t = track_find(f->can_id & CAN_EFF_MASK);

	if (!t)
		return;
	t->len = f->can_dlc > 8 ? 8 : f->can_dlc;
	memcpy(t->data, f->data, t->len);
	t->count++;
	t->last_ms = now_ms();
	t->seen = true;
}

static void status_write(void)
{
	char tmp[256];
	FILE *fp;
	int i, j;

	if (!g.status_path)
		return;

	snprintf(tmp, sizeof(tmp), "%s.tmp", g.status_path);
	fp = fopen(tmp, "w");
	if (!fp)
		return;

	fprintf(fp, "{\n\t\"interface\": \"%s\",\n", g.ifname);
	fprintf(fp, "\t\"rx\": %llu,\n", (unsigned long long)g.rx);
	fprintf(fp, "\t\"tx\": %llu,\n", (unsigned long long)g.tx);
	fprintf(fp, "\t\"injected\": %llu,\n", (unsigned long long)g.injected);
	fprintf(fp, "\t\"rejected\": %llu,\n", (unsigned long long)g.rejected);
	fprintf(fp, "\t\"dropped\": %llu,\n", (unsigned long long)g.dropped);
	/* Detected generation, by the vendor's own discriminators. "unknown"
	 * covers both "nothing heard yet" and the conflicting case, which the
	 * log distinguishes; a consumer should treat either as do-not-command. */
	fprintf(fp, "\t\"agilex_protocol\": \"%s\",\n",
		(g.proto_v1_seen && g.proto_v2_seen) ? "unknown" :
		g.proto_v1_seen ? "v1" :
		g.proto_v2_seen ? "v2" : "unknown");
	/* How many frames went by before the generation was settled. A number
	 * that keeps rising means no discriminator is arriving at all, which is a
	 * different fault from a bus that is quiet. */
	fprintf(fp, "\t\"agilex_undecided\": %llu,\n",
		(unsigned long long)g.proto_undecided);
	fprintf(fp, "\t\"inject_allowed\": %s,\n",
		g.allow_inject ? "true" : "false");
	fprintf(fp, "\t\"frames\": {");
	for (i = 0, j = 0; i < g.ntrack; i++) {
		struct tracked *t = &g.track[i];
		int b;

		if (!t->seen)
			continue;
		fprintf(fp, "%s\n\t\t\"%03X\": {\"data\": \"", j++ ? "," : "",
			t->id);
		for (b = 0; b < t->len; b++)
			fprintf(fp, "%02X", t->data[b]);
		fprintf(fp, "\", \"count\": %llu, \"age_ms\": %llu}",
			(unsigned long long)t->count,
			(unsigned long long)(now_ms() - t->last_ms));
	}
	fprintf(fp, "%s}", j ? "\n\t" : "");

	if (g.agilex) {
		const struct agx_state *a = &g.agx;
		int k, first;

		fprintf(fp, ",\n\t\"agilex\": {\n");
		fprintf(fp, "\t\t\"variant\": \"%s\",\n", agx_variant(a));
		fprintf(fp, "\t\t\"decoded\": %llu,\n",
			(unsigned long long)a->decoded);
		fprintf(fp, "\t\t\"undecoded\": %llu", 
			(unsigned long long)a->unknown);
		if (a->system_valid)
			fprintf(fp, ",\n\t\t\"vehicle_state\": %u,"
				"\n\t\t\"control_mode\": %u,"
				"\n\t\t\"battery_v\": %.1f,"
				"\n\t\t\"error_code\": %u",
				a->vehicle_state, a->control_mode,
				a->battery_v, a->error_code);
		if (a->motion_valid)
			fprintf(fp, ",\n\t\t\"linear_mps\": %.3f,"
				"\n\t\t\"angular_rps\": %.3f,"
				"\n\t\t\"lateral_mps\": %.3f",
				a->linear_mps, a->angular_rps, a->lateral_mps);
		if (a->bms_valid)
			fprintf(fp, ",\n\t\t\"soc\": %u,"
				"\n\t\t\"bms_v\": %.1f,"
				"\n\t\t\"bms_a\": %.1f,"
				"\n\t\t\"bms_temp_c\": %.1f",
				a->soc, a->bms_v, a->bms_a, a->bms_temp_c);
		if (a->odom_valid)
			fprintf(fp, ",\n\t\t\"left_wheel\": %d,"
				"\n\t\t\"right_wheel\": %d",
				a->left_wheel, a->right_wheel);
		fprintf(fp, ",\n\t\t\"rpm\": [");
		for (k = 0, first = 1; k < AGX_ACTUATORS; k++) {
			if (!a->act_hs_valid[k])
				continue;
			fprintf(fp, "%s%d", first ? "" : ", ", a->rpm[k]);
			first = 0;
		}
		fprintf(fp, "],\n\t\t\"motor_temp_c\": [");
		for (k = 0, first = 1; k < AGX_ACTUATORS; k++) {
			if (!a->act_ls_valid[k])
				continue;
			fprintf(fp, "%s%d", first ? "" : ", ",
				(int)a->motor_temp_c[k]);
			first = 0;
		}
		fprintf(fp, "]\n\t}");
	}

	fprintf(fp, "\n}\n");
	fclose(fp);

	if (rename(tmp, g.status_path) < 0)
		unlink(tmp);
}

static void usage(const char *a0)
{
	fprintf(stderr,
"Usage: %s [options]\n"
"  -i, --interface IF     CAN interface (default can0)\n"
"  -r, --remote HOST:PORT send received frames here\n"
"  -l, --listen PORT      accept frames from the network on this port\n"
"      --allow-inject     actually put received frames on the bus.\n"
"                         Without this the bridge is read-only, which is the\n"
"                         right default for a vehicle bus.\n"
"  -t, --track ID[,ID...] hex CAN IDs to decode into the status file\n"
"  -A, --agilex           decode AgileX protocol v2 into named fields. Off by\n"
"                         default: on any other bus these ids mean something\n"
"                         else and named values would be confident nonsense.\n"
"  -d, --discover         log every distinct id seen. can-utils is not in the\n"
"                         OpenWrt feeds, so this is how you find out what a\n"
"                         vehicle actually emits.\n"
"  -S, --status PATH      JSON status file (default /var/run/can-bridge.json)\n"
"  -I, --status-interval MS   status rewrite interval (default 200)\n"
"  -f, --foreground       log to stderr instead of syslog\n", a0);
}

static bool parse_hostport(const char *s, struct sockaddr_in *out, int defport)
{
	char buf[128];
	char *colon;

	snprintf(buf, sizeof(buf), "%s", s);
	memset(out, 0, sizeof(*out));
	out->sin_family = AF_INET;
	out->sin_port = htons((uint16_t)defport);

	colon = strrchr(buf, ':');
	if (colon) {
		*colon = '\0';
		out->sin_port = htons((uint16_t)atoi(colon + 1));
	}
	return inet_pton(AF_INET, buf, &out->sin_addr) == 1;
}

static void parse_track(char *s)
{
	char *tok = strtok(s, ",");

	while (tok && g.ntrack < MAX_TRACK) {
		g.track[g.ntrack].id = (uint32_t)strtoul(tok, NULL, 16);
		g.track[g.ntrack].seen = false;
		g.ntrack++;
		tok = strtok(NULL, ",");
	}
}

int main(int argc, char **argv)
{
	enum { OPT_ALLOW_INJECT = 1000 };
	static const struct option opts[] = {
		{ "interface",       required_argument, NULL, 'i' },
		{ "remote",          required_argument, NULL, 'r' },
		{ "listen",          required_argument, NULL, 'l' },
		{ "allow-inject",    no_argument,       NULL, OPT_ALLOW_INJECT },
		{ "track",           required_argument, NULL, 't' },
		{ "discover",        no_argument,       NULL, 'd' },
		{ "agilex",          no_argument,       NULL, 'A' },
		{ "status",          required_argument, NULL, 'S' },
		{ "status-interval", required_argument, NULL, 'I' },
		{ "foreground",      no_argument,       NULL, 'f' },
		{ "help",            no_argument,       NULL, 'h' },
		{ NULL, 0, NULL, 0 }
	};
	struct sockaddr_can caddr;
	struct sockaddr_in uaddr;
	struct sigaction sa;
	struct pollfd pfd[2];
	struct ifreq ifr;
	uint8_t pkt[HDR + MAX_BATCH * REC];
	int cs, us = -1, opt, i;
	uint64_t last_status = 0;

	g.ifname = (char *)"can0";
	g.status_path = (char *)"/var/run/can-bridge.json";
	g.status_ms = 200;

	while ((opt = getopt_long(argc, argv, "i:r:l:t:S:I:dAfh", opts, NULL)) != -1) {
		switch (opt) {
		case 'i': g.ifname = optarg; break;
		case 'r':
			if (!parse_hostport(optarg, &g.peer, 7700)) {
				fprintf(stderr, "bad --remote '%s'\n", optarg);
				return 1;
			}
			g.have_peer = true;
			break;
		case 'l': g.listen_port = atoi(optarg); break;
		case OPT_ALLOW_INJECT: g.allow_inject = true; break;
		case 't': parse_track(optarg); break;
		case 'd': g.discover = true; break;
		case 'A': g.agilex = true; break;
		case 'S': g.status_path = optarg; break;
		case 'I': g.status_ms = atoi(optarg); break;
		case 'f': g.foreground = true; break;
		case 'h': usage(argv[0]); return 0;
		default: usage(argv[0]); return 1;
		}
	}

	if (g.status_ms < 20)
		g.status_ms = 20;

	if (!g.foreground)
		openlog("can-bridge", LOG_PID, LOG_DAEMON);

	sa.sa_handler = on_signal;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	signal(SIGPIPE, SIG_IGN);

	cs = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (cs < 0) {
		logmsg(LOG_ERR, "CAN socket: %s (is kmod-can loaded?)",
		       strerror(errno));
		return 1;
	}

	memset(&ifr, 0, sizeof(ifr));
	snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", g.ifname);
	if (ioctl(cs, SIOCGIFINDEX, &ifr) < 0) {
		logmsg(LOG_ERR, "no interface '%s': %s", g.ifname, strerror(errno));
		return 1;
	}

	memset(&caddr, 0, sizeof(caddr));
	caddr.can_family = AF_CAN;
	caddr.can_ifindex = ifr.ifr_ifindex;
	if (bind(cs, (struct sockaddr *)&caddr, sizeof(caddr)) < 0) {
		logmsg(LOG_ERR, "bind %s: %s", g.ifname, strerror(errno));
		return 1;
	}
	fcntl(cs, F_SETFL, fcntl(cs, F_GETFL, 0) | O_NONBLOCK);

	if (g.have_peer || g.listen_port) {
		us = socket(AF_INET, SOCK_DGRAM, 0);
		if (us < 0) {
			logmsg(LOG_ERR, "udp socket: %s", strerror(errno));
			return 1;
		}
		fcntl(us, F_SETFL, fcntl(us, F_GETFL, 0) | O_NONBLOCK);
		if (g.listen_port) {
			memset(&uaddr, 0, sizeof(uaddr));
			uaddr.sin_family = AF_INET;
			uaddr.sin_addr.s_addr = htonl(INADDR_ANY);
			uaddr.sin_port = htons((uint16_t)g.listen_port);
			if (bind(us, (struct sockaddr *)&uaddr, sizeof(uaddr)) < 0) {
				logmsg(LOG_ERR, "bind :%d: %s", g.listen_port,
				       strerror(errno));
				return 1;
			}
		}
	}

	logmsg(LOG_NOTICE, "%s bridged%s%s, %d tracked ids, injection %s%s",
	       g.ifname,
	       g.have_peer ? " -> udp" : "",
	       g.listen_port ? " <- udp" : "",
	       g.ntrack,
	       g.allow_inject ? "ALLOWED" : "blocked (read-only)",
	       g.discover ? ", discover on" : "");
	if (g.agilex)
		logmsg(LOG_NOTICE, "AgileX protocol v2 decoding enabled");

	while (!stop_requested) {
		int np = 0;

		pfd[np].fd = cs;
		pfd[np].events = POLLIN;
		pfd[np++].revents = 0;
		if (us >= 0 && g.listen_port) {
			pfd[np].fd = us;
			pfd[np].events = POLLIN;
			pfd[np++].revents = 0;
		}

		if (poll(pfd, (nfds_t)np, g.status_ms) < 0 && errno != EINTR)
			logmsg(LOG_WARNING, "poll: %s", strerror(errno));

		/* CAN -> network, batched so a busy bus does not turn into one
		 * datagram per frame. */
		if (pfd[0].revents & POLLIN) {
			int cnt = 0;
			size_t off = HDR;

			while (cnt < MAX_BATCH) {
				struct can_frame f;
				ssize_t r = read(cs, &f, sizeof(f));

				if (r != (ssize_t)sizeof(f))
					break;
				g.rx++;
				track_update(&f);

				wr32(pkt + off, f.can_id);
				pkt[off + 4] = f.can_dlc > 8 ? 8 : f.can_dlc;
				pkt[off + 5] = pkt[off + 6] = pkt[off + 7] = 0;
				memset(pkt + off + 8, 0, 8);
				memcpy(pkt + off + 8, f.data, pkt[off + 4]);
				off += REC;
				cnt++;
			}

			if (cnt && g.have_peer) {
				wr32(pkt, MAGIC);
				pkt[4] = VERSION;
				pkt[5] = (uint8_t)cnt;
				pkt[6] = pkt[7] = 0;
				if (sendto(us, pkt, off, 0,
					   (struct sockaddr *)&g.peer,
					   sizeof(g.peer)) < 0)
					g.dropped++;
				else
					g.tx += (uint64_t)cnt;
			}
		}

		/* network -> CAN, only if explicitly permitted */
		if (np > 1 && (pfd[1].revents & POLLIN)) {
			uint8_t in[HDR + MAX_BATCH * REC];
			ssize_t r = recv(us, in, sizeof(in), 0);

			if (r >= (ssize_t)HDR && rd32(in) == MAGIC &&
			    in[4] == VERSION) {
				int cnt = in[5];

				if ((size_t)r < HDR + (size_t)cnt * REC)
					cnt = (int)(((size_t)r - HDR) / REC);

				for (i = 0; i < cnt; i++) {
					const uint8_t *rec = in + HDR + (size_t)i * REC;
					struct can_frame f;

					if (!g.allow_inject) {
						g.rejected++;
						continue;
					}
					memset(&f, 0, sizeof(f));
					f.can_id = rd32(rec);
					f.can_dlc = rec[4] > 8 ? 8 : rec[4];
					memcpy(f.data, rec + 8, f.can_dlc);
					if (write(cs, &f, sizeof(f)) ==
					    (ssize_t)sizeof(f))
						g.injected++;
					else
						g.dropped++;
				}
			}
		}

		if (now_ms() - last_status >= (uint64_t)g.status_ms) {
			last_status = now_ms();
			status_write();
		}
	}

	logmsg(LOG_NOTICE,
	       "stopping: rx=%llu tx=%llu injected=%llu rejected=%llu",
	       (unsigned long long)g.rx, (unsigned long long)g.tx,
	       (unsigned long long)g.injected,
	       (unsigned long long)g.rejected);
	status_write();
	close(cs);
	if (us >= 0)
		close(us);
	return 0;
}
