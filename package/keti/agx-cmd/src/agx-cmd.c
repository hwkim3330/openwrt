// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * agx-cmd - turn teleop intent into AgileX motion commands
 *
 * This is the last link in the control chain and the only one that can make a
 * vehicle move, so it is worth being explicit about what it does and does not
 * do.
 *
 *   teleop  --TELE/UDP-->  agx-cmd  --BCAN/UDP-->  can-bridge  --> CAN 0x111
 *
 * It does not open a CAN socket. It emits exactly the same inject datagram a
 * laptop would send, so can-bridge stays the single thing in this tree that
 * touches the bus and its read-only default keeps meaning something: without
 * `can-bridge --allow-inject` nothing here reaches the wheels, no matter what
 * this daemon is doing.
 *
 * Four things stand between a stick and the motors:
 *
 *  1. It is off unless enabled, like the injection it depends on.
 *  2. Full stick is not full speed. The defaults are walking pace, not the
 *     vehicle's maximum, because the first time this drives a real machine
 *     somebody will be standing next to it.
 *  3. A step input is ramped. Commanding 0 to full in one frame is a lurch, and
 *     on a mecanum base a lurch is sideways as easily as forwards.
 *  4. Losing the operator commands zero, repeatedly, before going quiet. The
 *     vehicle's own protocol timeout would eventually stop it, but "eventually"
 *     is the wrong word to rely on here.
 *
 * Axis mapping, from doc/TELEOP.md:
 *
 *     a0  strafe   + right
 *     a1  forward  + forward
 *     a2  yaw      + anticlockwise
 *
 * AgileX takes +lateral as left (see agilex.h), so a0 is negated. That sign is
 * the one unverified thing in this path - confirm it with the wheels clear.
 */

#define _GNU_SOURCE

/* Plain name, with -I supplying the directory: OpenWrt builds in a copied tree,
 * so a path relative to the source location does not exist at compile time. */
#include "agilex.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#define TELE_MAGIC	"TELE"
#define TELE_LEN	32
#define AXIS_SCALE	10000.0

/* can-bridge's inject wire format, from can-bridge.c. */
#define BCAN_MAGIC	0x4e414342u	/* "BCAN" little-endian */
#define BCAN_VERSION	1
#define BCAN_HDR	8
#define BCAN_REC	16

enum { PROTO_AUTO, PROTO_V1, PROTO_V2 };

static struct {
	int listen_port;		/* where teleop's TELE frames arrive */
	struct sockaddr_in inject_to;	/* can-bridge --listen */
	bool have_inject;

	double max_linear;		/* m/s at full stick */
	double max_lateral;		/* m/s */
	double max_angular;		/* rad/s */
	double accel;			/* m/s^2 and rad/s^2 slew limit */
	bool lateral_invert;

	int rate_hz;
	int deadman_ms;
	int zero_frames;		/* explicit stops after losing the link */

	const char *status_path;
	bool foreground;
	bool enabled;

	/*
	 * Which protocol generation to speak.
	 *
	 * This daemon only transmits, so it cannot detect anything itself - it
	 * would have to put a frame on the bus to find out, which is exactly what
	 * must not happen before the generation is known. can-bridge is already
	 * listening and already writes what it heard, so the answer is read from
	 * there rather than guessed or configured.
	 *
	 * AUTO with an unreadable or undecided bridge status sends nothing at all.
	 * The alternative - defaulting to a generation - means the first command
	 * to a vehicle of the other kind is either a checksum-less frame a v1
	 * vehicle ignores, or a v2-shaped frame whose bytes a v1 vehicle reads as
	 * percentages of its 3 m/s maximum. One of those two mistakes drives away.
	 */
	int proto;			/* PROTO_AUTO until settled, then V1 or V2 */
	const char *bridge_status;
	uint64_t proto_unknown_skips;
	bool proto_said;
	uint8_t v1_count;		/* v1's rolling frame counter */

	/* live state */
	double cur_lin, cur_lat, cur_ang;
	double want_lin, want_lat, want_ang;
	bool armed;
	uint32_t seq_in;
	uint64_t last_cmd_ms;
	uint64_t rx, applied, rejected_seq, malformed, sent, stops;
	int zeros_left;
} g;

static volatile sig_atomic_t stop_flag;

static void on_signal(int sig)
{
	(void)sig;
	stop_flag = 1;
}

static uint64_t now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000);
}

static void logmsg(int level, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	if (g.foreground) {
		vfprintf(stderr, fmt, ap);
		fputc('\n', stderr);
	} else {
		vsyslog(level, fmt, ap);
	}
	va_end(ap);
}

static void wr32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xff);
	p[1] = (uint8_t)((v >> 8) & 0xff);
	p[2] = (uint8_t)((v >> 16) & 0xff);
	p[3] = (uint8_t)((v >> 24) & 0xff);
}

/*
 * Read the generation can-bridge heard.
 *
 * A substring search, not a JSON parse. The field is written by one printf in
 * can-bridge.c with a fixed shape, both files are in this tree, and the test
 * pins the shape - so a parser would be more code for the same guarantee. If
 * the string ever changes the match fails, which stops commands rather than
 * sending the wrong ones.
 */
static int read_bridge_proto(void)
{
	char buf[4096];
	FILE *fp;
	size_t n;

	if (!g.bridge_status)
		return PROTO_AUTO;
	fp = fopen(g.bridge_status, "r");
	if (!fp)
		return PROTO_AUTO;
	n = fread(buf, 1, sizeof(buf) - 1, fp);
	fclose(fp);
	buf[n] = '\0';

	if (strstr(buf, "\"agilex_protocol\": \"v1\""))
		return PROTO_V1;
	if (strstr(buf, "\"agilex_protocol\": \"v2\""))
		return PROTO_V2;
	return PROTO_AUTO;
}

/*
 * Settle the generation, once.
 *
 * Re-read while unknown, then stop: a bus does not change generation while the
 * daemon runs, and a value that could flip mid-drive would change what every
 * subsequent command byte means.
 */
static void resolve_proto(void)
{
	if (g.proto != PROTO_AUTO)
		return;
	g.proto = read_bridge_proto();
	if (g.proto == PROTO_AUTO)
		return;
	logmsg(LOG_NOTICE, "protocol %s, from %s",
	       g.proto == PROTO_V1 ? "v1" : "v2", g.bridge_status);
}

/* One motion command wrapped in can-bridge's inject datagram. */
static void emit(int sock)
{
	uint8_t pkt[BCAN_HDR + BCAN_REC];
	uint8_t payload[8];
	uint32_t id;

	if (!g.have_inject || sock < 0)
		return;

	resolve_proto();
	if (g.proto == PROTO_AUTO) {
		g.proto_unknown_skips++;
		if (!g.proto_said) {
			g.proto_said = true;
			/*
			 * Detection in can-bridge is not gated on --agilex; that
			 * flag only enables decoding. So this state means one of:
			 * can-bridge is not running, its --status path is not the
			 * one being read here, or the vehicle is not powered and
			 * no discriminator frame has arrived.
			 */
			logmsg(LOG_WARNING,
			       "not commanding: nothing in %s says which AgileX "
			       "generation this bus is. Check can-bridge is running "
			       "and writing there, and that the vehicle is on. Pass "
			       "--protocol v1|v2 to override.",
			       g.bridge_status ? g.bridge_status : "(no status path)");
		}
		return;
	}

	if (g.proto == PROTO_V1) {
		/*
		 * v1 wants a fraction of the vehicle's maximum, so the divisors
		 * are the vehicle's - AGX1_MINI_MAX_*, not this daemon's own
		 * --max-linear. Those two are different things: the -L defaults
		 * are walking pace and already applied to cur_lin above, and
		 * dividing by them here would turn walking pace back into full
		 * speed.
		 */
		id = AGX1_ID_MOTION_CMD;
		agx1_encode_motion(payload, g.cur_lin, g.cur_ang, g.cur_lat,
				   AGX1_MINI_MAX_LINEAR, AGX1_MINI_MAX_ANGULAR,
				   AGX1_MINI_MAX_LATERAL, g.v1_count++);
	} else {
		id = AGX_ID_MOTION_CMD;
		agx_encode_motion(payload, g.cur_lin, g.cur_ang, g.cur_lat);
	}

	memset(pkt, 0, sizeof(pkt));
	wr32(pkt, BCAN_MAGIC);
	pkt[4] = BCAN_VERSION;
	pkt[5] = 1;				/* one frame in this batch */
	wr32(pkt + BCAN_HDR, id);
	pkt[BCAN_HDR + 4] = 8;			/* dlc */
	memcpy(pkt + BCAN_HDR + 8, payload, 8);

	if (sendto(sock, pkt, sizeof(pkt), 0,
		   (struct sockaddr *)&g.inject_to,
		   sizeof(g.inject_to)) == (ssize_t)sizeof(pkt))
		g.sent++;
}

/*
 * Ramp toward the request rather than jumping to it.
 *
 * Applied per axis independently, which is right for a mecanum base: the axes
 * are physically independent, so limiting their vector sum would make a diagonal
 * slower than a straight line for no reason the driver could predict.
 */
static double slew(double cur, double want, double step)
{
	double d = want - cur;

	if (d > step)
		return cur + step;
	if (d < -step)
		return cur - step;
	return want;
}

static void neutral_request(void)
{
	g.want_lin = 0.0;
	g.want_lat = 0.0;
	g.want_ang = 0.0;
}

static void handle_tele(const uint8_t *p, size_t len)
{
	int16_t a[4];
	uint32_t seq;
	int i;

	if (len < TELE_LEN || memcmp(p, TELE_MAGIC, 4) || p[4] != 1) {
		g.malformed++;
		return;
	}
	g.rx++;

	seq = (uint32_t)p[8] | ((uint32_t)p[9] << 8) |
	      ((uint32_t)p[10] << 16) | ((uint32_t)p[11] << 24);

	/* Reject replays, but treat a large backwards jump as the sender having
	 * restarted rather than locking it out for good. Same rule as teleop's
	 * own input, and for the same reason. */
	if (g.applied && seq <= g.seq_in) {
		if (g.seq_in - seq <= 1000) {
			g.rejected_seq++;
			return;
		}
		logmsg(LOG_NOTICE, "teleop restarted (seq %u -> %u)",
		       g.seq_in, seq);
	}
	g.seq_in = seq;

	for (i = 0; i < 4; i++)
		a[i] = (int16_t)((uint16_t)p[20 + i * 2] |
				 ((uint16_t)p[21 + i * 2] << 8));

	g.armed = (p[5] & 1) != 0;
	g.last_cmd_ms = now_ms();
	g.applied++;

	if (!g.armed) {
		neutral_request();
		return;
	}

	{
		double strafe = a[0] / AXIS_SCALE;
		double fwd = a[1] / AXIS_SCALE;
		double yaw = a[2] / AXIS_SCALE;

		if (strafe > 1.0) strafe = 1.0;
		if (strafe < -1.0) strafe = -1.0;
		if (fwd > 1.0) fwd = 1.0;
		if (fwd < -1.0) fwd = -1.0;
		if (yaw > 1.0) yaw = 1.0;
		if (yaw < -1.0) yaw = -1.0;

		g.want_lin = fwd * g.max_linear;
		g.want_ang = yaw * g.max_angular;
		/* a0 is +right; AgileX takes +lateral as left. */
		g.want_lat = (g.lateral_invert ? strafe : -strafe) *
			     g.max_lateral;
	}
}

static void status_write(void)
{
	char tmp[256];
	FILE *f;

	if (!g.status_path)
		return;

	snprintf(tmp, sizeof(tmp), "%s.tmp", g.status_path);
	f = fopen(tmp, "w");
	if (!f)
		return;

	fprintf(f, "{\n");
	fprintf(f, "\t\"enabled\": %s,\n", g.enabled ? "true" : "false");
	fprintf(f, "\t\"armed\": %s,\n", g.armed ? "true" : "false");
	fprintf(f, "\t\"age_ms\": %llu,\n",
		(unsigned long long)(g.last_cmd_ms ? now_ms() - g.last_cmd_ms : 0));
	fprintf(f, "\t\"deadman_ms\": %d,\n", g.deadman_ms);
	fprintf(f, "\t\"linear_mps\": %.3f,\n", g.cur_lin);
	fprintf(f, "\t\"lateral_mps\": %.3f,\n", g.cur_lat);
	fprintf(f, "\t\"angular_rps\": %.3f,\n", g.cur_ang);
	fprintf(f, "\t\"limits\": { \"linear\": %.2f, \"lateral\": %.2f, "
		   "\"angular\": %.2f, \"accel\": %.2f },\n",
		g.max_linear, g.max_lateral, g.max_angular, g.accel);
	fprintf(f, "\t\"lateral_invert\": %s,\n",
		g.lateral_invert ? "true" : "false");
	/*
	 * Which generation is going on the wire, and how many commands were
	 * withheld because nobody had said yet. A rising skip count with a
	 * connected operator is the one failure this design introduces, so it is
	 * worth being able to see it rather than wondering why the vehicle is
	 * still.
	 */
	fprintf(f, "\t\"protocol\": \"%s\",\n",
		g.proto == PROTO_V1 ? "v1" :
		g.proto == PROTO_V2 ? "v2" : "unknown");
	fprintf(f, "\t\"protocol_skips\": %llu,\n",
		(unsigned long long)g.proto_unknown_skips);
	fprintf(f, "\t\"rx\": %llu,\n", (unsigned long long)g.rx);
	fprintf(f, "\t\"applied\": %llu,\n", (unsigned long long)g.applied);
	fprintf(f, "\t\"rejected_seq\": %llu,\n",
		(unsigned long long)g.rejected_seq);
	fprintf(f, "\t\"malformed\": %llu,\n", (unsigned long long)g.malformed);
	fprintf(f, "\t\"deadman_stops\": %llu,\n", (unsigned long long)g.stops);
	fprintf(f, "\t\"sent\": %llu\n", (unsigned long long)g.sent);
	fprintf(f, "}\n");
	fclose(f);
	rename(tmp, g.status_path);
}

static bool parse_hostport(const char *s, struct sockaddr_in *out, int defport)
{
	char buf[128], *colon;

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

static void usage(const char *me)
{
	printf(
"Usage: %s [options]\n"
"\n"
"Turns teleop TELE frames into AgileX MotionCommand (0x111) frames and hands\n"
"them to can-bridge's inject port. Does not open a CAN socket; can-bridge still\n"
"has to be running with --allow-inject for anything to reach the bus.\n"
"\n"
"  -l, --listen PORT        where TELE frames arrive (default 7722)\n"
"  -i, --inject HOST:PORT   can-bridge --listen (no default; required to emit)\n"
"  -L, --max-linear MPS     speed at full stick (default 0.5)\n"
"  -X, --max-lateral MPS    strafe speed at full stick (default 0.5)\n"
"  -A, --max-angular RPS    yaw rate at full stick (default 0.8)\n"
"  -a, --accel PER_S2       slew limit (default 1.0)\n"
"      --lateral-invert     flip the strafe sign, if the vehicle goes the\n"
"                           wrong way; see agilex.h on why this is a flag\n"
"  -H, --rate HZ            command rate (default 50, which is the rate\n"
"                           AgileX's own SDK requires; see doc/CAN.md)\n"
"  -t, --deadman MS         neutral and stop after this long without a frame\n"
"                           (default 300)\n"
"  -z, --zero-frames N      explicit zero commands after a loss (default 10)\n"
"      --protocol WHICH     auto|v1|v2 (default auto: read what can-bridge\n"
"                           heard. Neither generation is assumed - if the\n"
"                           bus has not said, nothing is commanded)\n"
"      --bridge-status PATH can-bridge's status json, for auto\n"
"                           (default /var/run/can-bridge.json)\n"
"  -S, --status PATH        write a status json here\n"
"  -f, --foreground         log to stderr\n"
"  -h, --help\n", me);
}

enum { OPT_LATERAL_INVERT = 1000, OPT_PROTOCOL, OPT_BRIDGE_STATUS };

int main(int argc, char **argv)
{
	static const struct option opts[] = {
		{ "listen",         required_argument, NULL, 'l' },
		{ "inject",         required_argument, NULL, 'i' },
		{ "max-linear",     required_argument, NULL, 'L' },
		{ "max-lateral",    required_argument, NULL, 'X' },
		{ "max-angular",    required_argument, NULL, 'A' },
		{ "accel",          required_argument, NULL, 'a' },
		{ "lateral-invert", no_argument,       NULL, OPT_LATERAL_INVERT },
		{ "protocol",       required_argument, NULL, OPT_PROTOCOL },
		{ "bridge-status",  required_argument, NULL, OPT_BRIDGE_STATUS },
		{ "rate",           required_argument, NULL, 'H' },
		{ "deadman",        required_argument, NULL, 't' },
		{ "zero-frames",    required_argument, NULL, 'z' },
		{ "status",         required_argument, NULL, 'S' },
		{ "foreground",     no_argument,       NULL, 'f' },
		{ "help",           no_argument,       NULL, 'h' },
		{ NULL, 0, NULL, 0 }
	};
	struct sockaddr_in a;
	struct sigaction sa;
	struct pollfd pfd;
	uint64_t next_tx, next_status;
	int rfd, tfd, c;

	g.listen_port = 7722;
	g.max_linear = 0.5;
	g.max_lateral = 0.5;
	g.max_angular = 0.8;
	g.accel = 1.0;
	/*
	 * 50 Hz, because that is what the vehicle is documented to want.
	 *
	 * ugv_sdk's SendMotionCommand carries "must be called at a frequency >=
	 * 50Hz" directly above it. This ran at 20 Hz, which satisfies any
	 * plausible command timeout and therefore looked fine, but "looked fine"
	 * is not a reason to sit below a stated requirement when the cost of
	 * meeting it is 50 eight-byte frames a second on a bus with room to
	 * spare - and the cost of being wrong is a vehicle that stutters or
	 * stops while someone is walking beside it.
	 */
	g.rate_hz = 50;
	g.deadman_ms = 300;
	g.zero_frames = 10;
	g.proto = PROTO_AUTO;
	g.bridge_status = "/var/run/can-bridge.json";

	while ((c = getopt_long(argc, argv, "l:i:L:X:A:a:H:t:z:S:fh",
			        opts, NULL)) != -1) {
		switch (c) {
		case 'l': g.listen_port = atoi(optarg); break;
		case 'i':
			if (!parse_hostport(optarg, &g.inject_to, 7701)) {
				fprintf(stderr, "bad --inject '%s'\n", optarg);
				return 1;
			}
			g.have_inject = true;
			break;
		case 'L': g.max_linear = atof(optarg); break;
		case 'X': g.max_lateral = atof(optarg); break;
		case 'A': g.max_angular = atof(optarg); break;
		case 'a': g.accel = atof(optarg); break;
		case OPT_LATERAL_INVERT: g.lateral_invert = true; break;
		case 'H': g.rate_hz = atoi(optarg); break;
		case 't': g.deadman_ms = atoi(optarg); break;
		case 'z': g.zero_frames = atoi(optarg); break;
		case OPT_PROTOCOL:
			if (!strcmp(optarg, "v1")) {
				g.proto = PROTO_V1;
			} else if (!strcmp(optarg, "v2")) {
				g.proto = PROTO_V2;
			} else if (!strcmp(optarg, "auto")) {
				g.proto = PROTO_AUTO;
			} else {
				fprintf(stderr, "--protocol takes auto, v1 or v2\n");
				return 2;
			}
			break;
		case OPT_BRIDGE_STATUS: g.bridge_status = optarg; break;
		case 'S': g.status_path = optarg; break;
		case 'f': g.foreground = true; break;
		case 'h': usage(argv[0]); return 0;
		default: usage(argv[0]); return 1;
		}
	}

	if (g.rate_hz < 1 || g.rate_hz > 200) {
		fprintf(stderr, "--rate must be 1..200\n");
		return 1;
	}
	/* A deadman shorter than two command intervals would trip on ordinary
	 * scheduling jitter. */
	if (g.deadman_ms < 2 * 1000 / g.rate_hz)
		g.deadman_ms = 2 * 1000 / g.rate_hz;
	if (g.max_linear < 0 || g.max_lateral < 0 || g.max_angular < 0 ||
	    g.accel <= 0) {
		fprintf(stderr, "limits must be positive\n");
		return 1;
	}
	g.enabled = g.have_inject;

	if (!g.foreground)
		openlog("agx-cmd", LOG_PID, LOG_DAEMON);

	/* sigaction, not signal(): glibc and musl both install handlers with
	 * SA_RESTART, which would silently restart the poll below and leave
	 * SIGTERM unable to stop a daemon that is commanding a vehicle. */
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = on_signal;
	sa.sa_flags = 0;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	signal(SIGPIPE, SIG_IGN);

	rfd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
	if (rfd < 0) {
		logmsg(LOG_ERR, "socket: %s", strerror(errno));
		return 1;
	}
	memset(&a, 0, sizeof(a));
	a.sin_family = AF_INET;
	a.sin_addr.s_addr = htonl(INADDR_ANY);
	a.sin_port = htons((uint16_t)g.listen_port);
	if (bind(rfd, (struct sockaddr *)&a, sizeof(a)) < 0) {
		logmsg(LOG_ERR, "bind :%d: %s", g.listen_port, strerror(errno));
		return 1;
	}
	/*
	 * Non-blocking, or the drain loop below stops being a drain: after the
	 * queued datagrams are read, recv() blocks waiting for the next one and
	 * the command cadence, the slew and the deadman all stop running until
	 * a frame happens to arrive. The symptom is a daemon that emits nothing
	 * while armed and then emits once on SIGTERM, because the signal is the
	 * only thing that interrupts the wait.
	 */
	fcntl(rfd, F_SETFL, fcntl(rfd, F_GETFL, 0) | O_NONBLOCK);

	tfd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);

	logmsg(LOG_NOTICE,
	       "agx-cmd on :%d, %d Hz, deadman %d ms, limits %.2f/%.2f m/s %.2f rad/s, %s",
	       g.listen_port, g.rate_hz, g.deadman_ms,
	       g.max_linear, g.max_lateral, g.max_angular,
	       g.have_inject ? "injecting" : "NOT injecting (no --inject)");

	next_tx = now_ms();
	next_status = now_ms();

	while (!stop_flag) {
		uint64_t t;
		int wait_ms;

		pfd.fd = rfd;
		pfd.events = POLLIN;
		pfd.revents = 0;

		t = now_ms();
		wait_ms = next_tx > t ? (int)(next_tx - t) : 0;
		if (wait_ms > 20)
			wait_ms = 20;

		if (poll(&pfd, 1, wait_ms) < 0 && errno != EINTR)
			logmsg(LOG_WARNING, "poll: %s", strerror(errno));

		if (pfd.revents & POLLIN) {
			uint8_t buf[128];
			ssize_t n;

			while ((n = recv(rfd, buf, sizeof(buf), 0)) > 0)
				handle_tele(buf, (size_t)n);
		}

		t = now_ms();

		/* Deadman. Note the request goes neutral but the ramp still
		 * runs, so the vehicle decelerates under the same slew limit it
		 * accelerated with rather than being told to stop dead. */
		if (g.last_cmd_ms &&
		    t - g.last_cmd_ms > (uint64_t)g.deadman_ms) {
			if (g.armed || g.want_lin != 0.0 || g.want_lat != 0.0 ||
			    g.want_ang != 0.0) {
				logmsg(LOG_WARNING,
				       "deadman: no teleop for %llu ms, stopping",
				       (unsigned long long)(t - g.last_cmd_ms));
				g.stops++;
				g.zeros_left = g.zero_frames;
			}
			g.armed = false;
			neutral_request();
		}

		if (t >= next_tx) {
			double step = g.accel / (double)g.rate_hz;

			next_tx = t + (uint64_t)(1000 / g.rate_hz);

			g.cur_lin = slew(g.cur_lin, g.want_lin, step);
			g.cur_lat = slew(g.cur_lat, g.want_lat, step);
			g.cur_ang = slew(g.cur_ang, g.want_ang, step);

			/*
			 * Emit while armed, while still ramping down, or for a
			 * few frames after a loss. Then go quiet: a silent bus
			 * is what the vehicle's own timeout expects, and holding
			 * a zero command open for ever would mask a dead link.
			 */
			if (g.armed || g.cur_lin != 0.0 || g.cur_lat != 0.0 ||
			    g.cur_ang != 0.0) {
				emit(tfd);
			} else if (g.zeros_left > 0) {
				g.zeros_left--;
				emit(tfd);
			}
		}

		if (t >= next_status) {
			next_status = t + 200;
			status_write();
		}
	}

	/* Whatever else happens on the way out, the last thing on the wire is a
	 * stop. */
	g.cur_lin = g.cur_lat = g.cur_ang = 0.0;
	g.armed = false;
	for (c = 0; c < 3; c++)
		emit(tfd);

	logmsg(LOG_NOTICE, "stopping: %llu applied, %llu sent, %llu deadman stops",
	       (unsigned long long)g.applied, (unsigned long long)g.sent,
	       (unsigned long long)g.stops);
	status_write();
	if (tfd >= 0)
		close(tfd);
	close(rfd);
	return 0;
}
