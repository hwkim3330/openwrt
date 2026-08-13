// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Decode checks for AgileX protocol v2.
 *
 * There is no vehicle on the bench, so frames are built by hand from the layouts
 * in AgileX's own SDK and the decoded physical values are asserted. That catches
 * the two mistakes this protocol invites:
 *
 *  - reading the payload little-endian. Everything else in this tree is LE, and
 *    a wrong-endian read of 24.0 V gives 6144.0 V, which at least is obvious;
 *    a wrong-endian read of 1.0 m/s gives -6.144 m/s, which is not.
 *  - confusing 0x221 (motion state) with 0x251 (first actuator). Both carry
 *    plausible small numbers, so the mix-up survives a glance at a dashboard.
 *
 *   cc -O2 -Wall -Wextra -o test_agilex test_agilex.c ../src/agilex.c && ./test_agilex
 */

#include "../src/agilex.h"

#include <stdio.h>
#include <string.h>

static int fails;

static void eqd(const char *name, double got, double want)
{
	int ok = (got > want - 1e-6) && (got < want + 1e-6);

	printf("  %s  %s: got %.4f want %.4f\n", ok ? "PASS" : "FAIL",
	       name, got, want);
	if (!ok)
		fails++;
}

static void eqi(const char *name, long got, long want)
{
	int ok = got == want;

	printf("  %s  %s: got %ld want %ld\n", ok ? "PASS" : "FAIL",
	       name, got, want);
	if (!ok)
		fails++;
}

static void eqs(const char *name, const char *got, const char *want)
{
	int ok = got && want && !strcmp(got, want);

	printf("  %s  %s: got %s\n", ok ? "PASS" : "FAIL", name,
	       got ? got : "(null)");
	if (!ok)
		fails++;
}

static void ok(const char *name, int cond)
{
	printf("  %s  %s\n", cond ? "PASS" : "FAIL", name);
	if (!cond)
		fails++;
}

/* big-endian, as the SDK's struct16_t lays it out */
static void be16(uint8_t *p, int v)
{
	p[0] = (uint8_t)((v >> 8) & 0xff);
	p[1] = (uint8_t)(v & 0xff);
}

static void be32(uint8_t *p, long v)
{
	p[0] = (uint8_t)((v >> 24) & 0xff);
	p[1] = (uint8_t)((v >> 16) & 0xff);
	p[2] = (uint8_t)((v >> 8) & 0xff);
	p[3] = (uint8_t)(v & 0xff);
}

int main(void)
{
	struct agx_state s;
	uint8_t d[8];

	printf("AgileX protocol v2 decode\n");

	/* ---- ids are the ones the SDK defines ---- */
	printf("\n--- ids ---\n");
	eqi("system state", AGX_ID_SYSTEM_STATE, 0x211);
	eqi("motion state", AGX_ID_MOTION_STATE, 0x221);
	eqi("rc state", AGX_ID_RC_STATE, 0x241);
	eqi("actuator 1 high speed", AGX_ID_ACT_HS_BASE, 0x251);
	eqi("actuator 1 low speed", AGX_ID_ACT_LS_BASE, 0x261);
	eqi("odometry", AGX_ID_ODOMETRY, 0x311);
	eqi("motion command (never sent)", AGX_ID_MOTION_CMD, 0x111);
	eqs("0x221 names motion", agx_id_name(0x221), "motion state");
	eqs("0x251 names an actuator, not motion",
	    agx_id_name(0x251), "actuator hs state");

	/* ---- system state: 24.0 V, normal, error 0 ---- */
	printf("\n--- system state 0x211 ---\n");
	memset(&s, 0, sizeof(s));
	memset(d, 0, 8);
	d[0] = 0x00;			/* vehicle state */
	d[1] = 0x01;			/* control mode: CAN */
	be16(d + 2, 240);		/* 24.0 V in 0.1 V steps */
	be16(d + 4, 0x0000);
	ok("frame accepted", agx_decode(&s, 0x211, d, 8));
	eqd("battery", s.battery_v, 24.0);
	eqi("control mode", s.control_mode, 1);
	eqi("error code", s.error_code, 0);

	/* a little-endian read would give 6144.0 - the point of the check */
	ok("not decoded little-endian", s.battery_v < 100.0);

	/* an error bitmap must survive intact */
	be16(d + 4, 0x0402);
	agx_decode(&s, 0x211, d, 8);
	eqi("error bitmap", s.error_code, 0x0402);

	/* ---- motion state: 1.0 m/s forward, -0.5 rad/s, no lateral ---- */
	printf("\n--- motion state 0x221 ---\n");
	memset(&s, 0, sizeof(s));
	memset(d, 0, 8);
	be16(d, 1000);			/* mm/s */
	be16(d + 2, -500);
	be16(d + 4, 0);
	be16(d + 6, 0);
	agx_decode(&s, 0x221, d, 8);
	eqd("linear", s.linear_mps, 1.0);
	eqd("angular", s.angular_rps, -0.5);
	eqd("lateral", s.lateral_mps, 0.0);
	eqs("variant with no lateral yet", agx_variant(&s),
	    "skid (no lateral motion observed yet)");

	/* reverse, to prove the sign survives */
	be16(d, -1500);
	agx_decode(&s, 0x221, d, 8);
	eqd("reverse", s.linear_mps, -1.5);

	/* ---- lateral motion means mecanum wheels ---- */
	printf("\n--- variant inference ---\n");
	be16(d, 0);
	be16(d + 2, 0);
	be16(d + 4, 300);		/* 0.3 m/s sideways */
	agx_decode(&s, 0x221, d, 8);
	eqd("lateral", s.lateral_mps, 0.3);
	eqs("variant after lateral", agx_variant(&s),
	    "omni (mecanum: lateral motion observed)");

	/* a motion-mode frame is the other tell */
	memset(&s, 0, sizeof(s));
	memset(d, 0, 8);
	d[0] = 1;
	agx_decode(&s, 0x291, d, 8);
	eqs("variant from a mode frame", agx_variant(&s),
	    "omni (mecanum: lateral motion observed)");
	eqi("motion mode", s.motion_mode, 1);

	memset(&s, 0, sizeof(s));
	eqs("variant before any motion frame", agx_variant(&s),
	    "unknown (no motion state yet)");

	/* ---- actuators: four of them, indexed by id offset ---- */
	printf("\n--- actuators 0x251..0x254, 0x261..0x264 ---\n");
	memset(&s, 0, sizeof(s));
	for (int i = 0; i < 4; i++) {
		memset(d, 0, 8);
		be16(d, 1000 + i * 100);	/* rpm */
		be16(d + 2, 15 + i);		/* 0.1 A steps */
		be32(d + 4, 123456 + i);
		agx_decode(&s, (uint32_t)(0x251 + i), d, 8);

		memset(d, 0, 8);
		be16(d, 240);			/* driver 24.0 V */
		be16(d + 2, 35 + i);		/* driver temp, whole degrees */
		d[4] = (uint8_t)(int8_t)(40 + i);
		d[5] = 0x00;
		agx_decode(&s, (uint32_t)(0x261 + i), d, 8);
	}
	eqi("actuator 0 rpm", s.rpm[0], 1000);
	eqi("actuator 3 rpm", s.rpm[3], 1300);
	eqd("actuator 0 current", s.motor_current_a[0], 1.5);
	eqd("actuator 3 current", s.motor_current_a[3], 1.8);
	eqi("actuator 0 pulses", s.pulse_count[0], 123456);
	eqd("actuator 2 driver volts", s.driver_v[2], 24.0);
	eqi("actuator 2 motor temp", s.motor_temp_c[2], 42);
	ok("actuator 4 untouched", !s.act_hs_valid[4] && !s.act_ls_valid[4]);

	/* a negative motor temperature must stay negative */
	memset(d, 0, 8);
	be16(d, 240);
	d[4] = (uint8_t)(int8_t)-11;
	agx_decode(&s, 0x261, d, 8);
	eqi("negative motor temp", s.motor_temp_c[0], -11);

	/* ---- BMS ---- */
	printf("\n--- bms 0x361 ---\n");
	memset(&s, 0, sizeof(s));
	memset(d, 0, 8);
	d[0] = 87;			/* soc % */
	d[1] = 99;			/* soh % */
	be16(d + 2, 253);		/* 25.3 V */
	be16(d + 4, -42);		/* -4.2 A, charging */
	be16(d + 6, 312);		/* 31.2 C */
	agx_decode(&s, 0x361, d, 8);
	eqi("soc", s.soc, 87);
	eqd("voltage", s.bms_v, 25.3);
	eqd("current signed", s.bms_a, -4.2);
	eqd("temperature", s.bms_temp_c, 31.2);

	/* ---- odometry, 32-bit and signed ---- */
	printf("\n--- odometry 0x311 ---\n");
	memset(&s, 0, sizeof(s));
	memset(d, 0, 8);
	be32(d, 100000);
	be32(d + 4, -100000);
	agx_decode(&s, 0x311, d, 8);
	eqi("left wheel", s.left_wheel, 100000);
	eqi("right wheel signed", s.right_wheel, -100000);

	/* ---- frames it must refuse ---- */
	printf("\n--- refusals ---\n");
	memset(&s, 0, sizeof(s));
	memset(d, 0, 8);
	ok("short frame refused", !agx_decode(&s, 0x211, d, 4));
	eqi("short frame counted as undecoded", s.unknown, 1);
	ok("unknown id refused", !agx_decode(&s, 0x7ff, d, 8));
	eqi("unknown id counted", s.unknown, 2);
	eqi("nothing decoded", s.decoded, 0);
	ok("unknown id has no name", agx_id_name(0x7ff) == NULL);

	printf("\n");
	if (fails) {
		printf("FAILED (%d)\n", fails);
		return 1;
	}
	printf("all checks passed\n");
	return 0;
}
