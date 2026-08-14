// SPDX-License-Identifier: GPL-2.0-or-later
/* See agilex.h for where these numbers come from. */

#include "agilex.h"

#include <string.h>

/* Big-endian on the wire: the SDK's struct16_t is {high_byte, low_byte}. */
static int16_t be16s(const uint8_t *p)
{
	return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint16_t be16u(const uint8_t *p)
{
	return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static int32_t be32s(const uint8_t *p)
{
	return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
			 ((uint32_t)p[2] << 8) | p[3]);
}

/* Saturating convert to a big-endian int16 at a given scale. */
static void be16_sat(uint8_t *p, double v, double scale)
{
	double s = v * scale;
	long n;

	/* Round half away from zero, then clamp. Truncation would bias every
	 * command toward zero, which is harmless, but wrapping would not be. */
	n = (long)(s < 0 ? s - 0.5 : s + 0.5);
	if (n > 32767)
		n = 32767;
	if (n < -32768)
		n = -32768;

	p[0] = (uint8_t)(((uint16_t)(int16_t)n >> 8) & 0xff);
	p[1] = (uint8_t)((uint16_t)(int16_t)n & 0xff);
}

/* ------------------------------------------------------- protocol v1 ---- */

uint8_t agx1_checksum(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
	uint8_t sum = (uint8_t)(can_id & 0xff) + (uint8_t)((can_id >> 8) & 0xff)
		    + dlc;
	int i;

	/* Every byte but the last, which is where the result goes. */
	for (i = 0; i + 1 < dlc; i++)
		sum += data[i];
	return sum;
}

bool agx1_decode(struct agx_state *s, uint32_t can_id, const uint8_t *data,
		 uint8_t dlc)
{
	/*
	 * Both rejections below count as unknown rather than returning quietly.
	 * The reason is the case this decoder exists for: if the vehicle turns
	 * out to be v2 and this is what is running, every frame fails the
	 * checksum - and an early return would leave decoded and unknown both at
	 * zero, which reads as a bus with nothing on it. Counting them makes
	 * "wrong generation" look different from "no traffic", which is the whole
	 * question the first minute on real hardware has to answer.
	 */
	if (dlc != 8) {
		s->unknown++;
		return false;
	}
	/* A wrong checksum means the frame is not to be trusted, not that it is
	 * to be decoded with a warning. */
	if (agx1_checksum(can_id, data, dlc) != data[7]) {
		s->unknown++;
		return false;
	}

	switch (can_id) {
	case AGX1_ID_MOTION_STATE:
		/* Big-endian int16 in thousandths, same as v2's state frame. */
		s->linear_mps = be16s(data + 0) / 1000.0;
		s->angular_rps = be16s(data + 2) / 1000.0;
		s->lateral_mps = be16s(data + 4) / 1000.0;
		s->steering_rad = 0.0;		/* unused on a mecanum base */
		s->motion_valid = true;
		if (s->lateral_mps != 0.0)
			s->lateral_seen = true;
		s->decoded++;
		return true;
	case AGX1_ID_SYSTEM_STATE:
		s->vehicle_state = data[0];
		s->control_mode = data[1];
		s->battery_v = ((data[2] << 8) | data[3]) / 10.0;
		s->error_code = (uint16_t)((data[4] << 8) | data[5]);
		s->system_valid = true;
		s->decoded++;
		return true;
	case AGX1_ID_LIGHT_STATE:
	case AGX1_ID_VALUE_SET_STATE:
		/*
		 * Recognised and deliberately not parsed - counted as decoded
		 * anyway, because the number that matters here is "does this
		 * decoder understand the bus". A healthy v1 vehicle sends these
		 * periodically, and letting them raise `undecoded` would make
		 * doc/CAN.md's own rule - undecoded climbing while decoded stays
		 * flat means the wrong generation - fire on a bus that is in fact
		 * the right one.
		 *
		 * Note 0x211 is the *value set* state here and the *system* state
		 * in v2, which is the collision that makes reading a frame without
		 * knowing the generation a way to publish a battery voltage that
		 * was never sent.
		 */
		s->decoded++;
		return true;
	default:
		if (can_id >= AGX1_ID_ACT_STATE_BASE &&
		    can_id <= AGX1_ID_ACT_STATE_BASE + 3) {
			s->decoded++;
			return true;
		}
		s->unknown++;
		return false;
	}
}

/* Round half away from zero, then clamp to a signed percentage. */
static int8_t pct_sat(double v, double max)
{
	double f;
	long p;

	if (max <= 0.0)
		return 0;
	f = v / max * 100.0;
	p = (long)(f >= 0 ? f + 0.5 : f - 0.5);
	if (p > 100)
		p = 100;
	if (p < -100)
		p = -100;
	return (int8_t)p;
}

void agx1_encode_motion(uint8_t out[8], double linear_mps, double angular_rps,
			double lateral_mps, double max_linear_mps,
			double max_angular_rps, double max_lateral_mps,
			uint8_t count)
{
	memset(out, 0, 8);
	out[0] = AGX1_CTRL_MODE_CAN;
	out[1] = AGX1_ERROR_CLR_NONE;
	out[2] = (uint8_t)pct_sat(linear_mps, max_linear_mps);
	out[3] = (uint8_t)pct_sat(angular_rps, max_angular_rps);
	/*
	 * Lateral, with the same caveat as the v2 encoder: which way is positive
	 * is the one thing that cannot be settled from a document. Confirm it
	 * with the wheels off the ground and use the bridge's lateral_invert
	 * rather than editing this.
	 */
	out[4] = (uint8_t)pct_sat(lateral_mps, max_lateral_mps);
	out[5] = 0;
	out[6] = count;
	out[7] = agx1_checksum(AGX1_ID_MOTION_CMD, out, 8);
}

void agx_encode_motion(uint8_t out[8], double linear_mps, double angular_rps,
		       double lateral_mps)
{
	be16_sat(out + 0, linear_mps, 1000.0);		/* mm/s   */
	be16_sat(out + 2, angular_rps, 1000.0);		/* mrad/s */
	be16_sat(out + 4, lateral_mps, 1000.0);		/* mm/s   */
	be16_sat(out + 6, 0.0, 1000.0);			/* steering angle: unused
							 * on a mecanum base */
}

const char *agx_id_name(uint32_t id)
{
	switch (id) {
	case AGX_ID_MOTION_CMD:		return "motion command";
	case AGX_ID_LIGHT_CMD:		return "light command";
	case AGX_ID_BRAKE_CMD:		return "brake command";
	case AGX_ID_MODE_CMD:		return "mode command";
	case AGX_ID_SYSTEM_STATE:	return "system state";
	case AGX_ID_MOTION_STATE:	return "motion state";
	case AGX_ID_LIGHT_STATE:	return "light state";
	case AGX_ID_RC_STATE:		return "rc state";
	case AGX_ID_MODE_STATE:		return "motion mode state";
	case AGX_ID_ODOMETRY:		return "odometry";
	case AGX_ID_IMU_ACCEL:		return "imu accel";
	case AGX_ID_IMU_GYRO:		return "imu gyro";
	case AGX_ID_IMU_EULER:		return "imu euler";
	case AGX_ID_BUMPER:		return "safety bumper";
	case AGX_ID_BMS_BASIC:		return "bms basic";
	case AGX_ID_BMS_EXTENDED:	return "bms extended";
	default:			break;
	}
	if (id >= AGX_ID_ACT_HS_BASE && id < AGX_ID_ACT_HS_BASE + AGX_ACTUATORS)
		return "actuator hs state";
	if (id >= AGX_ID_ACT_LS_BASE && id < AGX_ID_ACT_LS_BASE + AGX_ACTUATORS)
		return "actuator ls state";
	return NULL;
}

const char *agx_variant(const struct agx_state *s)
{
	if (s->lateral_seen || s->mode_frame_seen)
		return "omni (mecanum: lateral motion observed)";
	if (s->motion_valid)
		return "skid (no lateral motion observed yet)";
	return "unknown (no motion state yet)";
}

bool agx_decode(struct agx_state *s, uint32_t id, const uint8_t *d, uint8_t len)
{
	int i;

	/* Every frame this protocol defines is 8 bytes. A short one is either a
	 * different protocol on the same bus or corruption; either way, decoding
	 * it would invent values. */
	if (len < 8) {
		s->unknown++;
		return false;
	}

	if (id >= AGX_ID_ACT_HS_BASE && id < AGX_ID_ACT_HS_BASE + AGX_ACTUATORS) {
		i = (int)(id - AGX_ID_ACT_HS_BASE);
		s->rpm[i] = be16s(d);
		s->motor_current_a[i] = be16s(d + 2) * 0.1;
		s->pulse_count[i] = be32s(d + 4);
		s->act_hs_valid[i] = true;
		s->decoded++;
		return true;
	}

	if (id >= AGX_ID_ACT_LS_BASE && id < AGX_ID_ACT_LS_BASE + AGX_ACTUATORS) {
		i = (int)(id - AGX_ID_ACT_LS_BASE);
		s->driver_v[i] = be16u(d) * 0.1;
		s->driver_temp_c[i] = be16s(d + 2);
		s->motor_temp_c[i] = (int8_t)d[4];
		s->driver_state[i] = d[5];
		s->act_ls_valid[i] = true;
		s->decoded++;
		return true;
	}

	switch (id) {
	case AGX_ID_SYSTEM_STATE:
		s->vehicle_state = d[0];
		s->control_mode = d[1];
		s->battery_v = be16s(d + 2) * 0.1;
		s->error_code = be16u(d + 4);
		s->system_valid = true;
		break;

	case AGX_ID_MOTION_STATE:
		/* mm/s and mrad/s on the wire */
		s->linear_mps = be16s(d) / 1000.0;
		s->angular_rps = be16s(d + 2) / 1000.0;
		s->lateral_mps = be16s(d + 4) / 1000.0;
		s->steering_rad = be16s(d + 6) / 1000.0;
		s->motion_valid = true;
		if (be16s(d + 4) != 0)
			s->lateral_seen = true;
		break;

	case AGX_ID_MODE_STATE:
		s->motion_mode = d[0];
		s->mode_changing = d[1];
		s->mode_valid = true;
		s->mode_frame_seen = true;
		break;

	case AGX_ID_BMS_BASIC:
		s->soc = d[0];
		s->soh = d[1];
		s->bms_v = be16s(d + 2) * 0.1;
		s->bms_a = be16s(d + 4) * 0.1;
		s->bms_temp_c = be16s(d + 6) * 0.1;
		s->bms_valid = true;
		break;

	case AGX_ID_ODOMETRY:
		s->left_wheel = be32s(d);
		s->right_wheel = be32s(d + 4);
		s->odom_valid = true;
		break;

	default:
		/* Recognised by name but not decoded, or not ours at all. Either
		 * way the raw bytes are still reported, so nothing is lost. */
		s->unknown++;
		return false;
	}

	s->decoded++;
	return true;
}
