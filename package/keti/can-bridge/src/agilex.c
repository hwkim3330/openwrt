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
