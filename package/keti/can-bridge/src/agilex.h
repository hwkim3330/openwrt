/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * agilex - decode AgileX protocol v2 CAN frames into named values
 *
 * IDs, field layouts and scale factors are taken from AgileX's own SDK, not from
 * a forum post: src/protocol_v2/agilex_protocol_v2.h for the layouts and
 * src/protocol_v2/agilex_msg_parser_v2.c for the scaling, in
 * github.com/agilexrobotics/ugv_sdk.
 *
 * Two things that are easy to get wrong and were checked against that source:
 *
 *  - The payload is BIG-endian. The SDK's struct16_t is {high_byte, low_byte},
 *    so a value spans two bytes most-significant first. Everything else in this
 *    tree is little-endian, which makes this the obvious place to slip.
 *
 *  - 0x251 is not motion feedback. Motion state is 0x221; 0x251 is the first
 *    actuator's high-speed state. Getting those two confused yields plausible
 *    numbers that are wrong, which is the worst kind.
 *
 * Decoding is the bulk of this file. There is one encoder, agx_encode_motion(),
 * for the single command this tree ever sends; see doc/CAN.md for why commanding
 * a vehicle from the router is off unless it is explicitly turned on.
 */

#ifndef AGILEX_H
#define AGILEX_H

#include <stdint.h>
#include <stdbool.h>

/* Control group 0x1 - listed for recognition, never emitted by this daemon. */
#define AGX_ID_MOTION_CMD	0x111
#define AGX_ID_LIGHT_CMD	0x121
#define AGX_ID_BRAKE_CMD	0x131
#define AGX_ID_MODE_CMD		0x141

/* State feedback group 0x2 */
#define AGX_ID_SYSTEM_STATE	0x211
#define AGX_ID_MOTION_STATE	0x221
#define AGX_ID_LIGHT_STATE	0x231
#define AGX_ID_RC_STATE		0x241
#define AGX_ID_ACT_HS_BASE	0x251	/* 0x251..0x258, one per actuator */
#define AGX_ID_ACT_LS_BASE	0x261	/* 0x261..0x268 */
#define AGX_ID_MODE_STATE	0x291

/* Sensors group 0x3 */
#define AGX_ID_ODOMETRY		0x311
#define AGX_ID_IMU_ACCEL	0x321
#define AGX_ID_IMU_GYRO		0x322
#define AGX_ID_IMU_EULER	0x323
#define AGX_ID_BUMPER		0x331
#define AGX_ID_BMS_BASIC	0x361
#define AGX_ID_BMS_EXTENDED	0x362

#define AGX_ACTUATORS		8

/* Everything the decoder can currently fill in. Fields carry a _valid flag
 * rather than a sentinel value, because 0 is a legitimate speed and a
 * legitimate current. */
struct agx_state {
	bool system_valid;
	uint8_t vehicle_state;
	uint8_t control_mode;
	double battery_v;		/* 0.1 V steps */
	uint16_t error_code;

	bool motion_valid;
	double linear_mps;		/* mm/s on the wire */
	double angular_rps;
	double lateral_mps;
	double steering_rad;

	bool act_hs_valid[AGX_ACTUATORS];
	int16_t rpm[AGX_ACTUATORS];
	double motor_current_a[AGX_ACTUATORS];	/* 0.1 A steps */
	int32_t pulse_count[AGX_ACTUATORS];

	bool act_ls_valid[AGX_ACTUATORS];
	double driver_v[AGX_ACTUATORS];		/* 0.1 V steps */
	int16_t driver_temp_c[AGX_ACTUATORS];
	int8_t motor_temp_c[AGX_ACTUATORS];
	uint8_t driver_state[AGX_ACTUATORS];

	bool bms_valid;
	uint8_t soc, soh;
	double bms_v, bms_a, bms_temp_c;

	bool odom_valid;
	int32_t left_wheel, right_wheel;

	bool mode_valid;
	uint8_t motion_mode;
	uint8_t mode_changing;

	/*
	 * Which SCOUT MINI this is, inferred rather than configured.
	 *
	 * Skid and Omni share the same feedback frames - MotionStateFrame always
	 * carries a lateral field - and differ in that only the Omni can be
	 * commanded sideways. So a non-zero lateral velocity, or a motion-mode
	 * frame at all, means mecanum wheels. Reporting what was observed beats
	 * asking someone to tell the daemon what it is standing next to, and
	 * beats guessing.
	 */
	bool lateral_seen;
	bool mode_frame_seen;

	uint64_t decoded;		/* frames this decoder understood */
	uint64_t unknown;		/* frames it did not */
};

/* "omni", "skid (no lateral motion seen)", or "unknown" before enough frames. */
const char *agx_variant(const struct agx_state *s);

/* Returns true if the frame was recognised and applied. */
bool agx_decode(struct agx_state *s, uint32_t can_id, const uint8_t *data,
		uint8_t len);

/* A short human name for an id, or NULL. Used in logs and the status file so a
 * raw dump becomes readable without a lookup table to hand. */
const char *agx_id_name(uint32_t can_id);

/*
 * Build the payload of a MotionCommand (0x111).
 *
 * The frame mirrors MotionState (0x221): four big-endian int16 in mm/s, mrad/s,
 * mm/s, mrad. Values are given here in physical units and saturate at the int16
 * range rather than wrapping, because a wrapped velocity is full speed in the
 * opposite direction.
 *
 * Sign convention, REP-103 as the AgileX SDK's own ROS wrapper uses it:
 *
 *   linear  > 0  forward
 *   angular > 0  anticlockwise seen from above
 *   lateral > 0  to the LEFT
 *
 * The lateral sign is the one thing here that has NOT been checked against a
 * vehicle. It follows from the SDK mapping linear_y straight through and from
 * ROS taking +y as left, but nothing in this tree has watched a SCOUT MINI Omni
 * strafe. Confirm it at walking pace with the wheels clear before trusting it,
 * and if it is inverted set `lateral_invert` in the config rather than editing
 * this function - the convention documented here should keep describing the
 * protocol, not the last board someone tested.
 *
 * This is the only function in this file that builds a frame rather than reading
 * one. Nothing calls it unless command output has been explicitly enabled.
 */
void agx_encode_motion(uint8_t out[8], double linear_mps, double angular_rps,
		       double lateral_mps);

#endif /* AGILEX_H */
