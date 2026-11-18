// SPDX-License-Identifier: MIT OR Apache-2.0
/*!
 * @file
 * @brief QuestPad VR-mode USB protocol: parse controller state reports and build commands.
 *
 * Self-contained (C99, no Monado headers), so it can be dropped into a Monado driver
 * (e.g. src/xrt/drivers/questpad/) or any other host. Protocol description: docs/vr.md.
 *
 * USB: VID 0x1209, PID 0x5151, bcdDevice 0x0300 in VR mode. One vendor interface per controller:
 * class 0xFF, subclass 0x51, protocol 0x01 (left) / 0x02 (right). Each has one interrupt IN
 * endpoint (64-byte state reports, ~500 Hz while the controller is connected, else every 100 ms)
 * and one interrupt OUT endpoint (commands).
 *
 * Frames (OpenXR conventions): world X right, Y up, Z backward (toward the user). Controller
 * frame for a controller held naturally: X right, Y up, -Z forward (pointing direction).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define QUESTPAD_VR_VID 0x1209
#define QUESTPAD_VR_PID 0x5151
#define QUESTPAD_VR_BCD_DEVICE 0x0300
#define QUESTPAD_VR_IFACE_CLASS 0xFF
#define QUESTPAD_VR_IFACE_SUBCLASS 0x51
#define QUESTPAD_VR_IFACE_PROTOCOL_LEFT 0x01
#define QUESTPAD_VR_IFACE_PROTOCOL_RIGHT 0x02

#define QUESTPAD_VR_REPORT_LEN 64
#define QUESTPAD_VR_REPORT_STATE 0x01
#define QUESTPAD_VR_PROTOCOL_VERSION 1

// Report flags (byte 2).
#define QUESTPAD_VR_FLAG_CONNECTED 0x01
#define QUESTPAD_VR_FLAG_RIGHT 0x02
#define QUESTPAD_VR_FLAG_ORIENTATION 0x04 //!< orientation initialised from gravity
#define QUESTPAD_VR_FLAG_GYRO_CAL 0x08    //!< gyro bias measured at least once (controller set down still)
#define QUESTPAD_VR_FLAG_STANDBY 0x10
#define QUESTPAD_VR_FLAG_DEAD_BATTERY 0x20
#define QUESTPAD_VR_FLAG_IMU_FRESH 0x40 //!< this report carries a new IMU sample
#define QUESTPAD_VR_FLAG_IMU_SCALE 0x80 //!< IMU units read from the controller (register 0x32)

// Buttons (byte 60).
#define QUESTPAD_VR_BTN_A_X 0x01      //!< A (right) / X (left)
#define QUESTPAD_VR_BTN_B_Y 0x02      //!< B (right) / Y (left)
#define QUESTPAD_VR_BTN_THUMBSTICK 0x04
#define QUESTPAD_VR_BTN_SYSTEM_MENU 0x08 //!< Oculus (right) / Menu (left)

// Capacitive touch (byte 61): low nibble = touching, high nibble = finger near.
#define QUESTPAD_VR_TOUCH_A_X 0x01
#define QUESTPAD_VR_TOUCH_B_Y 0x02
#define QUESTPAD_VR_TOUCH_THUMBSTICK 0x04
#define QUESTPAD_VR_TOUCH_TRIGGER 0x08
#define QUESTPAD_VR_NEAR_A_X 0x10
#define QUESTPAD_VR_NEAR_B_Y 0x20
#define QUESTPAD_VR_NEAR_THUMBSTICK 0x40
#define QUESTPAD_VR_NEAR_TRIGGER 0x80

// Commands (OUT endpoint).
#define QUESTPAD_VR_CMD_HAPTIC 0x10   //!< [0x10][amplitude u8][duration_ms u16 LE]
#define QUESTPAD_VR_CMD_RECENTER 0x11 //!< [0x11]: current heading becomes yaw 0
#define QUESTPAD_VR_CMD_RESET 0x12    //!< [0x12]: restart orientation from gravity

#define QUESTPAD_VR_STICK_FULL 32000.0f
#define QUESTPAD_VR_ANALOG_MAX 4095.0f

struct questpad_vr_state
{
	uint8_t flags;
	uint8_t seq;            //!< increments on every new report; unchanged = keep-alive resend
	uint32_t imu_ts_us;     //!< controller clock, wraps
	uint32_t dongle_ts_us;  //!< dongle clock when the report was built, wraps
	float orientation[4];   //!< x, y, z, w: controller -> world
	float angular_velocity[3]; //!< rad/s, controller frame
	float acceleration[3];  //!< specific force, m/s², controller frame (+9.81 on Y when level and still)
	float thumbstick[2];    //!< x right, y up, -1 .. 1
	float trigger;          //!< 0 .. 1
	float squeeze;          //!< grip, 0 .. 1
	uint8_t buttons;        //!< QUESTPAD_VR_BTN_*
	uint8_t touch;          //!< QUESTPAD_VR_TOUCH_* | QUESTPAD_VR_NEAR_*
	int battery_percent;    //!< -1 = unknown
};

static inline uint16_t
questpad_vr_u16(const uint8_t *p)
{
	return (uint16_t)(p[0] | (p[1] << 8));
}

static inline uint32_t
questpad_vr_u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline float
questpad_vr_f32(const uint8_t *p)
{
	uint32_t u = questpad_vr_u32(p);
	float f;
	memcpy(&f, &u, sizeof(f));
	return f;
}

static inline float
questpad_vr_clampf(float v, float lo, float hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

/*!
 * Parse one IN report. Returns false if it isn't a state report this header understands.
 */
static inline bool
questpad_vr_parse(const uint8_t *buf, int len, struct questpad_vr_state *out)
{
	if (len < QUESTPAD_VR_REPORT_LEN || buf[0] != QUESTPAD_VR_REPORT_STATE ||
	    buf[1] != QUESTPAD_VR_PROTOCOL_VERSION) {
		return false;
	}
	out->flags = buf[2];
	out->seq = buf[3];
	out->imu_ts_us = questpad_vr_u32(buf + 4);
	out->dongle_ts_us = questpad_vr_u32(buf + 8);
	for (int i = 0; i < 4; i++) {
		out->orientation[i] = questpad_vr_f32(buf + 12 + 4 * i);
	}
	for (int i = 0; i < 3; i++) {
		out->angular_velocity[i] = questpad_vr_f32(buf + 28 + 4 * i);
		out->acceleration[i] = questpad_vr_f32(buf + 40 + 4 * i);
	}
	out->thumbstick[0] = questpad_vr_clampf((int16_t)questpad_vr_u16(buf + 52) / QUESTPAD_VR_STICK_FULL, -1, 1);
	out->thumbstick[1] = questpad_vr_clampf((int16_t)questpad_vr_u16(buf + 54) / QUESTPAD_VR_STICK_FULL, -1, 1);
	out->trigger = questpad_vr_clampf(questpad_vr_u16(buf + 56) / QUESTPAD_VR_ANALOG_MAX, 0, 1);
	out->squeeze = questpad_vr_clampf(questpad_vr_u16(buf + 58) / QUESTPAD_VR_ANALOG_MAX, 0, 1);
	out->buttons = buf[60];
	out->touch = buf[61];
	out->battery_percent = buf[62] == 255 ? -1 : buf[62];
	return true;
}

/*!
 * Haptic command: amplitude 0..1, duration in ms (0 = until the next haptic command).
 * Monado's XRT_MIN_HAPTIC_DURATION (-1) maps to a short pulse; pass e.g. 20 ms for it.
 * Returns the command length (4).
 */
static inline int
questpad_vr_build_haptic(uint8_t out[4], float amplitude, uint16_t duration_ms)
{
	float a = questpad_vr_clampf(amplitude, 0, 1);
	out[0] = QUESTPAD_VR_CMD_HAPTIC;
	out[1] = (uint8_t)(a * 255.0f + 0.5f);
	out[2] = (uint8_t)(duration_ms & 0xFF);
	out[3] = (uint8_t)(duration_ms >> 8);
	return 4;
}

static inline int
questpad_vr_build_recenter(uint8_t out[1])
{
	out[0] = QUESTPAD_VR_CMD_RECENTER;
	return 1;
}

static inline int
questpad_vr_build_reset(uint8_t out[1])
{
	out[0] = QUESTPAD_VR_CMD_RESET;
	return 1;
}

#ifdef __cplusplus
}
#endif
