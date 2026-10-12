#pragma once

/* ---- Team ---- */
#define TEAM_NUMBER              7u   /* replace with team's number */

/* ---- Sensor task rates per flight state (Hz) ---- */
/* Order: BOOT, TEST_MODE, LAUNCH_PAD, ASCENT, PAYLOAD_SEP, DESCENT,  IMPACT */
#define NUM_FLIGHT_STATES        7

/* ---- Flight thresholds ---- */
#define LAUNCH_ACCEL_MPS2        25.0f      /* |a| above this for LAUNCH_SUSTAIN_MS */
#define LAUNCH_SUSTAIN_MS        200u
#define LAUNCH_ALT_M             3.0f       /* and altitude above this */

#define APOGEE_MIN_ALT_M         900.0f
#define APOGEE_NEG_VEL_MPS       1.0f       /* descending faster than this */
#define APOGEE_CONFIRM_MS        500u

/* ---- Payload lock servo (MG996R, separate supply) ---- */
#define SERVO_UNLOCKED_DEG       0.0f
#define SERVO_LOCKED_DEG         180.0f
#define SERVO_PWM_FREQ_HZ        50u
#define SERVO_PULSE_MIN_US       500u    /* typical MG996R; calibrate per servo */
#define SERVO_PULSE_MAX_US       2500u
#define SERVO_MOVE_TIME_MS       1000u   /* time allowed for a 180 deg move */

#define IMPACT_ALT_M             10.0f
#define IMPACT_ACCEL_MPS2        29.4f      /* about 3 g deceleration spike */
#define IMPACT_STILL_VEL_MPS     0.5f
#define IMPACT_STILL_MS          3000u

/* ---- Health ---- */
#define VBAT_LOW_V               6.4f       /* adjust for your battery */

/* ---- Telemetry and logging ---- */
#define TELEMETRY_RATE_HZ        1u
#define HEALTH_RATE_HZ           1u