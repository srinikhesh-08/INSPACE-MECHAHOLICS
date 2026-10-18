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

#define APOGEE_MIN_ALT_M         200.0f
#define APOGEE_NEG_VEL_MPS       1.0f       /* descending faster than this */
#define APOGEE_CONFIRM_MS        500u

/* ---- Cabinet servos (continuous rotation, timed moves) ---- */
#define PAYLOAD_SERVO_GPIO       18
#define PARACHUTE_SERVO_GPIO     4          /* set to the pin you actually wired */

#define SERVO_LOCK_SPEED_PCT     150.0f     /* forward: 0 -> 180 deg (your tested value) */
#define SERVO_UNLOCK_SPEED_PCT   (-150.0f)  /* reverse: 180 -> 0 deg */

/* How long each move runs. Calibrate on the bench until one move = 180 deg. */
#define PAYLOAD_LOCK_MS          1000u
#define PAYLOAD_UNLOCK_MS        1000u
#define PARACHUTE_LOCK_MS        1000u
#define PARACHUTE_UNLOCK_MS      1000u

#define IMPACT_ALT_M             10.0f
#define IMPACT_ACCEL_MPS2        29.4f      /* about 3 g deceleration spike */
#define IMPACT_STILL_VEL_MPS     0.5f
#define IMPACT_STILL_MS          3000u

/* ---- Health ---- */
#define VBAT_LOW_V               6.4f       /* adjust for your battery */

/* ---- Telemetry and logging ---- */
#define TELEMETRY_RATE_HZ        1u
#define HEALTH_RATE_HZ           1u

/* ---- Simulated sensors (TestAndSim) ---- */
#define USE_SIM_SENSORS          1          /* 1 = fake data, 0 = real drivers (later) */
#define SIM_PERIOD_MS            20u        /* sim runs at 50 Hz */
#define SIM_PAD_S                10.0f      /* time sitting on the pad */
#define SIM_LAUNCH_ON_COMMAND    1   /* 1 = sim waits on the pad for sim_request_launch(); 0 = auto launch after SIM_PAD_S */
#define SIM_BOOST_S              2.0f       /* motor burn time */
#define SIM_BOOST_ACCEL_MPS2     65.0f      /* net upward accel during burn, gives about 990 m apogee */
#define SIM_DESCENT_MPS          4.0f       /* steady parachute descent rate (spec: 2 to 5 m/s) */
#define SIM_SPIN_DPS             2.0f       /* small steady roll rate */
#define SIM_P0_PA                101325.0f  /* sea-level pressure */
#define SIM_PAD_ELEV_M           500.0f     /* fake GNSS altitude MSL of the pad */

/* ---- Data layer ---- */
#define DL_RATE_HZ               20u        /* ASCENT rate; Stage 5 will change it per state */
#define DL_CAL_SAMPLES           20u        /* pad pressure samples averaged for the zero reference */
#define DL_ALPHA                 0.30f      /* alpha-beta filter: altitude gain */
#define DL_BETA                  0.053f     /* velocity gain, about alpha^2 / (2 - alpha) */
#define ISA_ALT_SCALE_M          44330.0f   /* standard atmosphere constants */
#define ISA_EXPONENT             5.255f

/* ---- FSM ---- */
#define FSM_PERIOD_MS            100u       /* TaskFSM: 10 Hz in every state */
#define TEST_MODE_MIN_MS         2000u      /* minimum time in TEST_MODE */
#define SEP_DWELL_MS             2000u      /* time in PAYLOAD_SEP before DESCENT (servo move time + margin) */
#define USE_DEBUG_CONSOLE        1          /* keys typed in the serial monitor act as telecommands */

