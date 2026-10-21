#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* ---- Flight states ---- */
typedef enum {
    ST_BOOT = 0,
    ST_TEST_MODE,
    ST_LAUNCH_PAD,
    ST_ASCENT,
    ST_PAYLOAD_SEP,
    ST_DESCENT,
    ST_IMPACT,
    ST_COUNT                 /* number of states, must equal NUM_FLIGHT_STATES */
} fsw_state_t;

_Static_assert(ST_COUNT == NUM_FLIGHT_STATES, "state enum and config.h disagree");

/* ---- Telecommand opcodes ---- */
typedef enum {
    CMD_NONE            = 0x00,
    CMD_START_TELEMETRY = 0x01,
    CMD_STOP_TELEMETRY  = 0x02,
    CMD_CALIBRATE_BARO  = 0x03,
    CMD_CALIBRATE_GYRO  = 0x04,
    CMD_CALIBRATE_ACCEL = 0x05,
    CMD_ENTER_TEST      = 0x06,
    CMD_ENTER_LAUNCHPAD = 0x07,
    CMD_SWITCH_CHANNEL  = 0x08,
    CMD_TRIGGER_SEP     = 0x09,
    CMD_TOGGLE_PAYLOAD    = 0x0A,
    CMD_TOGGLE_PARACHUTE  = 0x0B
} cmd_opcode_t;

/* ---- Fault flags (bit mask) ---- */
enum {
    FAULT_IMU    = 1u << 0,
    FAULT_BARO   = 1u << 1,
    FAULT_GNSS   = 1u << 2,
    FAULT_SD     = 1u << 3,
    FAULT_RADIO  = 1u << 4,
    FAULT_LOWBAT = 1u << 5
};

/* ---- Sensor samples (filled by sensor tasks in later stages) ---- */
typedef struct {
    uint32_t t_ms;
    float ax, ay, az;        /* m/s^2, body frame, gravity included */
    float gx, gy, gz;        /* deg/s */
    bool  valid;
} imu_data_t;

typedef struct {
    uint32_t t_ms;
    float pressure_pa;
    float temp_c;
    bool  valid;
} baro_data_t;

typedef struct {
    uint32_t t_ms;
    uint32_t utc_hhmmss;
    float lat, lon, alt_msl;
    uint8_t sats;
    bool fix, valid;
} gnss_data_t;
/* ---- Data layer output: what the FSM reads ---- */
typedef struct {
    uint32_t t_ms;
    float alt_agl_m;          /* filtered altitude above the pad */
    float vel_mps;            /* filtered vertical velocity, + = up */
    float max_alt_m;          /* highest altitude since launch */
    float p_ref_pa;           /* pad pressure reference, 0 until calibrated */
    float accel_mag;          /* |a| in m/s^2 */
    bool  launch_detected;    /* latched */
    bool  apogee_detected;    /* latched */
    bool  impact_detected;    /* latched */
    bool  valid;              /* false until the pad reference is set */
} abstract_state_t;