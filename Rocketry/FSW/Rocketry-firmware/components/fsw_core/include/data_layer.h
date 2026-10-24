#pragma once
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* "Condition must stay true for N ms" helper */
typedef struct {
    bool     active;
    uint32_t since_ms;
} hold_t;

/* All state of the data layer. Owned by one task, no locking needed. */
typedef struct {
    /* pad reference */
    bool     calibrated;
    float    cal_sum;
    uint32_t cal_count;
    float    p_ref;
    /* alpha-beta filter */
    bool     have_last;
    uint32_t last_t_ms;
    float    alt, vel, max_alt;
    /* detection */
    bool     launched, apogee, impact;
    hold_t   h_launch, h_apogee, h_still;
} dl_ctx_t;

void dl_init(dl_ctx_t *c);

/* Ask for a new pad reference. Ignored once launch is detected. */
void dl_recalibrate(dl_ctx_t *c);
/* After a mid-flight reset: skip calibration and rebuild the latches. */
void dl_restore(dl_ctx_t *c, float p_ref, float max_alt,
                bool launched, bool apogee, bool impact);
/* One processing step. Pure logic: no FreeRTOS, easy to test on a PC. */
void dl_step(dl_ctx_t *c, uint32_t t_ms, float pressure_pa, float accel_mag,
             abstract_state_t *out);

/* Starts the data layer task (reads shared baro and IMU, writes shared abstract) */
void data_layer_start(void);
/* Ask the data layer task for a new pad pressure reference (ignored after launch). */
void data_layer_request_recal(void);
#ifdef __cplusplus
}
#endif