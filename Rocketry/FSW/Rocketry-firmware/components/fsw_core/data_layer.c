#include <math.h>
#include <string.h>
#include "config.h"
#include "data_layer.h"

/* True once `cond` has been continuously true for need_ms. */
static bool hold(hold_t *h, bool cond, uint32_t now_ms, uint32_t need_ms)
{
    if (!cond) {
        h->active = false;
        return false;
    }
    if (!h->active) {
        h->active   = true;
        h->since_ms = now_ms;
    }
    return (now_ms - h->since_ms) >= need_ms;
}

static void publish(const dl_ctx_t *c, uint32_t t_ms, float accel_mag,
                    bool valid, abstract_state_t *o)
{
    o->t_ms            = t_ms;
    o->alt_agl_m       = c->alt;
    o->vel_mps         = c->vel;
    o->max_alt_m       = c->max_alt;
    o->p_ref_pa        = c->p_ref;
    o->accel_mag       = accel_mag;
    o->launch_detected = c->launched;
    o->apogee_detected = c->apogee;
    o->impact_detected = c->impact;
    o->valid           = valid;
}

void dl_init(dl_ctx_t *c)
{
    memset(c, 0, sizeof(*c));
}

void dl_recalibrate(dl_ctx_t *c)
{
    if (c->launched) {
        return;                 /* never move the zero reference during flight */
    }
    c->calibrated = false;
    c->cal_sum    = 0.0f;
    c->cal_count  = 0;
}

void dl_restore(dl_ctx_t *c, float p_ref, float max_alt,
                bool launched, bool apogee, bool impact)
{
    c->calibrated = true;          /* use the saved pad pressure, do not re-average */
    c->p_ref      = p_ref;
    c->have_last  = false;         /* filter restarts from the first new sample */
    c->alt = 0.0f;
    c->vel = 0.0f;
    c->max_alt  = max_alt;
    c->launched = launched;
    c->apogee   = apogee;
    c->impact   = impact;
}

void dl_step(dl_ctx_t *c, uint32_t t_ms, float pressure_pa, float accel_mag,
             abstract_state_t *out)
{
    /* 1. Pad calibration: average the first samples to get the zero-altitude pressure */
    if (!c->calibrated) {
        c->cal_sum += pressure_pa;
        c->cal_count++;
        if (c->cal_count >= DL_CAL_SAMPLES) {
            c->p_ref      = c->cal_sum / (float)c->cal_count;
            c->calibrated = true;
            c->have_last  = false;
            c->alt = 0.0f;
            c->vel = 0.0f;
        }
        publish(c, t_ms, accel_mag, false, out);
        return;
    }

    /* 2. Raw altitude above the pad from pressure */
    float alt_meas = ISA_ALT_SCALE_M * (1.0f - powf(pressure_pa / c->p_ref, 1.0f / ISA_EXPONENT));

    /* 3. Alpha-beta filter: predict, then correct with the measurement */
    if (!c->have_last) {
        c->alt       = alt_meas;
        c->vel       = 0.0f;
        c->have_last = true;
    } else {
        float dt = (float)(t_ms - c->last_t_ms) / 1000.0f;
        if (dt <= 0.0f || dt > 1.0f) {
            c->alt = alt_meas;                         /* bad gap: resync altitude only */
        } else {
            float predicted = c->alt + c->vel * dt;
            float residual  = alt_meas - predicted;
            c->alt = predicted + DL_ALPHA * residual;
            c->vel = c->vel + (DL_BETA / dt) * residual;
        }
    }
    c->last_t_ms = t_ms;

    /* 4. Launch: high |a| sustained, AND altitude above a small threshold */
    if (!c->launched) {
        bool accel_ok = hold(&c->h_launch, accel_mag > LAUNCH_ACCEL_MPS2, t_ms, LAUNCH_SUSTAIN_MS);
        if (accel_ok && c->alt > LAUNCH_ALT_M) {
            c->launched = true;
        }
    }

    /* 5. Peak altitude since launch */
    if (c->launched && c->alt > c->max_alt) {
        c->max_alt = c->alt;
    }

    /* 6. Apogee: still high, and descending faster than a threshold for a confirm time */
    if (c->launched && !c->apogee) {
        bool descending = (c->alt > APOGEE_MIN_ALT_M) && (c->vel < -APOGEE_NEG_VEL_MPS);
        if (hold(&c->h_apogee, descending, t_ms, APOGEE_CONFIRM_MS)) {
            c->apogee = true;
        }
    }

    /* 7. Impact: only after apogee, low altitude, and either a spike or standing still */
    if (c->apogee && !c->impact) {
        bool low   = c->alt < IMPACT_ALT_M;
        bool spike = low && (accel_mag > IMPACT_ACCEL_MPS2);
        bool still = hold(&c->h_still, low && fabsf(c->vel) < IMPACT_STILL_VEL_MPS,
                          t_ms, IMPACT_STILL_MS);
        if (spike || still) {
            c->impact = true;
        }
    }

    publish(c, t_ms, accel_mag, true, out);
}