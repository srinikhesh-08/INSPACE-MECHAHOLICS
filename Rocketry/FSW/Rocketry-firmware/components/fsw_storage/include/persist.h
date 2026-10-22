#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    uint32_t magic;             /* set by persist_save, checked on load */
    uint8_t  state;             /* fsw_state_t as a byte */
    uint8_t  payload_locked;
    uint8_t  parachute_locked;
    uint8_t  radio_flags;       /* bit0 TX on, bit1 backup channel, bit2 90-minute limit reached */
    float    p_ref_pa;          /* pad pressure reference */
    float    max_alt_m;         /* peak altitude so far */
    uint16_t launch_age_s;      /* seconds since launch detection, refreshed every 30 s in flight */
} persist_record_t;

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t persist_init(void);
esp_err_t persist_repair(void);     /* ground only, never call in flight */
bool      persist_ready(void);
esp_err_t persist_save(const persist_record_t *rec);
bool      persist_load(persist_record_t *out);    /* false if nothing valid */
esp_err_t persist_clear(void);
#ifdef __cplusplus
}
#endif