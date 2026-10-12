#pragma once
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void shared_init(void);

/* Writers (sensor tasks) */
void shared_write_baro(const baro_data_t *b);

/* Readers (data layer, FSM, logger) */
void shared_read_baro(baro_data_t *out);

#ifdef __cplusplus
}
#endif