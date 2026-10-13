#pragma once
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void shared_init(void);

/* Writers (sensor tasks) */
void shared_write_baro(const baro_data_t *b);
void shared_write_imu(const imu_data_t *i);
void shared_write_gnss(const gnss_data_t *g);
/* Readers (data layer, FSM, logger) */
void shared_read_baro(baro_data_t *out);
void shared_read_imu(imu_data_t *out);
void shared_read_gnss(gnss_data_t *out);

void shared_write_abstract(const abstract_state_t *a);
void shared_read_abstract(abstract_state_t *out);
#ifdef __cplusplus
}
#endif