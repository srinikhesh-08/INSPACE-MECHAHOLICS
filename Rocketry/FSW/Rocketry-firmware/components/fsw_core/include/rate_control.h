#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RT_IMU = 0,
    RT_BARO,
    RT_GNSS,
    RT_DATA,
    RT_LOG,
    RT_COUNT
} rate_task_t;

/* Task side: call once at the start of a rate-controlled task, from that task. */
void rate_register(rate_task_t id);

/* Task side: call once per cycle. Returns the new period if the FSM sent one, else current_ms. */
uint32_t rate_poll(uint32_t current_ms);

/* FSM side: ask a task to run with a new period. */
void rate_set(rate_task_t id, uint32_t period_ms);

#ifdef __cplusplus
}
#endif