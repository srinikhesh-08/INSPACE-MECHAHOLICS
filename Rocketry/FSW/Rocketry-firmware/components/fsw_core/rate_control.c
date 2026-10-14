#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rate_control.h"

static TaskHandle_t s_handle[RT_COUNT];
static uint32_t     s_last_ms[RT_COUNT];   /* latest period requested, 0 = none yet */

void rate_register(rate_task_t id)
{
    s_handle[id] = xTaskGetCurrentTaskHandle();
    if (s_last_ms[id] != 0) {              /* the FSM asked before this task existed */
        xTaskNotify(s_handle[id], s_last_ms[id], eSetValueWithOverwrite);
    }
}

void rate_set(rate_task_t id, uint32_t period_ms)
{
    s_last_ms[id] = period_ms;
    if (s_handle[id] != NULL) {
        xTaskNotify(s_handle[id], period_ms, eSetValueWithOverwrite);
    }
}

uint32_t rate_poll(uint32_t current_ms)
{
    uint32_t value = 0;
    /* timeout 0 = do not wait, just check whether a notification is pending */
    if (xTaskNotifyWait(0, 0xFFFFFFFFu, &value, 0) == pdTRUE && value != 0) {
        return value;
    }
    return current_ms;
}