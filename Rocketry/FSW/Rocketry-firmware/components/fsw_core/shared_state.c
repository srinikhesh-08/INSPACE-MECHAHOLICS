#include "shared_state.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t s_lock;
static baro_data_t       s_baro;

void shared_init(void)
{
    s_lock = xSemaphoreCreateMutex();   /* created once, at startup */
}

void shared_write_baro(const baro_data_t *b)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_baro = *b;                        /* copy while locked */
    xSemaphoreGive(s_lock);
}

void shared_read_baro(baro_data_t *out)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_baro;
    xSemaphoreGive(s_lock);
}