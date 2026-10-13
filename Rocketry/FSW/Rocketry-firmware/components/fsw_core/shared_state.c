#include "shared_state.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t s_lock;
static baro_data_t       s_baro;
static imu_data_t        s_imu;
static gnss_data_t       s_gnss;
void shared_init(void)
{
    s_lock = xSemaphoreCreateMutex();   /* created once, at startup */
}

void shared_write_baro(const baro_data_t *b)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_baro = *b;
    xSemaphoreGive(s_lock);
}

void shared_write_imu(const imu_data_t *i)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_imu = *i;
    xSemaphoreGive(s_lock);
}

void shared_write_gnss(const gnss_data_t *g)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_gnss = *g;
    xSemaphoreGive(s_lock);
}

void shared_read_baro(baro_data_t *out)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_baro;
    xSemaphoreGive(s_lock);
}

void shared_read_imu(imu_data_t *out)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_imu;
    xSemaphoreGive(s_lock);
}

void shared_read_gnss(gnss_data_t *out)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_gnss;
    xSemaphoreGive(s_lock);
}