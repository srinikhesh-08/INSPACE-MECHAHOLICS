#include <cstdio>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "shared_state.h"
#include "sim_sensors.h"

static void monitor_task(void *arg)
{
    baro_data_t b;
    imu_data_t  i;
    gnss_data_t g;
    while (true) {
        shared_read_baro(&b);
        shared_read_imu(&i);
        shared_read_gnss(&g);
        float alt = 44330.0f * (1.0f - powf(b.pressure_pa / SIM_P0_PA, 1.0f / 5.255f));
        printf("t=%6lu ms | p=%8.1f Pa  alt~%7.1f m | az=%6.2f  gz=%5.2f | gnss alt=%7.1f sats=%u\n",
               (unsigned long)b.t_ms, b.pressure_pa, alt, i.az, i.gz,
               g.alt_msl, (unsigned)g.sats);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main(void)
{
    shared_init();
#if USE_SIM_SENSORS
    sim_sensors_start();
#endif
    xTaskCreate(monitor_task, "monitor", 4096, nullptr, 4, nullptr);
}