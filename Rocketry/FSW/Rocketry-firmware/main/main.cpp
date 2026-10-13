#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "shared_state.h"
#include "sim_sensors.h"
#include "data_layer.h"

static void monitor_task(void *arg)
{
    abstract_state_t a;
    while (true) {
        shared_read_abstract(&a);
        printf("t=%6lu ms | alt=%7.1f m  vel=%6.1f m/s  max=%7.1f | |a|=%5.1f | launch=%d apogee=%d impact=%d valid=%d\n",
               (unsigned long)a.t_ms, a.alt_agl_m, a.vel_mps, a.max_alt_m, a.accel_mag,
               a.launch_detected, a.apogee_detected, a.impact_detected, a.valid);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main(void)
{
    shared_init();
#if USE_SIM_SENSORS
    sim_sensors_start();
#endif
    data_layer_start();
    xTaskCreate(monitor_task, "monitor", 4096, nullptr, 4, nullptr);
}