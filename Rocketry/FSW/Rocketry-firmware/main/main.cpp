#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "shared_state.h"
#include "sim_sensors.h"
#include "data_layer.h"
#include "fsw_core.h"
#include "debug_console.h"

static void monitor_task(void *arg)
{
    abstract_state_t a;
    while (true) {
        shared_read_abstract(&a);
                printf("t=%6lu ms | %-11s | alt=%7.1f m vel=%6.1f m/s | payload_locked=%d chute_locked=%d | L=%d A=%d I=%d\n",
               (unsigned long)a.t_ms, fsw_state_name(fsw_get_state()),
               a.alt_agl_m, a.vel_mps, fsw_payload_locked(), fsw_parachute_locked(),
               a.launch_detected, a.apogee_detected, a.impact_detected);
    }
}

extern "C" void app_main(void)
{
    shared_init();
    fsw_core_start();
#if USE_SIM_SENSORS
    sim_sensors_start();
#endif
    data_layer_start();
#if USE_DEBUG_CONSOLE
    debug_console_start();
#endif
    xTaskCreate(monitor_task, "monitor", 4096, nullptr, 4, nullptr);
}