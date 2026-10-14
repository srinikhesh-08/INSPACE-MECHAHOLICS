#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "config.h"
#include "shared_state.h"
#include "data_layer.h"
#include "rate_control.h"

static const char *TAG = "data";

static void data_layer_task(void *arg)
{
    dl_ctx_t ctx;
    dl_init(&ctx);
    rate_register(RT_DATA);
    uint32_t period_ms = 1000u / DL_RATE_HZ;
    bool was_valid = false, was_launch = false, was_apogee = false, was_impact = false;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        baro_data_t b;
        imu_data_t  i;
        shared_read_baro(&b);
        shared_read_imu(&i);

        if (b.valid && i.valid) {
            uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            float a_mag = sqrtf(i.ax * i.ax + i.ay * i.ay + i.az * i.az);

            abstract_state_t out;
            dl_step(&ctx, now_ms, b.pressure_pa, a_mag, &out);
            shared_write_abstract(&out);

            /* log each event once, on the tick it first happens */
            if (out.valid && !was_valid) {
                ESP_LOGI(TAG, "baro calibrated, p_ref=%.1f Pa", ctx.p_ref);
            }
            if (out.launch_detected && !was_launch) {
                ESP_LOGI(TAG, "LAUNCH detected   alt=%.1f m vel=%.1f m/s", out.alt_agl_m, out.vel_mps);
            }
            if (out.apogee_detected && !was_apogee) {
                ESP_LOGI(TAG, "APOGEE detected   alt=%.1f m max=%.1f m vel=%.1f m/s",
                         out.alt_agl_m, out.max_alt_m, out.vel_mps);
            }
            if (out.impact_detected && !was_impact) {
                ESP_LOGI(TAG, "IMPACT detected   alt=%.1f m vel=%.1f m/s", out.alt_agl_m, out.vel_mps);
            }
            was_valid  = out.valid;
            was_launch = out.launch_detected;
            was_apogee = out.apogee_detected;
            was_impact = out.impact_detected;
        }
        period_ms = rate_poll(period_ms);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
    }
}

void data_layer_start(void)
{
    xTaskCreate(data_layer_task, "data", 4096, NULL, 5, NULL);
}