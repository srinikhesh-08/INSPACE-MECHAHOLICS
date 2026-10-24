#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "config.h"
#include "shared_state.h"
#include "fsw_core.h"
#include "sensors.h"
#include "sd_logger.h"
#include <string.h>

static const char *TAG = "health";

#define FAULT_BITS     6
#define FAULT_HOLD_S   3                 /* a fault is raised after this many bad seconds in a row */

static volatile uint32_t s_faults = 0;

static const uint32_t FAULT_MASK[FAULT_BITS] = {
    FAULT_IMU, FAULT_BARO, FAULT_GNSS, FAULT_SD, FAULT_RADIO, FAULT_LOWBAT
};
static const char *const FAULT_NAME[FAULT_BITS] = {
    "IMU", "BARO", "GNSS", "SD", "RADIO", "LOWBAT"
};

uint32_t health_faults(void) { return s_faults; }

/* watchdog: a task calls watch once at its start, then feed once per cycle */
void health_watch_this_task(void) { esp_task_wdt_add(NULL); }
void health_feed(void)            { esp_task_wdt_reset(); }

static void health_task(void *arg)
{
    int bad[FAULT_BITS] = { 0 };
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        baro_data_t b;  imu_data_t i;  gnss_data_t g;
        shared_read_baro(&b);
        shared_read_imu(&i);
        shared_read_gnss(&g);

        bool now_bad[FAULT_BITS] = {
            !i.valid, !b.valid, !g.valid, !sd_logger_ok(), !telemetry_radio_ok(), power_battery_low()
        };

        uint32_t faults = s_faults;
        for (int k = 0; k < FAULT_BITS; k++) {
            if (now_bad[k]) {
                if (bad[k] < FAULT_HOLD_S) bad[k]++;
                if (bad[k] >= FAULT_HOLD_S) faults |= FAULT_MASK[k];
            } else {
                bad[k] = 0;
                faults &= ~FAULT_MASK[k];
            }
        }

        if (faults != s_faults) {                      /* log only when something changes */
            char text[64] = "";
            for (int k = 0; k < FAULT_BITS; k++) {
                if (faults & FAULT_MASK[k]) {
                    strncat(text, FAULT_NAME[k], sizeof(text) - strlen(text) - 2);
                    strncat(text, " ", sizeof(text) - strlen(text) - 1);
                }
            }
            ESP_LOGW(TAG, "faults: %s", faults ? text : "none");
            s_faults = faults;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
    }
}

void health_start(void)
{
    xTaskCreate(health_task, "health", 3072, NULL, 2, NULL);
}