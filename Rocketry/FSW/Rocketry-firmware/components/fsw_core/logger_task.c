#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "shared_state.h"
#include "rate_control.h"
#include "fsw_core.h"
#include "sd_logger.h"
#include <math.h>
#include <string.h>
#include <string.h>
#include "freertos/queue.h"

#define LOG_QUEUE_LEN   20      /* lines kept while the card is busy or missing */

static QueueHandle_t s_line_q = NULL;

#define LOG_SYNC_MS  1u    /* force data onto the card once per second */
#define LOG_RETRY_MS   2000u    /* retry the card this often if it is missing */

static const char CSV_HEADER[] =
    "TEAM_ID,TIME_S,PACKET_COUNT,ALTITUDE_M,PRESSURE_PA,TEMP_C,VOLTAGE_V,"
    "GNSS_TIME,GNSS_LAT,GNSS_LON,GNSS_ALT_M,GNSS_SATS,"
    "ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_SPIN_DPS,FLIGHT_STATE\n";

static bool s_opened_before = false;

static void try_start_card(void)
{
    if (sd_logger_mount() == ESP_OK) {
        bool resume = fsw_restored_info(NULL, NULL) || s_opened_before;
        if (sd_logger_open(resume, CSV_HEADER) == ESP_OK) {
            s_opened_before = true;
        } else {
            sd_logger_close();
        }
    }
}

bool logger_post_line(const char *line)
{
    if (s_line_q == NULL) return false;
    char item[TLM_LINE_MAX];
    snprintf(item, sizeof(item), "%s", line);
    return xQueueSend(s_line_q, item, 0) == pdTRUE;    /* never blocks; a full queue drops the newest */
}

static void logger_task(void *arg)
{
    char line[TLM_LINE_MAX];
    uint32_t last_try_ms = 0;

    try_start_card();

    while (true) {
        if (!sd_logger_ok()) {
            /* no card: lines wait in the queue, retry the mount now and then */
            uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now_ms - last_try_ms >= LOG_RETRY_MS) {
                last_try_ms = now_ms;
                try_start_card();
            }
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (xQueuePeek(s_line_q, line, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (sd_logger_write_line(line)) {
                xQueueReceive(s_line_q, line, 0);      /* written: now remove it */
                sd_logger_sync();                      /* one line per second, so sync every time */
            }
            /* on a write error the card was closed and the line stays queued for the retry */
        }
    }
}

void logger_start(void)
{
    s_line_q = xQueueCreate(LOG_QUEUE_LEN, TLM_LINE_MAX);
    xTaskCreate(logger_task, "logger", 6144, NULL, 3, NULL);
}