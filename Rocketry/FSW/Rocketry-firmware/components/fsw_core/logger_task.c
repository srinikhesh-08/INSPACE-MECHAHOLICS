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

#define LOG_SYNC_MS  1u    /* force data onto the card once per second */
#define LOG_RETRY_MS   2000u    /* retry the card this often if it is missing */

static const char CSV_HEADER[] =
    "TEAM_ID,TIME_S,PACKET_COUNT,ALTITUDE_M,PRESSURE_PA,TEMP_C,VOLTAGE_V,"
    "GNSS_TIME,GNSS_LAT,GNSS_LON,GNSS_ALT_M,GNSS_SATS,"
    "ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_SPIN_DPS,FLIGHT_STATE\n";

static float round_to(float v, float step)
{
    return roundf(v / step) * step;
}

static uint16_t s_packet_count = 0;

static void fill_packet(telemetry_packet_t *p)
{
    baro_data_t      b;
    imu_data_t       i;
    gnss_data_t      g;
    abstract_state_t a;
    shared_read_baro(&b);
    shared_read_imu(&i);
    shared_read_gnss(&g);
    shared_read_abstract(&a);

    memset(p, 0, sizeof(*p));
    p->team_number    = TEAM_NUMBER;
    p->mission_time   = (xTaskGetTickCount() * portTICK_PERIOD_MS) / 1000u;
    p->packet_count   = ++s_packet_count;
    p->altitude       = round_to(a.alt_agl_m, 0.1f);
    p->pressure       = round_to(b.pressure_pa, 1.0f);
    p->temperature    = round_to(b.temp_c, 0.1f);
    p->voltage        = 0.0f;                       /* Stage 8: battery ADC */
    p->gnss_time      = g.utc_hhmmss;
    p->latitude       = round_to(g.lat, 0.0001f);
    p->longitude      = round_to(g.lon, 0.0001f);
    p->gnss_altitude  = round_to(g.alt_msl, 0.1f);
    p->gnss_sats      = g.sats;
    p->accel_x        = i.ax;
    p->accel_y        = i.ay;
    p->accel_z        = i.az;
    p->gyro_spin_rate = i.gz;                       /* assumed roll axis */
    p->flight_state   = (uint8_t)fsw_get_state();
}
static int packet_to_csv(const telemetry_packet_t *p, char *buf, size_t n)
{
    return snprintf(buf, n,
        "2026 IN-SPACe-%03u,%lu,%u,%.1f,%.0f,%.1f,%.2f,"
        "%lu,%.4f,%.4f,%.1f,%u,%.2f,%.2f,%.2f,%.2f,%s\n",
        (unsigned)p->team_number, (unsigned long)p->mission_time,
        (unsigned)p->packet_count,
        p->altitude, p->pressure, p->temperature, p->voltage,
        (unsigned long)p->gnss_time, p->latitude, p->longitude, p->gnss_altitude,
        (unsigned)p->gnss_sats, p->accel_x, p->accel_y, p->accel_z,
        p->gyro_spin_rate, fsw_state_name((fsw_state_t)p->flight_state));
}
static void try_start_card(void)
{
    if (sd_logger_mount() == ESP_OK) {
        if (sd_logger_open(false, CSV_HEADER) != ESP_OK) {
            sd_logger_close();      /* unmount so the next retry starts clean */
        }
    }
}

static void logger_task(void *arg)
{
    rate_register(RT_LOG);
    uint32_t period_ms = 1000u / TELEMETRY_RATE_HZ;
    char row[256];

    try_start_card();
    uint32_t last_try_ms  = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t last_sync_ms = last_try_ms;
    TickType_t last_wake  = xTaskGetTickCount();

    while (true) {
        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        telemetry_packet_t pkt;
        fill_packet(&pkt);

        if (!sd_logger_ok()) {
            if (now_ms - last_try_ms >= LOG_RETRY_MS) {
                last_try_ms = now_ms;
                try_start_card();
            }
        } else {
            int len = packet_to_csv(&pkt, row, sizeof(row));
            if (len > 0 && len < (int)sizeof(row)) {
                sd_logger_write_line(row);
            }
            if (now_ms - last_sync_ms >= LOG_SYNC_MS) {
                last_sync_ms = now_ms;
                sd_logger_sync();
            }
        }

        period_ms = rate_poll(period_ms);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
    }
}

void logger_start(void)
{
    xTaskCreate(logger_task, "logger", 6144, NULL, 3, NULL);
}