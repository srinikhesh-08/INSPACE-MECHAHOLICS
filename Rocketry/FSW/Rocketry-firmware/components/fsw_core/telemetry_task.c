#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "config.h"
#include "shared_state.h"
#include "rate_control.h"
#include "fsw_core.h"
#include "sx1276.h"

static const char *TAG = "tlm";

static uint32_t now_ms(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static uint32_t s_packet_count = 0;

/* One telemetry line, spec order and resolutions. The SD card and the radio get this exact text. */
static int build_line(char *buf, size_t n)
{
    baro_data_t      b;
    imu_data_t       i;
    gnss_data_t      g;
    abstract_state_t a;
    shared_read_baro(&b);
    shared_read_imu(&i);
    shared_read_gnss(&g);
    shared_read_abstract(&a);

    return snprintf(buf, n,
        "%s,%lu,%lu,%.1f,%.0f,%.1f,%.2f,"
        "%lu,%.4f,%.4f,%.1f,%u,"
        "%.2f,%.2f,%.2f,%.2f,%s\n",
        TEAM_ID_TEXT,
        (unsigned long)(now_ms() / 1000u),            /* TIME_S: seconds since power-on */
        (unsigned long)++s_packet_count,
        a.alt_agl_m, b.pressure_pa, b.temp_c,
        0.0,                                           /* VOLTAGE_V: battery reading comes at Stage 8 */
        (unsigned long)g.utc_hhmmss, g.lat, g.lon, g.alt_msl, (unsigned)g.sats,
        i.ax, i.ay, i.az, i.gz,                        /* gz assumed to be the roll axis */
        fsw_state_name(fsw_get_state()));
}

static volatile bool s_tx_enabled     = false;   /* OFF at power-up: no TX before the slot is called */
static volatile bool s_backup         = false;   /* false = MAIN channel, true = BACKUP */
static volatile bool s_retune         = true;    /* apply frequency and settings on the next pass */
static bool          s_tx_latched_off = false;   /* 90-minute limit reached */
static uint32_t      s_launch_ms      = 0;       /* time launch was detected, 0 = not yet */
static bool          s_radio_ok       = false;
static int           s_last_dbm       = -1;

void telemetry_set_tx(bool on)
{
    if (on && s_tx_latched_off) {
        ESP_LOGW(TAG, "TX stays OFF: the 90 minute limit was reached");
        return;
    }
    s_tx_enabled = on;
    ESP_LOGW(TAG, "telemetry TX %s", on ? "ON" : "OFF");
    fsw_save_record();
}

static void set_channel(bool backup)
{
    s_backup = backup;
    s_retune = true;
    ESP_LOGW(TAG, "LoRa channel: %s", backup ? "BACKUP (867.25 MHz)" : "MAIN (865.50 MHz)");
    fsw_save_record();
}

void telemetry_toggle_channel(void) { set_channel(!s_backup); }
bool telemetry_tx_enabled(void)     { return s_tx_enabled; }
bool telemetry_on_backup(void)      { return s_backup; }

uint8_t telemetry_get_flags(void)
{
    return (s_tx_enabled ? 1u : 0u) | (s_backup ? 2u : 0u) | (s_tx_latched_off ? 4u : 0u);
}

uint16_t telemetry_launch_age_s(void)
{
    return (s_launch_ms != 0) ? (uint16_t)((now_ms() - s_launch_ms) / 1000u) : 0;
}

void telemetry_restore(uint8_t flags, uint16_t launch_age_s)
{
    s_tx_enabled     = (flags & 1u) != 0;
    s_backup         = (flags & 2u) != 0;
    s_tx_latched_off = (flags & 4u) != 0;
    s_retune         = true;

    /* the saved age is up to 30 s old, so add 30 s: the 90-minute cutoff can only come early, never late */
    s_launch_ms = now_ms() - ((uint32_t)launch_age_s + 30u) * 1000u;
    if (s_launch_ms == 0) s_launch_ms = 1;            /* 0 means "not launched yet" */
    ESP_LOGW(TAG, "radio state restored: TX %s, %s channel, launch %u s ago",
             s_tx_enabled ? "ON" : "OFF", s_backup ? "BACKUP" : "MAIN", (unsigned)launch_age_s + 30u);
}

/* Frequency by channel, power by state. Radio goes back to listening afterwards. */
static void radio_apply(void)
{
    if (s_retune) {
        s_retune = false;
        lora_configure(s_backup ? LORA_BACKUP_HZ : LORA_MAIN_HZ, LORA_SF, (uint8_t)LORA_SYNC_WORD);
        s_last_dbm = -1;                              /* configure resets the power, set it again */
        lora_start_rx();
    }
    int dbm = (fsw_get_state() >= ST_ASCENT) ? LORA_PWR_FLIGHT_DBM : LORA_PWR_PAD_DBM;
    if (dbm != s_last_dbm) {
        lora_set_power_dbm(dbm);
        s_last_dbm = dbm;
    }
}

/* ---- Telecommand frame from the ground, 5 bytes ----
   [0] team number  [1] sequence  [2] opcode  [3] argument  [4] XOR of bytes 0..3 */
#define TC_LEN      5

static int s_last_seq = -1;            /* sequence of the last command executed (-1 = none yet) */

/* The reply is a short text line too, so the ground station only ever deals with text lines. */
static void send_ack(uint8_t seq, uint8_t op)
{
    char ack[32];
    int n = snprintf(ack, sizeof(ack), "ACK,%03u,%u,%02X\n", (unsigned)TEAM_NUMBER, seq, op);
    if (n > 0 && n < (int)sizeof(ack)) {
        lora_send((const uint8_t *)ack, (uint8_t)n);
        lora_start_rx();
    }
}

static bool execute_command(uint8_t op, uint8_t arg)
{
    switch (op) {
    case CMD_START_TELEMETRY:
        telemetry_set_tx(true);
        return true;
    case CMD_STOP_TELEMETRY:
        telemetry_set_tx(false);
        return true;
    case CMD_SWITCH_CHANNEL:
        if (arg > 1) return false;
        set_channel(arg == 1);                    /* 0 = main, 1 = backup: repeating it is harmless */
        return true;
    default:
        if (op >= CMD_CALIBRATE_BARO && op <= CMD_TOGGLE_PARACHUTE) {
            return fsw_post_command((cmd_opcode_t)op);   /* the FSM decides if it is allowed now */
        }
        return false;                             /* unknown opcode */
    }
}

static void poll_air_commands(void)
{
    uint8_t f[TC_LEN + 2];
    int n = lora_poll_rx(f, sizeof(f));
    if (n != TC_LEN) return;                      /* not a command frame */

    uint8_t x = 0;
    for (int k = 0; k < TC_LEN - 1; k++) x ^= f[k];
    if (f[0] != (uint8_t)TEAM_NUMBER || x != f[TC_LEN - 1]) {
        return;                                   /* other team, or damaged */
    }

    uint8_t seq = f[1], op = f[2], arg = f[3];

    if ((int)seq == s_last_seq) {                 /* a repeat of the last command: do not run it again */
        if (s_tx_enabled) send_ack(seq, op);      /* but answer again, in case the first ACK was lost */
        return;
    }

    bool ok = execute_command(op, arg);
    ESP_LOGI(TAG, "air command seq=%u op=0x%02X arg=%u: %s", seq, op, arg,
             ok ? "accepted" : "REJECTED");
    if (ok) {
        s_last_seq = seq;
        if (s_tx_enabled) send_ack(seq, op);      /* never transmit while TX is off */
    }
}

static void telemetry_task(void *arg)
{
    s_radio_ok = (lora_init() == ESP_OK);
    if (!s_radio_ok) {
        ESP_LOGE(TAG, "radio not found: no telemetry (SD logging continues)");
    }

    rate_register(RT_LOG);                            /* the FSM rate table drives the packet rate */
    uint32_t period_ms = 1000u / TELEMETRY_RATE_HZ;
    TickType_t last_wake = xTaskGetTickCount();
    char line[TLM_LINE_MAX];
    uint32_t last_save_ms = 0;
    while (true) {
        uint32_t now = now_ms();

        /* 90 minutes after launch detection the transmitter switches itself off for good */
        if (s_launch_ms == 0 && fsw_get_state() >= ST_ASCENT) {
            s_launch_ms = now;
        }
        if (s_launch_ms != 0 && !s_tx_latched_off &&
            (now - s_launch_ms) >= TX_AUTO_OFF_MIN * 60u * 1000u) {
            s_tx_latched_off = true;
            s_tx_enabled = false;
            ESP_LOGW(TAG, "90 minutes since launch: telemetry TX OFF");
            fsw_save_record();
        }

        if (s_radio_ok) radio_apply();

        /* build the line once, give the same text to the SD logger and the radio */
        int len = build_line(line, sizeof(line));
        if (len > 0 && len < (int)sizeof(line)) {
            logger_post_line(line);
            if (s_radio_ok && s_tx_enabled) {
                if (lora_send((const uint8_t *)line, (uint8_t)len) != ESP_OK) {
                    ESP_LOGW(TAG, "LoRa send failed");
                }
                lora_start_rx();                      /* back to listening */
            }
        }

        /* wait for the next packet time, listening for commands meanwhile */
        period_ms = rate_poll(period_ms);
        TickType_t next = last_wake + pdMS_TO_TICKS(period_ms);
        last_wake = next;
        while ((int32_t)(xTaskGetTickCount() - next) < 0) {
            if (s_radio_ok) {
                poll_air_commands();
                radio_apply();                        /* applies a channel change at once */
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void telemetry_start(void)
{
    xTaskCreate(telemetry_task, "telemetry", 6144, NULL, 4, NULL);
}