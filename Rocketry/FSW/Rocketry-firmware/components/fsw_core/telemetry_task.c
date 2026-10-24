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
#include "sensors.h"

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
        (unsigned long)s_packet_count,
        a.alt_agl_m, b.pressure_pa, b.temp_c,
        (double)power_battery_v(),                     /* VOLTAGE_V, 0.01 V resolution */                                           /* VOLTAGE_V: battery reading comes at Stage 8 */
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
bool telemetry_radio_ok(void) { return s_radio_ok; }

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

/* ---- Telecommand packet from the ground, 6 bytes ----
   [0] 0xAA sync  [1] team LSB  [2] team MSB  [3] opcode  [4] argument  [5] XOR of bytes 0..4 */
#define TC_LEN      6
#define TC_SYNC     0xAAu

/* Last accepted toggle sequence per command (-1 = none yet). The toggle commands carry their
   sequence number in the argument byte; a repeat of the same value is not executed again. */
static int s_last_payload_seq   = -1;
static int s_last_parachute_seq = -1;

/* The reply is a short text line too. Format: ACK,<team>,<opcode hex>,<argument> */
static void send_ack(uint8_t op, uint8_t arg)
{
    char ack[32];
    int n = snprintf(ack, sizeof(ack), "ACK,%03u,%02X,%u\n",
                     (unsigned)TEAM_NUMBER, (unsigned)op, (unsigned)arg);
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
    uint16_t team = (uint16_t)(f[1] | (f[2] << 8));
    if (f[0] != TC_SYNC || team != (uint16_t)TEAM_NUMBER || x != f[TC_LEN - 1]) {
        return;                                   /* no sync, other team, or damaged */
    }

    uint8_t op = f[3], arg = f[4];

    /* Toggle commands carry a sequence number in the argument: a repeat must not toggle twice */
    int *last = NULL;
    if (op == CMD_TOGGLE_PAYLOAD)   last = &s_last_payload_seq;
    if (op == CMD_TOGGLE_PARACHUTE) last = &s_last_parachute_seq;

    if (last != NULL && (int)arg == *last) {
        ESP_LOGI(TAG, "duplicate toggle op=0x%02X seq=%u ignored", op, arg);
        if (s_tx_enabled) send_ack(op, arg);      /* answer again, in case the first ACK was lost */
        return;
    }

    bool ok = execute_command(op, arg);
    ESP_LOGI(TAG, "air command op=0x%02X arg=%u: %s", op, arg, ok ? "accepted" : "REJECTED");
    if (ok) {
        if (last != NULL) *last = arg;
        if (s_tx_enabled) send_ack(op, arg);      /* never transmit while TX is off */
    }
}

static void telemetry_task(void *arg)
{
    s_radio_ok = (lora_init() == ESP_OK);
    if (!s_radio_ok) {
        ESP_LOGE(TAG, "radio not found: no telemetry (SD logging continues)");
    }

    rate_register(RT_LOG);                            /* the FSM rate table drives the packet rate */
    health_watch_this_task();
    uint32_t period_ms = 1000u / TELEMETRY_RATE_HZ;
    TickType_t last_wake = xTaskGetTickCount();
    char line[TLM_LINE_MAX];
    uint32_t last_save_ms = 0;
    while (true) {
        health_feed();
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

        /* in flight, refresh the saved launch age every 30 s so a reset does not restart the 90 minutes */
        if (s_launch_ms != 0 && !s_tx_latched_off && (now - last_save_ms) >= 30000u) {
            last_save_ms = now;
            fsw_save_record();
        }

        if (s_radio_ok) radio_apply();

        /* the count advances only for packets built for transmission: it stays 0 until
           CMD_START_TELEMETRY and stays frozen while TX is off */
        bool tx_now = s_radio_ok && s_tx_enabled;
        if (tx_now) s_packet_count++;

        /* build the line once, give the same text to the SD logger and the radio */
        int len = build_line(line, sizeof(line));
        if (len > 0 && len < (int)sizeof(line)) {
            logger_post_line(line);
            if (tx_now) {
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
            health_feed();
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void telemetry_start(void)
{
    xTaskCreate(telemetry_task, "telemetry", 6144, NULL, 4, NULL);
}