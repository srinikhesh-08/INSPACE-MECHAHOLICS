#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "config.h"
#include "sensors.h"
#include "i2c_bus.h"

static const char *tag = "power";

#define INA_ADDR           0x40       /* A0 and A1 to GND */
#define INA_REG_CONFIG     0x00
#define INA_REG_SHUNT      0x01
#define INA_REG_BUS        0x02
#define INA_CONFIG_VALUE   0x399F     /* 32 V bus range, +-320 mV shunt range, 12-bit, continuous (the reset default) */
#define INA_TIMEOUT_MS     20
#define BAT_SAMPLE_MS      100
#define BAT_AVG_SAMPLES    10         /* 10 samples x 100 ms = a 1 s average */
#define BAT_LOW_HOLD_MS    5000       /* below the limit this long => low-battery flag */
#define BAT_LOW_HYST_V     0.2f
#define BAT_V_GAIN         1.000f     /* adjust after comparing with a multimeter */

static i2c_master_dev_handle_t s_ina = nullptr;
static volatile float s_battery_v   = 0.0f;
static volatile bool  s_battery_ok  = false;
static volatile bool  s_battery_low = false;

static esp_err_t ina_read(uint8_t reg, int16_t *out)
{
    uint8_t buf[2];
    esp_err_t e = i2c_master_transmit_receive(s_ina, &reg, 1, buf, 2, INA_TIMEOUT_MS);
    if (e == ESP_OK) *out = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);    /* the chip sends the high byte first */
    return e;
}

static esp_err_t ina_write(uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return i2c_master_transmit(s_ina, buf, sizeof(buf), INA_TIMEOUT_MS);
}

/* Battery voltage = voltage at VIN- (bus register) + drop across the shunt: that is what sits at VIN+ */
static bool read_battery_v(float *v)
{
    int16_t shunt_raw, bus_raw;
    if (ina_read(INA_REG_SHUNT, &shunt_raw) != ESP_OK) return false;
    if (ina_read(INA_REG_BUS,   &bus_raw)   != ESP_OK) return false;

    float bus_v   = (float)((uint16_t)bus_raw >> 3) * 0.004f;    /* 4 mV per step */
    float shunt_v = (float)shunt_raw * 0.00001f;                 /* 10 uV per step */
    *v = (bus_v + shunt_v) * BAT_V_GAIN;
    return true;
}

static uint32_t now_ms(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static void power_task(void *arg)
{
    bool ok = false;
    i2c_master_bus_handle_t bus = i2c_bus_handle();
    if (bus != nullptr && i2c_master_probe(bus, INA_ADDR, 100) == ESP_OK) {
        i2c_device_config_t dc = {};
        dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dc.device_address  = INA_ADDR;
        dc.scl_speed_hz    = 100000;
        ok = (i2c_master_bus_add_device(bus, &dc, &s_ina) == ESP_OK) &&
             (ina_write(INA_REG_CONFIG, INA_CONFIG_VALUE) == ESP_OK);
    }
    if (!ok) {
        ESP_LOGE(tag, "INA219 not found at 0x%02X: no battery voltage", INA_ADDR);
    }

    float ring[BAT_AVG_SAMPLES];
    int count = 0, idx = 0, bad_reads = 0;
    bool below = false;
    uint32_t below_since_ms = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        if (ok) {
            float v;
            if (read_battery_v(&v) && v > 0.5f && v < 30.0f) {       /* plausible readings only */
                bad_reads = 0;
                ring[idx] = v;
                idx = (idx + 1) % BAT_AVG_SAMPLES;
                if (count < BAT_AVG_SAMPLES) count++;

                float sum = 0.0f;
                for (int k = 0; k < count; k++) sum += ring[k];
                float avg = sum / (float)count;
                s_battery_v  = avg;
                s_battery_ok = true;

                /* low-battery flag: below the limit for BAT_LOW_HOLD_MS, with hysteresis on the way back */
                float limit = VBAT_LOW_V + (s_battery_low ? BAT_LOW_HYST_V : 0.0f);
                uint32_t now = now_ms();
                if (avg < limit) {
                    if (!below) { below = true; below_since_ms = now; }
                    if (!s_battery_low && (now - below_since_ms) >= BAT_LOW_HOLD_MS) {
                        s_battery_low = true;
                        ESP_LOGW(tag, "LOW BATTERY: %.2f V", avg);
                    }
                } else {
                    below = false;
                    if (s_battery_low) {
                        s_battery_low = false;
                        ESP_LOGI(tag, "battery OK again: %.2f V", avg);
                    }
                }
            } else if (++bad_reads >= 20) {                           /* 2 s of failed reads */
                s_battery_ok = false;
            }
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(BAT_SAMPLE_MS));
    }
}

extern "C" void power_start(void)
{
    xTaskCreate(power_task, "power", 4096, NULL, 3, NULL);
}

extern "C" float power_battery_v(void)   { return s_battery_ok ? s_battery_v : 0.0f; }
extern "C" bool  power_battery_ok(void)  { return s_battery_ok; }
extern "C" bool  power_battery_low(void) { return s_battery_low; }