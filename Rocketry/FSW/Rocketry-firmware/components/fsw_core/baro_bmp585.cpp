#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "config.h"
#include "shared_state.h"
#include "rate_control.h"
#include "i2c_bus.h"

static const char* tag = "BMP585";

#define BMP585_REG_CHIP_ID      0x01
#define BMP585_REG_TEMP_XLSB    0x1D   /* 0x1D..0x1F temperature, 0x20..0x22 pressure */
#define BMP585_REG_STATUS       0x28
#define BMP585_REG_OSR_CONFIG   0x36
#define BMP585_REG_ODR_CONFIG   0x37
#define BMP585_REG_CMD          0x7E

#define BMP585_CHIP_ID          0x51
#define BMP585_STATUS_NVM_RDY   (1 << 1)
#define BMP585_STATUS_NVM_ERR   (1 << 2)
#define BMP585_CMD_SOFT_RESET   0xB6

#define BMP585_OSR_CONFIG_VALUE ((1 << 6) | (0x0 << 3) | 0x0)   /* pressure on, x1, x1 */

#define BMP585_ODR_50HZ         0x0F
#define BMP585_PWR_STANDBY      0x00
#define BMP585_PWR_NORMAL       0x01
#define BMP585_DEEP_DIS         (1 << 7)

#define BMP585_I2C_ADDR         0x47     /* 0x46 if the SDO pin is tied to GND */
#define BMP585_I2C_TIMEOUT_MS   20       /* a stuck bus returns an error, never hangs */


class temp_pressure_Sensor
{
private:
    int32_t raw_pressure = 0;
    int32_t raw_temp = 0;

    uint8_t data_address[1] = {BMP585_REG_TEMP_XLSB};
    uint8_t raw_data[6] = {};

    double final_T = 0.0;
    double final_P = 0.0;

    i2c_master_dev_handle_t device_handle = nullptr;

    esp_err_t write_register(uint8_t reg, uint8_t value)
    {
        uint8_t buf[2] = {reg, value};
        return i2c_master_transmit(device_handle, buf, sizeof(buf), BMP585_I2C_TIMEOUT_MS);
    }

    esp_err_t read_register(uint8_t reg, uint8_t* value)
    {
        return i2c_master_transmit_receive(device_handle, &reg, 1, value, 1, BMP585_I2C_TIMEOUT_MS);
    }

    static int32_t to_signed_24(uint32_t v)
    {
        int32_t s = (int32_t)(v & 0xFFFFFF);
        if (s & 0x800000) {
            s -= 0x1000000;
        }
        return s;
    }

public:
    bool initialise_sensor(i2c_master_bus_handle_t bus_handle)
    {
        i2c_device_config_t device_config = {};
        device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        device_config.device_address = BMP585_I2C_ADDR;
        device_config.scl_speed_hz = 100000;

        if (i2c_master_bus_add_device(bus_handle, &device_config, &device_handle) != ESP_OK) {
            return false;
        }

        write_register(BMP585_REG_CMD, BMP585_CMD_SOFT_RESET);
        vTaskDelay(pdMS_TO_TICKS(10));

        uint8_t chip_id = 0xFF;
        uint8_t status = 0xFF;
        read_register(BMP585_REG_CHIP_ID, &chip_id);
        read_register(BMP585_REG_STATUS, &status);

        if (chip_id != BMP585_CHIP_ID) {
            ESP_LOGE(tag, "Unexpected CHIP_ID 0x%02X (expected 0x%02X)", chip_id, BMP585_CHIP_ID);
            return false;
        }
        if (!(status & BMP585_STATUS_NVM_RDY) || (status & BMP585_STATUS_NVM_ERR)) {
            ESP_LOGE(tag, "NVM not ready or NVM error, STATUS = 0x%02X", status);
            return false;
        }

        esp_err_t e = write_register(BMP585_REG_ODR_CONFIG,
                                     BMP585_DEEP_DIS | (BMP585_ODR_50HZ << 2) | BMP585_PWR_STANDBY);
        vTaskDelay(pdMS_TO_TICKS(20));
        if (e == ESP_OK) e = write_register(BMP585_REG_OSR_CONFIG, BMP585_OSR_CONFIG_VALUE);
        if (e == ESP_OK) e = write_register(BMP585_REG_ODR_CONFIG,
                                            BMP585_DEEP_DIS | (BMP585_ODR_50HZ << 2) | BMP585_PWR_NORMAL);
        return e == ESP_OK;
    }

    esp_err_t update_temp_pressure()
    {
        esp_err_t err = i2c_master_transmit_receive(device_handle, data_address, sizeof(data_address),
                                                    raw_data, sizeof(raw_data), BMP585_I2C_TIMEOUT_MS);
        if (err != ESP_OK) {
            return err;
        }

        raw_temp     = to_signed_24((uint32_t)raw_data[2] << 16 | (uint32_t)raw_data[1] << 8 | raw_data[0]);
        raw_pressure = to_signed_24((uint32_t)raw_data[5] << 16 | (uint32_t)raw_data[4] << 8 | raw_data[3]);

        final_T = (double)raw_temp / 65536.0;
        final_P = (double)raw_pressure / 64.0;
        return ESP_OK;
    }

    double get_temp()     { return final_T; }
    double get_pressure() { return final_P; }
};

static void baro_task(void *arg)
{
    temp_pressure_Sensor baro;
    bool ok = (i2c_bus_handle() != nullptr) && baro.initialise_sensor(i2c_bus_handle());
    if (!ok) {
        ESP_LOGE(tag, "BMP585 init failed");
    }

    rate_register(RT_BARO);
    uint32_t period_ms = 200;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        baro_data_t b = {};
        b.t_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (ok && baro.update_temp_pressure() == ESP_OK) {
            float p = (float)baro.get_pressure();
            if (p > 30000.0f && p < 110000.0f) {       /* plausible readings only */
                b.pressure_pa = p;
                b.temp_c      = (float)baro.get_temp();
                b.valid       = true;
            }
        }
        shared_write_baro(&b);

        period_ms = rate_poll(period_ms);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
    }
}

extern "C" void baro_start(void)
{
    xTaskCreate(baro_task, "baro", 4096, NULL, 6, NULL);
}