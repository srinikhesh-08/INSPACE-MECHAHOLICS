#include "esp_log.h"
#include "i2c_bus.h"
#include "sensors.h"

static const char *TAG = "sensors";
static i2c_master_bus_handle_t s_bus = nullptr;

esp_err_t i2c_bus_init(void)
{
    if (s_bus != nullptr) return ESP_OK;               /* already created */

    i2c_master_bus_config_t cfg = {};
    cfg.i2c_port          = I2C_NUM_0;
    cfg.sda_io_num        = (gpio_num_t)I2C_PIN_SDA;
    cfg.scl_io_num        = (gpio_num_t)I2C_PIN_SCL;
    cfg.clk_source        = I2C_CLK_SRC_DEFAULT;
    cfg.glitch_ignore_cnt = 7;
    cfg.flags.enable_internal_pullup = true;

    esp_err_t e = i2c_new_master_bus(&cfg, &s_bus);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(e));
        s_bus = nullptr;
    }
    return e;
}

i2c_master_bus_handle_t i2c_bus_handle(void)
{
    return s_bus;
}

extern "C" void sensors_start(void)
{
    i2c_bus_init();            /* bus first, so every sensor task finds it ready */
    baro_start();
    imu_start();
    gnss_start();
    power_start();
}