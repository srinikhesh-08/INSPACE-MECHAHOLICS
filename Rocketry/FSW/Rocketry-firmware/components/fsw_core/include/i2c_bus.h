#pragma once
#include "esp_err.h"
#include "driver/i2c_master.h"

#define I2C_PIN_SDA   21
#define I2C_PIN_SCL   22

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t i2c_bus_init(void);
i2c_master_bus_handle_t i2c_bus_handle(void);
#ifdef __cplusplus
}
#endif