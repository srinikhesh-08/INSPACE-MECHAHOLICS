#pragma once
#include "esp_err.h"
#include <stdbool.h>

#define SD_PIN_MOSI   23
#define SD_PIN_MISO   19
#define SD_PIN_SCK    18
#define SD_PIN_CS     5
#define SD_SPI_HOST   SPI3_HOST

#define SD_MOUNT_POINT "/sdcard"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t sd_logger_mount(void);
esp_err_t sd_logger_open(bool resume, const char *header);
bool      sd_logger_write_line(const char *line);
bool      sd_logger_sync(void);
void      sd_logger_close(void);
bool      sd_logger_ok(void);

#ifdef __cplusplus
}
#endif