#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include "spi_bus.h"


#define SD_PIN_CS     13

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