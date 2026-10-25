#pragma once
#include "esp_err.h"
#include "driver/spi_common.h"

/* The one SPI bus shared by the SD card and the LoRa radio. PINS ARE ASSUMED: change to your wiring. */
#define FSW_SPI_HOST   SPI3_HOST      /* VSPI */
#define FSW_SPI_SCK    18
#define FSW_SPI_MISO   19
#define FSW_SPI_MOSI   23

#ifdef __cplusplus
extern "C" {
#endif
/* Starts the bus once. Safe to call from several places and tasks. */
esp_err_t fsw_spi_bus_init(void);
#ifdef __cplusplus
}
#endif