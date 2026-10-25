#include <stdbool.h>
#include "spi_bus.h"
#include "esp_log.h"

static const char *TAG = "spi_bus";
static bool s_ready = false;

esp_err_t fsw_spi_bus_init(void)
{
    if (s_ready) return ESP_OK;

    spi_bus_config_t bus = {
        .mosi_io_num     = FSW_SPI_MOSI,
        .miso_io_num     = FSW_SPI_MISO,
        .sclk_io_num     = FSW_SPI_SCK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096,                  /* the SD card moves 512-byte blocks */
    };
    esp_err_t e = spi_bus_initialize(FSW_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (e == ESP_ERR_INVALID_STATE) e = ESP_OK;   /* another task started it a moment ago */
    if (e == ESP_OK) {
        s_ready = true;
    } else {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(e));
    }
    return e;
}