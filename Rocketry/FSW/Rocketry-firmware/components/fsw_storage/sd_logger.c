#include "sd_logger.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>

static const char *TAG = "sd_log";
static sdmmc_card_t *s_card = NULL;
static FILE *s_file = NULL;

esp_err_t sd_logger_mount(void)
{
        /* 1. the shared SPI bus (started once for the SD card and the radio) */
    esp_err_t e = fsw_spi_bus_init();
    if (e != ESP_OK) return e;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = FSW_SPI_HOST;
    host.max_freq_khz = 10000;
    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = SD_PIN_CS;
    slot.host_id = FSW_SPI_HOST;

    /* 3. mount the FAT filesystem at /sdcard */
    esp_vfs_fat_sdmmc_mount_config_t mc = {
        .format_if_mount_failed = false,
        .max_files = 3,
        .allocation_unit_size = 16 * 1024,
    };
    e = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot, &mc, &s_card);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "mount failed: %s", esp_err_to_name(e));
        s_card = NULL;
        return e;
    }

    sdmmc_card_print_info(stdout, s_card);
    return ESP_OK;
}
static bool file_exists(const char *path)
{
    struct stat st;
    return stat(path, &st) == 0;
}

esp_err_t sd_logger_open(bool resume, const char *header)
{
    if (s_card == NULL) return ESP_ERR_INVALID_STATE;     /* not mounted */

    char path[40];
    int n = 1;
    while (n <= 9999) {                                   /* first unused number */
        snprintf(path, sizeof(path), SD_MOUNT_POINT "/flt_%04d.csv", n);
        if (!file_exists(path)) break;
        n++;
    }
    if (n > 9999) return ESP_FAIL;

    bool append = false;
    if (resume && n > 1) {                                /* reuse the newest file */
        n--;
        snprintf(path, sizeof(path), SD_MOUNT_POINT "/flt_%04d.csv", n);
        append = true;
    }

    s_file = fopen(path, "a");
    if (s_file == NULL) {
        ESP_LOGE(TAG, "cannot open %s", path);
        return ESP_FAIL;
    }
    
    if (!append && header) fputs(header, s_file);
    fflush(s_file);
    fsync(fileno(s_file));

    ESP_LOGI(TAG, "logging to %s (%s)", path, append ? "resumed" : "new");
    return ESP_OK;
}

void sd_logger_close(void)
{
    if (s_file) { fclose(s_file); s_file = NULL; }
    if (s_card) { esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card); s_card = NULL; }
}

bool sd_logger_write_line(const char *line)
{
    if (s_file == NULL) return false;
    if (fputs(line, s_file) < 0) {                        /* card pulled or full */
        ESP_LOGE(TAG, "write failed");
        sd_logger_close();
        return false;
    }
    return true;
}

bool sd_logger_sync(void)
{
    if (s_file == NULL) return false;
    if (fflush(s_file) != 0 || fsync(fileno(s_file)) != 0) {
        ESP_LOGE(TAG, "sync failed");
        sd_logger_close();
        return false;
    }
    return true;
}

bool sd_logger_ok(void)
{
    return s_file != NULL;
}