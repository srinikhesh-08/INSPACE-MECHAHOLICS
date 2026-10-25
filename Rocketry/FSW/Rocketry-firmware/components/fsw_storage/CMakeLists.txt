idf_component_register(
    SRCS "sd_logger.c" "persist.c"
    INCLUDE_DIRS "include"
    REQUIRES fatfs sdmmc esp_driver_spi esp_driver_sdspi nvs_flash fsw_bus
)