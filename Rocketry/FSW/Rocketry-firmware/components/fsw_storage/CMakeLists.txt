idf_component_register(
    SRCS "sd_logger.c"
    INCLUDE_DIRS "include"
    REQUIRES fsw_core fatfs sdmmc esp_driver_spi esp_driver_sdspi
)