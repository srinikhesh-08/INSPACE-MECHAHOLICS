idf_component_register(
    SRCS "sx1276.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_spi esp_driver_gpio fsw_bus
)