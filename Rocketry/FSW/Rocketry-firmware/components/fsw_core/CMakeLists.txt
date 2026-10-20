idf_component_register(
    SRCS "gnss_neo6m.cpp" "baro_bmp585.cpp" "logger_task.c" "fsw_core.c" "shared_state.c" "sim_sensors.c" "data_layer.c" "data_layer_task.c"
         "rate_control.c" "actuators.c" "debug_console.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos log esp_driver_ledc esp_driver_gpio esp_driver_i2c esp_driver_uart fsw_storage
)