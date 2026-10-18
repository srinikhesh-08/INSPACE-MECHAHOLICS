idf_component_register(
    SRCS "logger_task.c" "fsw_core.c" "shared_state.c" "sim_sensors.c" "data_layer.c" "data_layer_task.c"
         "rate_control.c" "actuators.cpp" "debug_console.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos log esp_driver_ledc esp_driver_gpio fsw_storage
)