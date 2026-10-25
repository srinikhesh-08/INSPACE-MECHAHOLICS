idf_component_register(
    SRCS "telemetry_task.c" "gnss_neo6m.cpp" "baro_bmp585.cpp" "sensors.cpp" "imu_bno086.cpp"
        "logger_task.c" "fsw_core.c" "shared_state.c" "sim_sensors.c" "data_layer.c" "data_layer_task.c"
        "rate_control.c" "actuators.c" "debug_console.c" "power_ina219.cpp" "health.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos log esp_driver_ledc esp_driver_gpio esp_driver_i2c sh2 fsw_bus
    esp_driver_uart fsw_storage fsw_radio esp_timer
)