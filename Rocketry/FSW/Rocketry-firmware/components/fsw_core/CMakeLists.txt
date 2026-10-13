idf_component_register(
    SRCS "fsw_core.c" "shared_state.c" "sim_sensors.c" "data_layer.c" "data_layer_task.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos log
)