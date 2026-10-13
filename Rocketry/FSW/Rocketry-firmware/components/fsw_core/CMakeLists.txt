idf_component_register(
    SRCS "fsw_core.c" "shared_state.c" "sim_sensors.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos log
)