idf_component_register(
    SRCS "fsw_core.c" "shared_state.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos
)