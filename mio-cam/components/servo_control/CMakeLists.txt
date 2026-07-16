idf_component_register(
    SRCS "src/servo_control.c"
    INCLUDE_DIRS "include"
    REQUIRES driver
    PRIV_REQUIRES esp_driver_ledc
)
 