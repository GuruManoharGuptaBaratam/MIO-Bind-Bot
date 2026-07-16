idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES uart_protocol servo_control
    PRIV_REQUIRES esp_driver_gpio  esp_driver_ledc
                    
)
