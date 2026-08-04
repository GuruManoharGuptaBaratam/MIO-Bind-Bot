idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp32-camera wifi_client servo_control uart_protocol camera_driver sd_storage
                       PRIV_REQUIRES esp_timer mbedtls
                       )