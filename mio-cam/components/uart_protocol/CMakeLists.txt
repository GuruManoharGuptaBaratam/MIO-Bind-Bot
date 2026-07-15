idf_component_register(
    SRCS "src/uart_protocol.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES main esp_driver_uart driver esp_driver_gpio
)