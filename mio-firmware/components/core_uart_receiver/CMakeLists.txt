idf_component_register(
    SRCS "src/core_uart_receiver.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos log bt hfp esp_driver_gpio
    PRIV_REQUIRES esp_driver_uart
)
