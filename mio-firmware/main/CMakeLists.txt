idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash
    PRIV_REQUIRES hfp ble core_uart_receiver
)