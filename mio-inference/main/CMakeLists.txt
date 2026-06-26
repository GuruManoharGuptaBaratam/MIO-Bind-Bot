idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES inference_engine esp_driver_uart
)