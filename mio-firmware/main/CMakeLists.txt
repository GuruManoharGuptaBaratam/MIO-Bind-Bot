idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash
    PRIV_REQUIRES hfp ble core_uart_receiver tft_display sd_config mpu6050 job_dispatcher cam_link audio_feedback button_handler
)