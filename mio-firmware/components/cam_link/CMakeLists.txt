idf_component_register(
    SRCS "src/cam_link.c"
    INCLUDE_DIRS "include"
    REQUIRES driver core_uart_receiver sd_config audio_feedback
    PRIV_REQUIRES esp_driver_uart job_dispatcher
)