idf_component_register(
    SRCS "src/button_handler.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer freertos audio_feedback tft_display core_uart_receiver job_dispatcher
)