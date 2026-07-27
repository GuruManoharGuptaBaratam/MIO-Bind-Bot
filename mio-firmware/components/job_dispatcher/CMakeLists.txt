idf_component_register(
    SRCS "src/job_dispatcher.c"
    INCLUDE_DIRS "include"
    REQUIRES mpu6050 cam_link tft_display core_uart_receiver button_handler audio_feedback
)