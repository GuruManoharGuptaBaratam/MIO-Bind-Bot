idf_component_register(
    SRCS "src/tft_display.cpp"
    INCLUDE_DIRS "include"
    REQUIRES st7735_gfx core_uart_receiver driver sd_config audio_feedback
)
