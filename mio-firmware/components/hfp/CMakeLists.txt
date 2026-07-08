idf_component_register(
    SRCS "src/hfp_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES bt i2s_audio
    PRIV_REQUIRES driver esp_driver_gpio esp_ringbuf tft_display
)