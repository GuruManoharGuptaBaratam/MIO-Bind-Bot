idf_component_register(
    SRCS "src/inference_engine.cpp"
    INCLUDE_DIRS "include"
    REQUIRES ei_model esp_driver_uart freertos esp_driver_gpio esp_driver_i2s 
    PRIV_REQUIRES esp_ringbuf
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -O2

    # Edge Impulse
    # -DEI_DSP_DISABLE_HARDWARE_FFT=1
    # -DEI_CLASSIFIER_TFLITE_ENABLE_ESP_NN=1

    # Disable warnings promoted to errors
    -Wno-error
    -Wno-error=missing-field-initializers
    -Wno-error=maybe-uninitialized
    -Wno-error=uninitialized
    -Wno-error=format
    -Wno-error=deprecated-declarations
    -Wno-error=strict-aliasing
    -Wno-error=address

    -Wno-missing-field-initializers
    -Wno-maybe-uninitialized
    -Wno-uninitialized
    -Wno-format
    -Wno-deprecated-declarations
    -Wno-strict-aliasing
)