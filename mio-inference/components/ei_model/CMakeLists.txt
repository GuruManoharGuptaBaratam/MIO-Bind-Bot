# 1. Gather ALL .cpp, .c, and .cc source files anywhere inside this component tree
file(GLOB_RECURSE COMPONENT_SRCS 
    "*.cpp"
    "*.c"
    "*.cc"
)

# 2. Safely filter out the standalone example file that causes printf format/ellipsis errors
list(FILTER COMPONENT_SRCS EXCLUDE REGEX "source/main.cpp")

# 3. Register the component with only valid base directories
idf_component_register(
    SRCS "${COMPONENT_SRCS}"
    INCLUDE_DIRS "."
                 "edge-impulse-sdk"
                 "edge-impulse-sdk/classifier"
                 "edge-impulse-sdk/dsp"
                 "edge-impulse-sdk/dsp/kissfft"
                 "edge-impulse-sdk/third_party/flatbuffers/include"
                 "edge-impulse-sdk/third_party/ruy"
                 "edge-impulse-sdk/third_party/gemmlowp"
                 "edge-impulse-sdk/third_party/arc_mli_package/include"
                    "edge-impulse-sdk/porting/espressif/esp-dsp/modules/common/include"
               "edge-impulse-sdk/porting/espressif/esp-dsp/modules/fft/include"
                 "model-parameters"
                 "tflite-model"
    REQUIRES esp_timer
)

# 4. Strip out global -Werror constraints AND turn off uninitialized/format checkers
component_compile_options(
    -Wno-error
    -Wno-attributes 
    -Wno-strict-aliasing 
    -Wno-maybe-uninitialized 
    -Wno-uninitialized
    -Wno-sign-compare
    -Wno-missing-field-initializers
    -Wno-deprecated-declarations
    -Wno-format
    -Wno-unused-function
    -Wno-unused-variable
    -Wno-unused-parameter
    -DEI_CLASSIFIER_TFLITE_ENABLE_ESP_NN=1
    -DEI_DSP_DISABLE_HARDWARE_FFT=1
)