cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mio-inference)

# Apply global build overrides across the active build ecosystem tree
add_compile_options(
    "-Wno-error=missing-field-initializers"
    "-Wno-error=format"
    "-Wno-error=deprecated-declarations"
    "-Wno-error=address"
    "-Wno-error=uninitialized"
    "-Wno-error=maybe-uninitialized"
    "-Wno-error=strict-aliasing"
    "-Wno-format"
)

# Apply global definitions to disable unreachable imagery elements
add_compile_definitions(
    EI_CLASSIFIER_HAS_VISUAL_ANOMALY=0
)

