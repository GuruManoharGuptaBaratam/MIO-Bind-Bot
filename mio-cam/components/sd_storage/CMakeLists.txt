idf_component_register(
    SRCS "src/sd_storage.c"
    INCLUDE_DIRS "include"
    REQUIRES fatfs sdmmc vfs
)