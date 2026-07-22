idf_component_register(
    SRCS "src/sd_config.c"
    INCLUDE_DIRS "include"
    REQUIRES ""
    PRIV_REQUIRES fatfs sdmmc driver spi_flash nvs_flash 
)
 