#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

esp_err_t sd_storage_init(void);   // mounts SDMMC 1-line, once at boot

// One video = one open/write.../close cycle. Not reentrant --
// only one video open at a time (matches your single kiroku job model).
esp_err_t sd_storage_open_video(uint32_t width, uint32_t height, uint8_t fps,
                                 char *out_path, size_t out_path_len);
esp_err_t sd_storage_write_frame(const uint8_t *jpeg_data, size_t jpeg_len);
esp_err_t sd_storage_close_video(void);