#pragma once
#include "esp_camera.h"
#include "esp_err.h"

esp_err_t camera_driver_init(void);
camera_fb_t *camera_driver_capture(void);      // caller must call camera_driver_return()
void camera_driver_return(camera_fb_t *fb);

int camera_driver_get_width(void);
int camera_driver_get_height(void);