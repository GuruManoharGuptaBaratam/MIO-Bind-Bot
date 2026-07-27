#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t wifi_client_init(const char *ssid, const char *password);
esp_err_t wifi_client_connect(uint32_t timeout_ms);
void wifi_client_disconnect(void);
bool wifi_client_is_connected(void);

esp_err_t wifi_client_set_credentials(const char *ssid, const char *password);

esp_err_t http_send_image_frame(const uint8_t *jpeg_data,
                                 size_t jpeg_len,
                                 const char *url,
                                 int *out_status_code);

// Batch sends all swept frames in a single multipart HTTP POST request
esp_err_t http_send_batch_kansei_frames(camera_fb_t **frames, size_t frame_count, 
                                        const uint8_t *user_ref_buf, size_t user_ref_len, 
                                        const char *url, int *out_status_code);

#ifdef __cplusplus
}
#endif