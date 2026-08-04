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

// Standard connect using currently configured credentials
esp_err_t wifi_client_connect(uint32_t timeout_ms);

// Adaptive connect: tries primary credentials first; if provided/configured, falls back to secondary credentials
esp_err_t wifi_client_connect_adaptive(const char *primary_ssid, const char *primary_pass,
                                       const char *backup_ssid, const char *backup_pass,
                                       uint32_t per_attempt_timeout_ms);

void wifi_client_disconnect(void);
bool wifi_client_is_connected(void);

esp_err_t wifi_client_set_credentials(const char *ssid, const char *password);

esp_err_t http_send_image_frame(const uint8_t *jpeg_data,
                                 size_t jpeg_len,
                                 const char *url,
                                 int *out_status_code);

esp_err_t http_send_batch_kansei_frames(camera_fb_t **frames, size_t frame_count,
                                        const uint8_t *user_ref_buf, size_t user_ref_len,
                                        const char *url,
                                        uint8_t **out_audio_buf, size_t *out_audio_len,
                                        char *out_text_buf, size_t out_text_buf_size,
                                        int *out_status_code);
esp_err_t wifi_client_connect_with_fallback(const char *primary_ssid, const char *primary_pass,
                                              const char *backup_ssid, const char *backup_pass,
                                              uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif