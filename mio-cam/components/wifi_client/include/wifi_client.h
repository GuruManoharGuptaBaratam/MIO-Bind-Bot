#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * wifi_client_init()
 *
 * Registers netif + event handlers and sets the driver config.
 * This does NOT turn the radio on, so it does not draw the big
 * current spike. Safe to call once at boot, right alongside your
 * other init calls (SD config, servos, UART).
 *
 * ssid/password are copied internally, so the strings you pass in
 * (e.g. loaded from SD config.txt) do not need to stay alive after
 * this call returns.
 */
esp_err_t wifi_client_init(const char *ssid, const char *password);

/**
 * wifi_client_connect()
 *
 * Turns the radio on and blocks until connected (got an IP) or the
 * timeout expires. This is the call that triggers the RF calibration
 * current spike, so only call it right before you actually need to
 * send something (e.g. at the top of your kansei/HTTP upload flow),
 * not at boot, and not while servos are actively moving if you can
 * help it.
 *
 * Call this from its own task with a stack of at least 4096 bytes,
 * not directly from a UART RX callback or ISR context.
 */
esp_err_t wifi_client_connect(uint32_t timeout_ms);

/**
 * wifi_client_disconnect()
 *
 * Turns the radio back off. Call this once your HTTP transaction is
 * done so the radio isn't drawing power / isn't a brownout risk
 * during the next servo move or camera capture.
 */
void wifi_client_disconnect(void);

bool wifi_client_is_connected(void);


/**
 * wifi_client_set_credentials()
 *
 * Updates SSID/password on an already-initialized client without
 * re-running nvs_flash_init()/esp_wifi_init() (those must only ever
 * run once). If currently connected, disconnects first. Call this
 * before wifi_client_connect() when a trigger packet carries fresh
 * credentials from Core's SD config.
 */
esp_err_t wifi_client_set_credentials(const char *ssid, const char *password);


/**
 * http_send_image_frame()
 *
 * POSTs one JPEG frame to `url` as a raw binary body
 * (Content-Type: image/jpeg). Blocking call.
 *
 * Requires wifi_client_connect() to have succeeded first.
 *
 * out_status_code (optional, may be NULL): filled with the HTTP
 * status code returned by the server.
 *
 * Returns ESP_OK only if the request completed AND the server
 * returned a 2xx status.
 */
esp_err_t http_send_image_frame(const uint8_t *jpeg_data,
                                 size_t jpeg_len,
                                 const char *url,
                                 int *out_status_code);

#ifdef __cplusplus
}
#endif
