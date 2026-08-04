#include <string.h>
#include "wifi_client.h"

#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include "esp_crt_bundle.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"

static const char *TAG = "wifi_client";

#define WIFI_CONNECT_MAX_RETRIES 5

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group = NULL;
static esp_netif_t *s_netif = NULL;
static int s_retry_count = 0;
static bool s_initialized = false;
static bool s_connected = false;
static bool s_disconnect_requested = false;

static char s_ssid[33] = {0};
static char s_password[65] = {0};

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        if (s_disconnect_requested) {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        } else if (s_retry_count < WIFI_CONNECT_MAX_RETRIES) {
            esp_wifi_connect();
            s_retry_count++;
            ESP_LOGW(TAG, "retrying wifi connect (%d/%d)", s_retry_count, WIFI_CONNECT_MAX_RETRIES);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_count = 0;
        s_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_client_set_credentials(const char *ssid, const char *password)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "call wifi_client_init() first");
        return ESP_ERR_INVALID_STATE;
    }
    if (ssid == NULL || strlen(ssid) == 0 || strlen(ssid) > sizeof(s_ssid) - 1) {
        ESP_LOGE(TAG, "invalid ssid");
        return ESP_ERR_INVALID_ARG;
    }
    if (s_connected) {
        ESP_LOGW(TAG, "changing credentials while connected -- disconnecting first");
        wifi_client_disconnect();
    }

    memset(s_ssid, 0, sizeof(s_ssid));
    memset(s_password, 0, sizeof(s_password));
    strncpy(s_ssid, ssid, sizeof(s_ssid) - 1);
    if (password != NULL) {
        strncpy(s_password, password, sizeof(s_password) - 1);
    }

    wifi_config_t wifi_config = {0};
    strncpy((char *) wifi_config.sta.ssid, s_ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *) wifi_config.sta.password, s_password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = strlen(s_password) == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "credentials updated: SSID='%s'", s_ssid);
    return ESP_OK;
}

esp_err_t wifi_client_init(const char *ssid, const char *password)
{
    if (s_initialized) {
        return ESP_OK;
    }
    if (ssid == NULL || strlen(ssid) == 0 || strlen(ssid) > sizeof(s_ssid) - 1) {
        ESP_LOGE(TAG, "invalid ssid");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    strncpy(s_ssid, ssid, sizeof(s_ssid) - 1);
    if (password != NULL) {
        strncpy(s_password, password, sizeof(s_password) - 1);
    }

    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char *) wifi_config.sta.ssid, s_ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *) wifi_config.sta.password, s_password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = strlen(s_password) == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    s_initialized = true;
    ESP_LOGI(TAG, "wifi_client_init done (radio not started yet)");
    return ESP_OK;
}

esp_err_t wifi_client_connect(uint32_t timeout_ms)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "call wifi_client_init() first");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_connected) {
        return ESP_OK;
    }

    s_retry_count = 0;
    s_disconnect_requested = false;
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    esp_err_t err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(err));
        return err;
    }

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE, pdMS_TO_TICKS(timeout_ms));

    if (bits & WIFI_CONNECTED_BIT) {
        return ESP_OK;
    }

    ESP_LOGE(TAG, "wifi connect failed or timed out");
    esp_wifi_stop();
    return ESP_FAIL;
}

void wifi_client_disconnect(void)
{
    if (!s_initialized) {
        return;
    }
    s_disconnect_requested = true;
    esp_wifi_disconnect();
    esp_wifi_stop();
    s_connected = false;
    s_disconnect_requested = false;
}

bool wifi_client_is_connected(void)
{
    return s_connected;
}

esp_err_t http_send_image_frame(const uint8_t *jpeg_data,
                                 size_t jpeg_len,
                                 const char *url,
                                 int *out_status_code)
{
    if (!s_connected) {
        ESP_LOGE(TAG, "not connected, call wifi_client_connect() first");
        return ESP_ERR_INVALID_STATE;
    }
    if (jpeg_data == NULL || jpeg_len == 0 || url == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 8000,
        .buffer_size = 1024,
        .buffer_size_tx = 1024,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_http_client_set_header(client, "Content-Type", "image/jpeg");
    esp_http_client_set_post_field(client, (const char *) jpeg_data, jpeg_len);

    esp_err_t err = esp_http_client_perform(client);
    int status = -1;
    if (err == ESP_OK) {
        status = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "POST %s -> status %d, %lld bytes sent", url, status, (long long) jpeg_len);
    } else {
        ESP_LOGE(TAG, "esp_http_client_perform failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);

    if (out_status_code != NULL) {
        *out_status_code = status;
    }

    if (err != ESP_OK) {
        return err;
    }
    return (status >= 200 && status < 300) ? ESP_OK : ESP_FAIL;
}

// Raw 16kHz mono PCM16 runs ~32 KB/s -- a ~45s worst-case description tops
// out around 1.5MB. This is a fallback cap only; we prefer the real
// Content-Length from the response headers when the server provides one
// (Modal's Response(content=bytes) sets it, since the body isn't chunked).
#define KANSEI_MAX_RESPONSE_BYTES (2048 * 1024)

// Compile-time swap point: set this to your real Modal secret, or better,
// load it from SD config alongside the WiFi credentials so it's not baked
// into the firmware image.
#define MIO_SHARED_SECRET "4ea256147e1602ff22bba0499e26bd4d"

esp_err_t http_send_batch_kansei_frames(camera_fb_t **frames, size_t frame_count,
                                        const uint8_t *user_ref_buf, size_t user_ref_len,
                                        const char *url,
                                        uint8_t **out_audio_buf, size_t *out_audio_len,
                                        char *out_text_buf, size_t out_text_buf_size,
                                        int *status_code)
{
    if (!s_connected) {
        ESP_LOGE(TAG, "not connected, call wifi_client_connect() first");
        return ESP_ERR_INVALID_STATE;
    }
    if (!frames || frame_count == 0 || !url || !out_audio_buf || !out_audio_len) {
        return ESP_ERR_INVALID_ARG;
    }

    *out_audio_buf = NULL;
    *out_audio_len = 0;
    if (out_text_buf && out_text_buf_size > 0) {
        out_text_buf[0] = '\0';
    }

    // Change this line in wifi_client.c:
    const char *boundary = "MioKanseiBoundary7MA4YWxkTrZu0gW";

    // Build every part's header up front and sum the exact body length so we
    // can declare a real Content-Length instead of opening with -1 (chunked
    // transfer encoding). Modal's front-end proxy resets the connection
    // mid-upload when it doesn't get an upfront length -- this is a common
    // mismatch between esp_http_client's "-1 means chunked" default and
    // gateways that expect to know the body size before forwarding.
#define WIFI_MAX_MULTIPART_PARTS 9
    if (frame_count > WIFI_MAX_MULTIPART_PARTS - 1) {
        ESP_LOGE(TAG, "frame_count %zu exceeds max supported parts (%d)",
                  frame_count, WIFI_MAX_MULTIPART_PARTS - 1);
        return ESP_ERR_INVALID_ARG;
    }

    static char part_header[WIFI_MAX_MULTIPART_PARTS][256];
    int part_header_len[WIFI_MAX_MULTIPART_PARTS] = {0};
    size_t part_count = 0;
    size_t body_len = 0;

    for (size_t i = 0; i < frame_count; i++) {
        if (!frames[i]) continue;
        int len = snprintf(part_header[part_count], sizeof(part_header[part_count]),
            "--%s\r\n"
            "Content-Disposition: form-data; name=\"frame_%d\"; filename=\"frame_%d.jpg\"\r\n"
            "Content-Type: image/jpeg\r\n\r\n",
            boundary, (int)i, (int)i);
        part_header_len[part_count] = len;
        body_len += (size_t)len + frames[i]->len + 2; // +2 for trailing \r\n after the data
        part_count++;
    }

    bool has_user_ref = (user_ref_buf && user_ref_len > 0);
    if (has_user_ref) {
        int len = snprintf(part_header[part_count], sizeof(part_header[part_count]),
            "--%s\r\n"
            "Content-Disposition: form-data; name=\"user_ref\"; filename=\"user_ref.jpg\"\r\n"
            "Content-Type: image/jpeg\r\n\r\n",
            boundary);
        part_header_len[part_count] = len;
        body_len += (size_t)len + user_ref_len + 2;
        part_count++;
    }

    char closing_boundary[64];
    int closing_len = snprintf(closing_boundary, sizeof(closing_boundary), "--%s--\r\n", boundary);
    body_len += (size_t)closing_len;

    ESP_LOGI(TAG, "kansei multipart body: %zu bytes across %zu parts (declared Content-Length, not chunked)",
              body_len, part_count);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        // Cold-start CPU inference + edge-tts round trip can run long;
        // 12s was fine for a single-frame POST but is too tight here.
        .timeout_ms = 90000,
        .buffer_size = 1024,
        .buffer_size_tx = 1024,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init http client");
        return ESP_FAIL;
    }

    char content_type_header[256];
    snprintf(content_type_header, sizeof(content_type_header), "multipart/form-data; boundary=%s", boundary);
    esp_http_client_set_header(client, "Content-Type", content_type_header);
    esp_http_client_set_header(client, "X-Mio-Key", MIO_SHARED_SECRET);

    esp_err_t err = esp_http_client_open(client, (int)body_len); // real length, not -1
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    size_t hdr_idx = 0;
    for (size_t i = 0; i < frame_count; i++) {
        if (!frames[i]) continue;
        esp_http_client_write(client, part_header[hdr_idx], part_header_len[hdr_idx]);
        esp_http_client_write(client, (const char *)frames[i]->buf, frames[i]->len);
        esp_http_client_write(client, "\r\n", 2);
        hdr_idx++;
    }

    if (has_user_ref) {
        esp_http_client_write(client, part_header[hdr_idx], part_header_len[hdr_idx]);
        esp_http_client_write(client, (const char *)user_ref_buf, user_ref_len);
        esp_http_client_write(client, "\r\n", 2);
        hdr_idx++;
    }

    esp_http_client_write(client, closing_boundary, closing_len);

    int content_len = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);

    if (status_code) {
        *status_code = status;
    }

    ESP_LOGI(TAG, "Batch POST HTTP Status = %d, content_length = %d", status, content_len);

    esp_err_t result = ESP_FAIL;

    if (status >= 200 && status < 300) {
        // Prefer the real Content-Length so we don't over-allocate PSRAM on
        // every call; fall back to the hard cap if the server didn't send
        // one (e.g. chunked transfer).
        size_t alloc_cap = (content_len > 0 && (size_t)content_len <= KANSEI_MAX_RESPONSE_BYTES)
                                ? (size_t)content_len
                                : KANSEI_MAX_RESPONSE_BYTES;
        uint8_t *response_buf = heap_caps_malloc(alloc_cap, MALLOC_CAP_SPIRAM);
        if (!response_buf) {
            ESP_LOGE(TAG, "Failed to allocate %u byte PSRAM response buffer", (unsigned)alloc_cap);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return ESP_ERR_NO_MEM;
        }

        size_t total_read = 0;
        while (total_read < alloc_cap) {
            int r = esp_http_client_read(client, (char *)(response_buf + total_read),
                                          alloc_cap - total_read);
            if (r < 0) {
                ESP_LOGE(TAG, "esp_http_client_read failed at offset %u", (unsigned)total_read);
                break;
            }
            if (r == 0) {
                break; // response fully read
            }
            total_read += (size_t)r;
        }

        ESP_LOGI(TAG, "Read %u bytes of response body", (unsigned)total_read);

        // Frame: [4-byte LE text length][UTF-8 text][remaining bytes = mSBC audio]
        if (total_read < 4) {
            ESP_LOGE(TAG, "Response too short to contain framing header");
            heap_caps_free(response_buf);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return ESP_FAIL;
        }

        uint32_t text_len = (uint32_t)response_buf[0]
                           | ((uint32_t)response_buf[1] << 8)
                           | ((uint32_t)response_buf[2] << 16)
                           | ((uint32_t)response_buf[3] << 24);

        if (4 + text_len > total_read) {
            ESP_LOGE(TAG, "Malformed response: text_len=%u exceeds body size=%u",
                      (unsigned)text_len, (unsigned)total_read);
            heap_caps_free(response_buf);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return ESP_FAIL;
        }

        if (out_text_buf && out_text_buf_size > 0) {
            size_t copy_len = text_len < (out_text_buf_size - 1) ? text_len : (out_text_buf_size - 1);
            memcpy(out_text_buf, response_buf + 4, copy_len);
            out_text_buf[copy_len] = '\0';
        }

        size_t audio_len = total_read - 4 - text_len;
        uint8_t *audio_buf = heap_caps_malloc(audio_len, MALLOC_CAP_SPIRAM);
        if (!audio_buf) {
            ESP_LOGE(TAG, "Failed to allocate %u byte audio buffer", (unsigned)audio_len);
            heap_caps_free(response_buf);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return ESP_ERR_NO_MEM;
        }
        memcpy(audio_buf, response_buf + 4 + text_len, audio_len);
        heap_caps_free(response_buf);

        *out_audio_buf = audio_buf;
        *out_audio_len = audio_len;
        ESP_LOGI(TAG, "Parsed response: text_len=%u audio_len=%u",
                  (unsigned)text_len, (unsigned)audio_len);
        result = ESP_OK;
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    return result;
}