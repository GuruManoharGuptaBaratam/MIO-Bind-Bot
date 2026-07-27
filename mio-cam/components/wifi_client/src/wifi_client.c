#include <string.h>
#include "wifi_client.h"

#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
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

esp_err_t http_send_batch_kansei_frames(camera_fb_t **frames, size_t frame_count, 
                                        const uint8_t *user_ref_buf, size_t user_ref_len, 
                                        const char *url, int *status_code)
{
    if (!s_connected) {
        ESP_LOGE(TAG, "not connected, call wifi_client_connect() first");
        return ESP_ERR_INVALID_STATE;
    }
    if (!frames || frame_count == 0 || !url) {
        return ESP_ERR_INVALID_ARG;
    }

    const char *boundary = "----MioKanseiBoundary7MA4YWxkTrZu0gW";
    char header_buf[256];

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 12000,
        .buffer_size = 1024,
        .buffer_size_tx = 1024,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init http client");
        return ESP_FAIL;
    }

    snprintf(header_buf, sizeof(header_buf), "multipart/form-data; boundary=%s", boundary);
    esp_http_client_set_header(client, "Content-Type", header_buf);

    esp_err_t err = esp_http_client_open(client, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    for (size_t i = 0; i < frame_count; i++) {
        if (!frames[i]) continue;

        int len = snprintf(header_buf, sizeof(header_buf),
            "--%s\r\n"
            "Content-Disposition: form-data; name=\"frame_%d\"; filename=\"frame_%d.jpg\"\r\n"
            "Content-Type: image/jpeg\r\n\r\n",
            boundary, (int)i, (int)i);

        esp_http_client_write(client, header_buf, len);
        esp_http_client_write(client, (const char *)frames[i]->buf, frames[i]->len);
        esp_http_client_write(client, "\r\n", 2);
    }

    if (user_ref_buf && user_ref_len > 0) {
        int len = snprintf(header_buf, sizeof(header_buf),
            "--%s\r\n"
            "Content-Disposition: form-data; name=\"user_ref\"; filename=\"user_ref.jpg\"\r\n"
            "Content-Type: image/jpeg\r\n\r\n",
            boundary);

        esp_http_client_write(client, header_buf, len);
        esp_http_client_write(client, (const char *)user_ref_buf, user_ref_len);
        esp_http_client_write(client, "\r\n", 2);
    }

    snprintf(header_buf, sizeof(header_buf), "--%s--\r\n", boundary);
    esp_http_client_write(client, header_buf, strlen(header_buf));

    int content_len = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    
    if (status_code) {
        *status_code = status;
    }

    ESP_LOGI(TAG, "Batch POST HTTP Status = %d, content_length = %d", status, content_len);

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    return (status >= 200 && status < 300) ? ESP_OK : ESP_FAIL;
}