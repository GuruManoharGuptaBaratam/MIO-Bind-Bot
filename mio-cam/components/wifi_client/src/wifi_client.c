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

/* How many times to retry associating before wifi_client_connect() gives
 * up and returns an error. Kept finite on purpose -- an infinite retry
 * loop here is exactly the kind of thing that can starve other tasks
 * and trip the watchdog if it happens on a busy core. */
#define WIFI_CONNECT_MAX_RETRIES 5

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group = NULL;
static esp_netif_t *s_netif = NULL;
static int s_retry_count = 0;
static bool s_initialized = false;
static bool s_connected = false;

static char s_ssid[33] = {0};
static char s_password[65] = {0};

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        if (s_retry_count < WIFI_CONNECT_MAX_RETRIES) {
            esp_wifi_connect();
            s_retry_count++;
            ESP_LOGW(TAG, "retrying wifi connect (%d/%d)", s_retry_count,
                     WIFI_CONNECT_MAX_RETRIES);
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

esp_err_t wifi_client_init(const char *ssid, const char *password)
{
    if (s_initialized) {
        return ESP_OK;
    }
    if (ssid == NULL || strlen(ssid) == 0 || strlen(ssid) > sizeof(s_ssid) - 1) {
        ESP_LOGE(TAG, "invalid ssid");
        return ESP_ERR_INVALID_ARG;
    }

    /* NVS is required for the WiFi driver, and is also where PHY/RF
     * calibration data gets stored between boots. Once a good
     * calibration has been saved here, later boots do a lighter
     * "partial calibration" instead of "full calibration", which is
     * what caused the big current spike in your logs. Erasing flash
     * wipes this out and puts you back to full calibration until the
     * next successful connect. */
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

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char *) wifi_config.sta.ssid, s_ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *) wifi_config.sta.password, s_password,
            sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = strlen(s_password) == 0
                                              ? WIFI_AUTH_OPEN
                                              : WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    /* Radio is configured but NOT started yet -- esp_wifi_start() is
     * what actually kicks off RF calibration and draws the current
     * spike. We defer that to wifi_client_connect() so it only
     * happens right when you need it, not automatically at boot. */

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
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    esp_err_t err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(err));
        return err;
    }

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE,
                                            pdMS_TO_TICKS(timeout_ms));

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
    esp_wifi_disconnect();
    esp_wifi_stop();
    s_connected = false;
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
        /* Small internal buffers are fine here -- we're sending one
         * JPEG frame and reading a short response, not streaming.
         * Keeping these small avoids the client grabbing more heap
         * than it needs on a board that's already tight on RAM. */
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
        ESP_LOGI(TAG, "POST %s -> status %d, %lld bytes sent", url, status,
                 (long long) jpeg_len);
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
