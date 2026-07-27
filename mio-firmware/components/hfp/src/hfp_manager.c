#include "hfp_manager.h"
#include <string.h>
#include "esp_log.h"
#include "esp_hf_ag_api.h"
#include "esp_gap_bt_api.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "i2s_tx.h"
#include "driver/gpio.h"
#include "tft_display.h"
#include "audio_feedback.h"

// Drives HIGH while SCO/HFP audio is actually connected.
// Wire this to PIN_BT_STATUS on the Inference ESP32 (GPIO21 there).
#define PIN_BT_STATUS_OUT   GPIO_NUM_23

// --- Smoothing ring buffer between HFP callback and I2S ---
#define AUDIO_RB_SIZE_BYTES   4096     // ~128ms of smoothing at 16kHz/16-bit mono
#define I2S_FEED_CHUNK_BYTES  128      // ~4ms per feed iteration, low latency
static RingbufHandle_t s_audio_rb = NULL;
static TaskHandle_t s_i2s_feeder_task_handle = NULL;
static volatile bool s_sco_connected = false;

static uint8_t s_loopback_buf[120] = {0};
static uint32_t s_loopback_len = 0;

static const char *TAG = "MIO_HFP";

static esp_bd_addr_t g_remote_bda = {0};
static bool g_audio_started = false;

// Dynamic Codec & Sample Rate tracking
static volatile hfp_codec_type_t g_active_codec = HFP_CODEC_UNKNOWN;
static volatile uint32_t g_negotiated_sample_rate = 16000; // Default fallback to 16kHz

uint32_t hfp_get_negotiated_sample_rate(void) {
    return g_negotiated_sample_rate;
}

hfp_codec_type_t hfp_get_active_codec(void) {
    return g_active_codec;
}

static esp_bd_addr_t g_pending_connect_bda = {0};
static bool g_pending_connect = false;

static void mio_i2s_feeder_task(void *pvParameters)
{
    ESP_LOGI(TAG, "MIO I2S Feeder Task Started (ring-buffer smoothing)");

    uint8_t feed_buf[I2S_FEED_CHUNK_BYTES];

    while (1) {
        if (!s_sco_connected) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        size_t item_size = 0;
        uint8_t *data = (uint8_t *)xRingbufferReceiveUpTo(
            s_audio_rb, &item_size, pdMS_TO_TICKS(4), I2S_FEED_CHUNK_BYTES);

        if (data && item_size > 0) {
            memcpy(feed_buf, data, item_size);
            vRingbufferReturnItem(s_audio_rb, data);

            if (item_size < I2S_FEED_CHUNK_BYTES) {
                memset(feed_buf + item_size, 0, I2S_FEED_CHUNK_BYTES - item_size);
            }
        } else {
            memset(feed_buf, 0, I2S_FEED_CHUNK_BYTES);
        }

        stream_audio_over_i2s(feed_buf, I2S_FEED_CHUNK_BYTES);
    }
}

static void incoming_data_callback(const uint8_t *buf, uint32_t len)
{
    if (!s_audio_rb || !buf || len == 0) {
        return;
    }

    // Keep the loopback copy (used by outgoing_data_callback for sidetone)
    uint32_t loop_len = len < sizeof(s_loopback_buf) ? len : sizeof(s_loopback_buf);
    memcpy(s_loopback_buf, buf, loop_len);
    s_loopback_len = loop_len;

    // Resample 8 kHz CVSD -> 16 kHz on ingress via 2x linear interpolation
    if (g_negotiated_sample_rate == 8000) {
        const int16_t *pcm_in = (const int16_t *)buf;
        size_t sample_count = len / sizeof(int16_t);
        int16_t resampled_buf[sample_count * 2];

        for (size_t i = 0; i < sample_count; i++) {
            int16_t current = pcm_in[i];
            int16_t next = (i + 1 < sample_count) ? pcm_in[i + 1] : current;

            resampled_buf[i * 2]     = current;
            resampled_buf[i * 2 + 1] = (int16_t)(((int32_t)current + (int32_t)next) / 2);
        }

        if (xRingbufferSend(s_audio_rb, resampled_buf, sizeof(resampled_buf), 0) != pdTRUE) {
            ESP_LOGW(TAG, "Audio ring buffer full — dropped %zu bytes (resampled)", sizeof(resampled_buf));
        }
    } else {
        // Direct pass-through for 16 kHz mSBC
        if (xRingbufferSend(s_audio_rb, buf, len, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Audio ring buffer full — dropped %lu bytes", (unsigned long)len);
        }
    }

    esp_hf_ag_outgoing_data_ready();
}

static uint32_t outgoing_data_callback(uint8_t *buf, uint32_t len)
{
    size_t fb_len = audio_feedback_pull_frame(buf, len);
    if (fb_len > 0) {
        if (fb_len < len) {
            memset(buf + fb_len, 0, len - fb_len);
        }
        return len;
    }

    if (s_loopback_len > 0) {
        uint32_t copy_len = len < s_loopback_len ? len : s_loopback_len;
        
        int16_t *samples = (int16_t *)s_loopback_buf;
        int16_t *out = (int16_t *)buf;
        uint32_t num_samples = copy_len / 2;
        
        for (uint32_t i = 0; i < num_samples; i++) {
            int32_t amplified = (int32_t)samples[i] * 3;
            if (amplified > 32767) amplified = 32767;
            if (amplified < -32768) amplified = -32768;
            out[i] = (int16_t)amplified;
        }
        
        if (copy_len < len) {
            memset(buf + copy_len, 0, len - copy_len);
        }
    } else {
        memset(buf, 0, len);
    }
    return len;
}

static void hfp_callback(
    esp_hf_cb_event_t event,
    esp_hf_cb_param_t *param)
{
    switch(event)
    {
        case ESP_HF_PROF_STATE_EVT:
            ESP_LOGI(TAG, "HFP Profile State Event — profile ready");
            if (g_pending_connect) {
                ESP_LOGI(TAG, "Connecting to pending target now");
                g_pending_connect = false;
                esp_hf_ag_slc_connect(g_pending_connect_bda);
            }
            break;

        case ESP_HF_CONNECTION_STATE_EVT:
            ESP_LOGI(TAG, "SLC STATE: %d", param->conn_stat.state);
            memcpy(g_remote_bda, param->conn_stat.remote_bda, ESP_BD_ADDR_LEN);

            if(param->conn_stat.state == ESP_HF_CONNECTION_STATE_SLC_CONNECTED)
            {
                ESP_LOGI(TAG, "SLC ESTABLISHED — opening audio");
                if(!g_audio_started)
                {
                    g_audio_started = true;
                    esp_err_t ret = esp_hf_ag_audio_connect(g_remote_bda);
                    ESP_LOGI(TAG, "Audio connect result: %s", esp_err_to_name(ret));
                }
            }
            else if(param->conn_stat.state == ESP_HF_CONNECTION_STATE_DISCONNECTED)
            {
                ESP_LOGW(TAG, "SLC DISCONNECTED — re-enabling connectable/discoverable so CMF Buds can auto-reconnect");
                
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                
                memset(g_remote_bda, 0, ESP_BD_ADDR_LEN);
                g_audio_started = false;
            }
            break;

        case ESP_HF_AUDIO_STATE_EVT:
            switch(param->audio_stat.state)
            {
                case ESP_HF_AUDIO_STATE_CONNECTING:
                    ESP_LOGI(TAG, "SCO: CONNECTING");
                    break;

                case ESP_HF_AUDIO_STATE_CONNECTED:
                    g_active_codec = HFP_CODEC_CVSD;
                    g_negotiated_sample_rate = 8000;
                    ESP_LOGI(TAG, "SCO: CONNECTED CVSD (8 kHz dynamically set)");
                    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                    esp_bt_sleep_disable();
                    gpio_set_level(PIN_BT_STATUS_OUT, 1);
                    s_sco_connected = true;
                    tft_display_on_bt_state(true);
                    audio_feedback_set_wideband(false);
                    audio_feedback_flush_pending();
                    break;

                case ESP_HF_AUDIO_STATE_CONNECTED_MSBC:
                    g_active_codec = HFP_CODEC_MSBC;
                    g_negotiated_sample_rate = 16000;
                    ESP_LOGI(TAG, "SCO: CONNECTED mSBC (16 kHz dynamically set)");
                    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                    esp_bt_sleep_disable();
                    gpio_set_level(PIN_BT_STATUS_OUT, 1);
                    s_sco_connected = true;
                    tft_display_on_bt_state(true);
                    audio_feedback_set_wideband(true);
                    audio_feedback_flush_pending();
                    break;

                case ESP_HF_AUDIO_STATE_DISCONNECTED:
                    ESP_LOGI(TAG, "SCO: DISCONNECTED — attempting reconnect");
                    gpio_set_level(PIN_BT_STATUS_OUT, 0);
                    s_sco_connected = false;
                    g_audio_started = false;
                    tft_display_on_bt_state(false);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    if (memcmp(g_remote_bda, (esp_bd_addr_t){0}, ESP_BD_ADDR_LEN) != 0) {
                        g_audio_started = true;
                        esp_err_t r = esp_hf_ag_audio_connect(g_remote_bda);
                        ESP_LOGI(TAG, "SCO reconnect attempt: %s", esp_err_to_name(r));
                    }
                    break;
            }
            break;

        case ESP_HF_CIND_RESPONSE_EVT:
            ESP_LOGI(TAG, "CIND Request From Headset");
            esp_err_t ret_cind = esp_hf_ag_cind_response(
                param->cind_rep.remote_addr,
                ESP_HF_CALL_STATUS_NO_CALLS,
                ESP_HF_CALL_SETUP_STATUS_IDLE,
                ESP_HF_NETWORK_STATE_AVAILABLE,
                5,
                ESP_HF_ROAMING_STATUS_INACTIVE,
                5,
                ESP_HF_CALL_HELD_STATUS_NONE
            );
            ESP_LOGI(TAG, "CIND Response Result: %s", esp_err_to_name(ret_cind));
            break;

        case ESP_HF_VOLUME_CONTROL_EVT:
            ESP_LOGI(TAG, "Volume Change: type=%d volume=%d",
                     param->volume_control.type, param->volume_control.volume);
            break;

        case ESP_HF_UNAT_RESPONSE_EVT: {
            const char *cmd = param->unat_rep.unat;
            ESP_LOGI(TAG, "Unknown AT: %s", cmd);

            if (strncmp(cmd, "+XAPL=", 6) == 0) {
                esp_hf_ag_unknown_at_send(g_remote_bda, "+XAPL=iPhone,7");
            } else if (strncmp(cmd, "+IPHONEACCEV", 12) == 0) {
                esp_hf_ag_unknown_at_send(g_remote_bda, "OK");
            } else if (strncmp(cmd, "+CGMI", 5) == 0) {
                esp_hf_ag_unknown_at_send(g_remote_bda, "Apple");
            } else {
                esp_hf_ag_unknown_at_send(g_remote_bda, "OK");
            }
            break;
        }

        case ESP_HF_WBS_RESPONSE_EVT:
            ESP_LOGI(TAG, "WBS RESPONSE codec=%d", param->wbs_rep.codec);
            break;

        case ESP_HF_BCS_RESPONSE_EVT:
            ESP_LOGI(TAG, "BCS RECEIVED mode=%d (Handled)", param->bcs_rep.mode);
            if (param->bcs_rep.mode == ESP_HF_WBS_NO) {
                g_active_codec = HFP_CODEC_CVSD;
                g_negotiated_sample_rate = 8000;
            } else if (param->bcs_rep.mode == ESP_HF_WBS_YES) {
                g_active_codec = HFP_CODEC_MSBC;
                g_negotiated_sample_rate = 16000;
            }
            break;

        default:
            break;
    }
}

void hfp_init(void)
{
    ESP_LOGI(TAG, "Initializing HFP Stack with Multithread Offloading");

    esp_bt_sleep_disable();

    gpio_config_t bt_status_out_cfg = {
        .pin_bit_mask = (1ULL << PIN_BT_STATUS_OUT),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&bt_status_out_cfg));
    gpio_set_level(PIN_BT_STATUS_OUT, 0);

    init_i2s_master_tx();

    s_audio_rb = xRingbufferCreate(AUDIO_RB_SIZE_BYTES, RINGBUF_TYPE_BYTEBUF);
    if (!s_audio_rb) {
        ESP_LOGE(TAG, "Failed to create audio ring buffer");
    }

    xTaskCreatePinnedToCore(
        mio_i2s_feeder_task,
        "mio_i2s_feeder",
        4096,
        NULL,
        22,
        &s_i2s_feeder_task_handle,
        1
    );

    esp_hf_ag_register_callback(hfp_callback);
    esp_hf_ag_init();
    esp_hf_ag_register_data_callback(incoming_data_callback, outgoing_data_callback);
}

void hfp_connect(esp_bd_addr_t remote_bda)
{
    memcpy(g_pending_connect_bda, remote_bda, ESP_BD_ADDR_LEN);
    g_pending_connect = true;
    ESP_LOGI(TAG, "hfp_connect() called — will connect once HFP profile confirms ready");

    esp_hf_ag_slc_connect(remote_bda);
}