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
#include "i2s_tx.h" // <--- ADD THIS INCLUDE
#include "driver/gpio.h"

// Drives HIGH while SCO/HFP audio is actually connected.
// Wire this to PIN_BT_STATUS on the Inference ESP32 (GPIO21 there).
#define PIN_BT_STATUS_OUT   GPIO_NUM_23

// --- Smoothing ring buffer between HFP callback and I2S ---
// mSBC delivers PCM in small bursts at a fixed cadence; the I2S master
// clock runs continuously and independently. Without this buffer, any
// scheduling jitter on the BT side leaves I2S with nothing to send and
// it transmits silence, corrupting the inference window on the other chip.
#define AUDIO_RB_SIZE_BYTES   4096     // ~128ms of smoothing at 16kHz/16-bit mono
#define I2S_FEED_CHUNK_BYTES  128      // ~4ms per feed iteration, low latency
static RingbufHandle_t s_audio_rb = NULL;
static TaskHandle_t s_i2s_feeder_task_handle = NULL;
static volatile bool s_sco_connected = false;

static uint8_t s_loopback_buf[120] = {0};
static uint32_t s_loopback_len = 0;

// ── Continuous speed-reduction (time-stretch) for the inference-bound copy ──
// Ported from the offline 1.0s -> 1.5s linear-interpolation stretch validated
// in audio_speech_change_hfp_manager_.c (16000 samples -> 24000 samples),
// reworked into a sample-by-sample STREAMING form so it runs continuously on
// live HFP packets instead of needing a full 1-second block captured first.
// It carries one sample of lookback + a fractional position across calls, so
// there's no click/discontinuity at packet boundaries.
//
// STRETCH_RATIO 1.5 => every 2 input samples become 3 output samples =>
// played back at the same 16kHz rate, audio takes 1.5x as long = ~0.667x
// speed. This matches the ratio already confirmed to fix command detection.
// Only this copy is slowed — the phone-call sidetone/loopback below is fed
// from the original, unstretched packet and is completely unaffected.
#define STRETCH_RATIO        1.5f
#define STRETCH_STEP         (1.0f / STRETCH_RATIO)   // ~0.6667 input-samples per output-sample
#define STRETCH_SCRATCH_LEN  256                        // int16 samples of headroom per callback

static float   s_stretch_pos     = 0.0f;
static int16_t s_stretch_prev    = 0;
static bool    s_stretch_primed  = false;

// Slows incoming 16-bit PCM by STRETCH_RATIO. Call once per incoming HFP
// packet, in order — state persists between calls (continuous stream).
// Returns number of bytes written to out_buf (always a whole number of
// int16 samples).
static uint32_t stretch_audio_stream(const uint8_t *in_buf, uint32_t in_len,
                                      uint8_t *out_buf, uint32_t out_buf_cap_bytes)
{
    const int16_t *in_samples = (const int16_t *)in_buf;
    uint32_t in_count = in_len / sizeof(int16_t);
    int16_t *out_samples = (int16_t *)out_buf;
    uint32_t out_cap = out_buf_cap_bytes / sizeof(int16_t);
    uint32_t out_count = 0;

    for (uint32_t i = 0; i < in_count; i++) {
        int16_t cur = in_samples[i];

        if (!s_stretch_primed) {
            // Very first sample ever seen — nothing to interpolate against yet.
            s_stretch_prev = cur;
            s_stretch_primed = true;
            continue;
        }

        while (s_stretch_pos < 1.0f) {
            if (out_count >= out_cap) {
                // Scratch buffer exhausted — should not happen at normal HFP
                // packet sizes (~<=120 bytes), but bail safely rather than
                // overrun the caller's buffer.
                return out_count * sizeof(int16_t);
            }
            float frac = s_stretch_pos;
            float value = (float)s_stretch_prev + ((float)cur - (float)s_stretch_prev) * frac;
            out_samples[out_count++] = (int16_t)value;
            s_stretch_pos += STRETCH_STEP;
        }
        s_stretch_pos -= 1.0f;
        s_stretch_prev = cur;
    }

    return out_count * sizeof(int16_t);
}

static const char *TAG = "MIO_HFP";

static esp_bd_addr_t g_remote_bda = {0};
static bool g_audio_started = false;

static void mio_i2s_feeder_task(void *pvParameters)
{
    ESP_LOGI(TAG, "MIO I2S Feeder Task Started (ring-buffer smoothing)");

    uint8_t feed_buf[I2S_FEED_CHUNK_BYTES];

    while (1) {
        if (!s_sco_connected) {
            // No active SCO session — don't touch the BT stack at all.
            // Idle here instead of spinning; nothing useful to feed yet.
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        size_t item_size = 0;
        // Pull whatever is available, up to one chunk, waiting briefly for
        // fresh data. This does NOT block indefinitely: on timeout we still
        // feed silence below so the I2S clock never stalls or free-runs on
        // stale DMA content.
        uint8_t *data = (uint8_t *)xRingbufferReceiveUpTo(
            s_audio_rb, &item_size, pdMS_TO_TICKS(4), I2S_FEED_CHUNK_BYTES);

        if (data && item_size > 0) {
            memcpy(feed_buf, data, item_size);
            vRingbufferReturnItem(s_audio_rb, data);

            // Pad any shortfall with silence rather than leaving garbage
            if (item_size < I2S_FEED_CHUNK_BYTES) {
                memset(feed_buf + item_size, 0, I2S_FEED_CHUNK_BYTES - item_size);
            }
        } else {
            // Genuine underrun (BT hasn't delivered anything in time) —
            // feed explicit silence so the I2S clock keeps a clean, known
            // cadence instead of repeating stale DMA content.
            memset(feed_buf, 0, I2S_FEED_CHUNK_BYTES);
        }

        stream_audio_over_i2s(feed_buf, I2S_FEED_CHUNK_BYTES);
        // NOTE: esp_hf_ag_outgoing_data_ready() intentionally NOT called here.
        // It's called from incoming_data_callback() instead, once per real
        // SCO packet arrival, which is the actual air-interface cadence.
        // Calling it from this loop (paced by I2S writes, not BT timing)
        // floods the transmit queue — see SCO xmit Q overflow.
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

    // Debug: first 10 bytes, to visually confirm bytes change with voice
    if (len >= 10) {
        ESP_LOGI(TAG, "★ AUDIO ★ %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
            buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9]);
    }

    // Non-blocking push into the smoothing ring buffer. If it's full
    // (feeder task falling behind), drop this packet rather than block
    // the BT callback context.
    //
    // Only the inference-bound copy is slowed down — this is the copy that
    // ends up on I2S -> Inference ESP32. The sidetone copy above (already
    // captured into s_loopback_buf from the original, unstretched `buf`)
    // is untouched, so the caller still hears themselves at normal speed.
    uint8_t stretched_buf[STRETCH_SCRATCH_LEN * sizeof(int16_t)];
    uint32_t stretched_len = stretch_audio_stream(buf, len, stretched_buf, sizeof(stretched_buf));

    if (stretched_len > 0) {
        if (xRingbufferSend(s_audio_rb, stretched_buf, stretched_len, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Audio ring buffer full — dropped %lu bytes", (unsigned long)stretched_len);
        }
    }

    // This must be paced by the real SCO air-interface cadence, not by our
    // own I2S write loop. Each incoming packet arrival IS that real cadence
    // (fixed by the BT radio, ~7.5ms slots), so signal "ready for more
    // outgoing data" exactly once per real incoming packet here — calling
    // it from the feeder loop instead floods the transmit queue
    // (SCO xmit Q overflow) since that loop isn't synchronized to the
    // actual over-the-air timing.
    esp_hf_ag_outgoing_data_ready();
}

static uint32_t outgoing_data_callback(uint8_t *buf, uint32_t len)
{
    if (s_loopback_len > 0) {
        uint32_t copy_len = len < s_loopback_len ? len : s_loopback_len;
        
        // Apply gain to boost volume
        int16_t *samples = (int16_t *)s_loopback_buf;
        int16_t *out = (int16_t *)buf;
        uint32_t num_samples = copy_len / 2;
        
        for (uint32_t i = 0; i < num_samples; i++) {
            int32_t amplified = (int32_t)samples[i] * 3; // increase 3 for more volume
            // Clamp to prevent overflow
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
            ESP_LOGI(TAG, "HFP Profile State Event");
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
                    ESP_LOGI(TAG, "SCO: CONNECTED CVSD");
                    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                    esp_bt_sleep_disable();
                    gpio_set_level(PIN_BT_STATUS_OUT, 1);
                    s_sco_connected = true;
                    break;

                case ESP_HF_AUDIO_STATE_CONNECTED_MSBC:
                    ESP_LOGI(TAG, "SCO: CONNECTED mSBC");
                    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                    esp_bt_sleep_disable();
                    gpio_set_level(PIN_BT_STATUS_OUT, 1);
                    s_sco_connected = true;
                    break;

                case ESP_HF_AUDIO_STATE_DISCONNECTED:
                    ESP_LOGI(TAG, "SCO: DISCONNECTED — attempting reconnect");
                    gpio_set_level(PIN_BT_STATUS_OUT, 0);
                    s_sco_connected = false;
                    g_audio_started = false;
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

    // === INTEGRATION POINT ===
    // Spin up the physical I2S peripheral hardware master configuration
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
    esp_hf_ag_slc_connect(remote_bda);
}