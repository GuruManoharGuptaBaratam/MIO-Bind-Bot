#include "inference_engine.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"  // <--- ADDED: Safe async buffer passing
#include "driver/uart.h"
#include "driver/i2s_std.h"   // <--- ADDED: Modern ESP-IDF v5/v6 I2S driver
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <string.h>

// Include standard Edge Impulse processing libraries
#include "edge-impulse-sdk/classifier/ei_run_dsp.h" 
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

// Load the multi-impulse parameter arrays
#include "model-parameters/model_variables.h"
#include <inttypes.h>

extern const ei_impulse_t impulse_1036490_1;
extern const ei_impulse_t impulse_1037438_3;
extern ei_impulse_handle_t impulse_handle_1037438_3;

static const char *TAG = "INF_ENGINE";

// ── Buffers ───────────────────────────────────────────────────────────────────
static int16_t *g_ww_ring     = NULL;   // rolling window for wakeword inference
static int16_t *g_cmd_capture = NULL;   // flat 2-second buffer for command inference
static int16_t *g_uart_rx_buf = NULL;   // staging buffer for processing I2S data

static i2s_chan_handle_t rx_chan   = NULL; // I2S Rx Handle
static RingbufHandle_t s_audio_rb  = NULL; // FreeRTOS Ringbuffer

#define CHUNK_SAMPLES      512

// Pin layout configuration matching the transmitter chip
#define PIN_SLAVE_BCLK     GPIO_NUM_4   
#define PIN_SLAVE_WS       GPIO_NUM_5   
#define PIN_DATA_IN        GPIO_NUM_19  

// Reads HIGH when Core ESP32 has an active SCO/HFP audio link.
// Wire this to a free GPIO on the Core chip, driven in hfp_manager.c.
#define PIN_BT_STATUS      GPIO_NUM_21

// ── State machine 
typedef enum {
    STATE_WAKEWORD,
    STATE_CAPTURE,
    // STATE_COMMAND,
} ie_state_t;

static ie_state_t g_state         = STATE_WAKEWORD;
static uint32_t   g_ww_write_pos  = 0;

// ── Rolling command-window classification state ─────────────────────────────
// g_cmd_capture is a RING buffer sized to exactly one classification window
// (CMD_MODEL_WINDOW_SAMPLES), not the full 5-second listen duration - storing
// 5s flat (160KB) doesn't fit in internal DRAM with no PSRAM. Elapsed time for
// the 5s timeout is tracked separately via g_cmd_total_written (a counter,
// not a buffer size).
static uint32_t   g_cmd_ring_pos       = 0;   // next write index into the ring, wraps at CMD_MODEL_WINDOW_SAMPLES
static uint32_t   g_cmd_total_written  = 0;   // total samples seen since capture started (monotonic, for timeout)
static uint32_t   g_cmd_last_check_pos = 0;   // throttles how often we classify
static bool       g_vad_triggered      = false;  // true once real speech energy detected
static char       g_pending_label[16]  = {0};    // last check's winning command, for consecutive-hit confirmation
static int        g_pending_count      = 0;

#define CMD_CHECK_INTERVAL_SAMPLES  4000       // check every ~0.25s of new audio
#define VAD_ENERGY_THRESHOLD        250        // mean abs raw-sample amplitude that counts as "speech started"
#define CMD_CONFIDENCE_MARGIN       0.15f      // best must beat 2nd-best by this much
#define CMD_REQUIRED_CONSECUTIVE    2          // same label must win this many checks in a row

// ── EI signal callbacks 
static int ww_get_data(unsigned int offset, unsigned int length, float *out)
{
    for (size_t i = 0; i < length; i++) {
        uint32_t idx = (g_ww_write_pos + offset + i) % WW_WINDOW_SAMPLES;
        out[i] = (float)g_ww_ring[idx];   // raw int16 magnitude, matches int16_to_float() used by run_static_test()
    }
    return 0;
}

static int cmd_get_data(unsigned int offset, unsigned int length, float *out)
{
    const float GAIN = 2.0f;  // tune 1.5-3.0 based on live confidence numbers

    for (size_t i = 0; i < length; i++) {
        // g_cmd_ring_pos currently points at the OLDEST sample (next write slot,
        // since the ring is exactly one window long once full) - reading forward
        // from here with wraparound gives chronological oldest -> newest order.
        uint32_t idx = (g_cmd_ring_pos + offset + i) % CMD_MODEL_WINDOW_SAMPLES;
        float sample = (float)g_cmd_capture[idx];
        sample *= GAIN;
        if (sample >  32767.0f) sample =  32767.0f;
        if (sample < -32768.0f) sample = -32768.0f;
        out[i] = sample;
    }
    return 0;
}

// ── Outbound UART configuration (Kept solely for sending commands back to Core)
static void uart_init(void)
{
    uart_config_t uart_config = {};
    uart_config.baud_rate = IE_UART_BAUD;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity    = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 122; 
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(IE_UART_NUM, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(IE_UART_NUM, &uart_config));
    
    ESP_ERROR_CHECK(uart_set_pin(IE_UART_NUM,
                                 IE_UART_TX_PIN, IE_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "Command response UART initialized.");
}

// ── New style I2S Slave Receiver Initializer
static void i2s_slave_init(void)
{
    ESP_LOGI(TAG, "Initializing Standard Mode I2S Slave Receiver...");

    // Allocate ring buffer: 16000 samples/sec * 2 bytes/sample * 1.5 seconds capacity
    s_audio_rb = xRingbufferCreate(16000 * 2 * 1.5, RINGBUF_TYPE_NOSPLIT);
    configASSERT(s_audio_rb);

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    i2s_std_config_t rx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,  // <--- Move mclk to the top
            .bclk = PIN_SLAVE_BCLK,
            .ws = PIN_SLAVE_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = PIN_DATA_IN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            }
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &rx_std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    ESP_LOGI(TAG, "I2S Slave Receiver Channel Enabled.");
}

// ── Background continuous DMA reader task running strictly on Core 0
static void i2s_dma_ingest_task(void *arg)
{
    uint8_t dma_read_buf[640]; // Stores 320 samples
    size_t bytes_read = 0;

    while (1) {
        // Automatically blocks on the physical hardware clock line until data frames exist
        if (i2s_channel_read(rx_chan, dma_read_buf, sizeof(dma_read_buf), &bytes_read, portMAX_DELAY) == ESP_OK) {
            if (bytes_read > 0) {
                xRingbufferSend(s_audio_rb, dma_read_buf, bytes_read, pdMS_TO_TICKS(10));
            }
        }
    }
}

// ── Main Inference processing worker task running strictly on Core 1
static void inference_task(void *arg)
{
    ESP_LOGI(TAG, "Inference task started on Core 1, state=WAKEWORD");

    signal_t ww_signal;
    ww_signal.total_length = WW_WINDOW_SAMPLES;
    ww_signal.get_data     = &ww_get_data;

    size_t rb_bytes_received = 0;

    while (1) {
        // === UNFREEZE IMPLEMENTATION ZONE ===
        // Pull available blocks from our synchronized hardware ring buffer
        int16_t *raw_rb_samples = (int16_t *)xRingbufferReceive(s_audio_rb, &rb_bytes_received, portMAX_DELAY);

        if (raw_rb_samples == NULL || rb_bytes_received == 0) {
            continue;
        }

        int samples_read = rb_bytes_received / sizeof(int16_t);

        // Only feed the classifiers real audio once the Core chip reports
        // an active SCO/HFP link. Otherwise drain the ring buffer (so it
        // doesn't back up / block the DMA task) but discard the data and
        // keep the state machine parked in WAKEWORD.
        if (gpio_get_level(PIN_BT_STATUS) == 0) {
            vRingbufferReturnItem(s_audio_rb, (void *)raw_rb_samples);
            g_state = STATE_WAKEWORD;
            g_ww_write_pos = 0;
            continue;
        }

        // Safely stage data into our tracking frame arrays
        for (int chunk_idx = 0; chunk_idx < samples_read; chunk_idx += CHUNK_SAMPLES) {
            int rem = samples_read - chunk_idx;
            int current_chunk_size = (rem < CHUNK_SAMPLES) ? rem : CHUNK_SAMPLES;
            
            memcpy(g_uart_rx_buf, &raw_rb_samples[chunk_idx], current_chunk_size * sizeof(int16_t));

            switch (g_state) {

            case STATE_WAKEWORD: {
                for (int i = 0; i < current_chunk_size; i++) {
                    g_ww_ring[g_ww_write_pos % WW_WINDOW_SAMPLES] = g_uart_rx_buf[i];
                    g_ww_write_pos++;

                    // Fire exactly once per full slice (checked per-sample since
                    // CHUNK_SAMPLES=512 does not evenly divide WW_SLICE_SAMPLES=4000;
                    // a post-chunk check can step over the boundary and never fire).
                    if (g_ww_write_pos >= WW_WINDOW_SAMPLES &&
                        (g_ww_write_pos % WW_SLICE_SAMPLES) == 0) {

                        // === RAW FEATURE DIAGNOSTIC ===
                        // Dump the first 20 raw samples of the current window so you
                        // can compare magnitude/shape against the ww_features[] array
                        // that works in run_static_test(). Silence should look like
                        // small numbers (-10..10); real speech should show swings
                        // into the hundreds/thousands, same as the static array.
                        {
                            char dbg[220];
                            int dbg_off = 0;
                            for (int d = 0; d < 20; d++) {
                                uint32_t didx = (g_ww_write_pos + d) % WW_WINDOW_SAMPLES;
                                dbg_off += snprintf(dbg + dbg_off, sizeof(dbg) - dbg_off, "%d ", g_ww_ring[didx]);
                            }
                            ESP_LOGI(TAG, "WW raw[0:20] = %s", dbg);
                        }

                        ei_impulse_result_t ww_result = {};
                        const ei_impulse_t *ww_snapshot_ptr = &impulse_1036490_1;
                        EI_IMPULSE_ERROR err = run_classifier_continuous(&ww_signal, &ww_result, ww_snapshot_ptr, false);
                        if (err != EI_IMPULSE_OK) {
                            ESP_LOGE(TAG, "WW classifier error: %d", err);
                            continue;
                        }

                        for (uint32_t k = 0; k < impulse_1036490_1.label_count; k++) {
                            ESP_LOGI(TAG, "WW [%s] = %.3f", impulse_1036490_1.categories[k], ww_result.classification[k].value);
                        }

                        for (uint32_t l = 0; l < impulse_1036490_1.label_count; l++) {
                            if (strcmp(ww_result.classification[l].label, "hey_mio") == 0) {
                                float conf = ww_result.classification[l].value;
                                if (conf > WW_CONFIDENCE_THRESHOLD) {
                                    ESP_LOGI(TAG, "Wakeword detected (%.3f) → CAPTURE", conf);
                                    g_cmd_ring_pos       = 0;
                                    g_cmd_total_written  = 0;
                                    g_cmd_last_check_pos = 0;
                                    g_vad_triggered      = false;
                                    g_pending_label[0]   = '\0';
                                    g_pending_count      = 0;
                                    memset(g_cmd_capture, 0, CMD_MODEL_WINDOW_SAMPLES * sizeof(int16_t));
                                    g_state = STATE_CAPTURE;
                                }
                                break;
                            }
                        }
                        if (g_state == STATE_CAPTURE) break;
                    }
                }
                break;
            }

            case STATE_CAPTURE: {
                // Write into the ring buffer sample-by-sample (wraps at
                // CMD_MODEL_WINDOW_SAMPLES); g_cmd_total_written tracks total
                // elapsed samples since capture began, independent of ring size,
                // used for the 5s timeout and the ~0.25s check throttle.
                uint32_t chunk_energy_sum = 0;
                for (int i = 0; i < current_chunk_size; i++) {
                    int16_t s = g_uart_rx_buf[i];
                    chunk_energy_sum += (s < 0) ? (uint32_t)(-s) : (uint32_t)s;

                    g_cmd_capture[g_cmd_ring_pos] = s;
                    g_cmd_ring_pos = (g_cmd_ring_pos + 1) % CMD_MODEL_WINDOW_SAMPLES;
                    g_cmd_total_written++;
                }

                // VAD: only start checking once real speech energy appears, instead
                // of guessing a fixed delay - much more robust to how long the user
                // actually pauses after the wake word.
                if (!g_vad_triggered && current_chunk_size > 0) {
                    uint32_t avg_energy = chunk_energy_sum / (uint32_t)current_chunk_size;
                    if (avg_energy > VAD_ENERGY_THRESHOLD) {
                        g_vad_triggered = true;
                        ESP_LOGI(TAG, "VAD: speech onset detected (energy=%u)", (unsigned)avg_energy);
                    }
                }

                bool have_full_window = (g_cmd_total_written >= CMD_MODEL_WINDOW_SAMPLES);
                bool due_for_check    = (g_cmd_total_written - g_cmd_last_check_pos) >= CMD_CHECK_INTERVAL_SAMPLES;
                bool window_time_out  = (g_cmd_total_written >= CMD_CAPTURE_SAMPLES);

                if (have_full_window && g_vad_triggered && (due_for_check || window_time_out)) {
                    g_cmd_last_check_pos = g_cmd_total_written;

                    ei_impulse_result_t cmd_result = {};
                    signal_t cmd_signal_local;
                    cmd_signal_local.total_length = CMD_MODEL_WINDOW_SAMPLES;
                    cmd_signal_local.get_data     = &cmd_get_data;

                    EI_IMPULSE_ERROR err = run_classifier(&impulse_handle_1037438_3, &cmd_signal_local, &cmd_result, false);

                    if (err == EI_IMPULSE_OK) {
                        float best_conf   = 0.0f;
                        float second_conf = 0.0f;
                        int   best_idx    = -1;

                        for (uint32_t i = 0; i < impulse_1037438_3.label_count; i++) {
                            float v = cmd_result.classification[i].value;
                            ESP_LOGI(TAG, "CMD [%s] = %.3f", impulse_1037438_3.categories[i], v);
                            if (v > best_conf) {
                                second_conf = best_conf;
                                best_conf   = v;
                                best_idx    = i;
                            } else if (v > second_conf) {
                                second_conf = v;
                            }
                        }

                        bool margin_ok = (best_conf - second_conf) >= CMD_CONFIDENCE_MARGIN;

                        if (best_idx >= 0 && best_conf > CMD_CONFIDENCE_THRESHOLD && margin_ok) {
                            const char *label = cmd_result.classification[best_idx].label;

                            uint8_t cmd_byte = 0x00;
                            if      (strcmp(label, "kansei") == 0) cmd_byte = 0x01;
                            else if (strcmp(label, "kiroku") == 0) cmd_byte = 0x02;
                            else if (strcmp(label, "ibasho") == 0) cmd_byte = 0x03;

                            if (cmd_byte != 0x00) {
                                // Track consecutive agreeing checks before acting - filters
                                // out one-off misfires from a single noisy window.
                                if (strcmp(g_pending_label, label) == 0) {
                                    g_pending_count++;
                                } else {
                                    strncpy(g_pending_label, label, sizeof(g_pending_label) - 1);
                                    g_pending_label[sizeof(g_pending_label) - 1] = '\0';
                                    g_pending_count = 1;
                                }

                                ESP_LOGI(TAG, "Tentative: %s (%.3f) x%d", label, best_conf, g_pending_count);

                                if (g_pending_count >= CMD_REQUIRED_CONSECUTIVE) {
                                    ESP_LOGI(TAG, "COMMAND DETECTED: %s (%.3f)", label, best_conf);
                                    uint8_t pkt[3] = { 0xAA, cmd_byte, static_cast<uint8_t>(0xAA ^ cmd_byte) };
                                    uart_write_bytes(IE_UART_NUM, (const char *)pkt, sizeof(pkt));
                                    ESP_LOGI(TAG, "Sent packet to core [AA %02X %02X]", cmd_byte, pkt[2]);

                                    g_state = STATE_WAKEWORD;
                                    break;
                                }
                            } else {
                                // "unknown" crossed threshold - not a real command, reset any
                                // pending streak and keep listening within the 5s window.
                                ESP_LOGI(TAG, "High-confidence unknown (%.3f) - still listening", best_conf);
                                g_pending_label[0] = '\0';
                                g_pending_count    = 0;
                            }
                        } else {
                            // Low confidence or ambiguous (no clear margin) - don't let a
                            // weak/uncertain check count toward the pending streak.
                            g_pending_label[0] = '\0';
                            g_pending_count    = 0;
                        }
                    } else {
                        ESP_LOGE(TAG, "CMD classifier error: %d", err);
                    }
                }

                if (window_time_out && g_state != STATE_WAKEWORD) {
                    ESP_LOGI(TAG, "5s command window expired, no match → WAKEWORD");
                    g_state = STATE_WAKEWORD;
                }
                break;
            }
            }
        }

        // Return allocation token window frame back to system memory
        vRingbufferReturnItem(s_audio_rb, (void *)raw_rb_samples);
    } 
}

void inference_engine_init(void)
{
    gpio_config_t bt_status_cfg = {
        .pin_bit_mask = (1ULL << PIN_BT_STATUS),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&bt_status_cfg));

    g_ww_ring = (int16_t *)heap_caps_malloc(
        WW_WINDOW_SAMPLES * sizeof(int16_t),
        MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);

    // g_cmd_capture is a ring buffer sized to exactly one classification
    // window (32KB), not the full 5s listen duration (which would be 160KB -
    // too large for a single contiguous internal-DRAM allocation with no PSRAM).
    g_cmd_capture = (int16_t *)heap_caps_malloc(
        CMD_MODEL_WINDOW_SAMPLES * sizeof(int16_t),
        MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);

    g_uart_rx_buf = (int16_t *)heap_caps_malloc(
        CHUNK_SAMPLES * sizeof(int16_t),
        MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);

    configASSERT(g_ww_ring);
    configASSERT(g_cmd_capture);
    configASSERT(g_uart_rx_buf);

    memset(g_ww_ring,     0, WW_WINDOW_SAMPLES        * sizeof(int16_t));
    memset(g_cmd_capture, 0, CMD_MODEL_WINDOW_SAMPLES  * sizeof(int16_t));

    uart_init();       // Outbound signaling UART initialization
    i2s_slave_init();  // Inbound streaming I2S Audio receiver initialization

    // Spawn high-frequency hardware ingestion onto Core 0
    xTaskCreatePinnedToCore(
        i2s_dma_ingest_task,
        "i2s_dma_ingest",
        4096,
        NULL,
        10,
        NULL,
        0
    );

    // Spawn heavy mathematical ML compilation framework loops onto Core 1
    xTaskCreatePinnedToCore(
        inference_task,
        "inference",
        16384, 
        NULL,
        5,
        NULL,
        1
    );

    ESP_LOGI(TAG, "Inference engine configuration fully updated to I2S.");
}