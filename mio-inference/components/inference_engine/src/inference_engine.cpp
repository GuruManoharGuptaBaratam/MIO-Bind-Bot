#include "inference_engine.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"  
#include "driver/uart.h"
#include "driver/i2s_std.h"   
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
extern ei_impulse_handle_t impulse_handle_1036490_1;
extern ei_impulse_handle_t impulse_handle_1037438_3;

static const char *TAG = "INF_ENGINE";

// ── Buffers ───────────────────────────────────────────────────────────────────
static int16_t *g_ww_ring     = NULL;   // rolling window for wakeword inference
static int16_t *g_cmd_ring    = NULL;   // Fixed 1.5-second capture buffer for single-shot command
static int16_t *g_uart_rx_buf = NULL;   // staging buffer for processing I2S data

static i2s_chan_handle_t rx_chan   = NULL; // I2S Rx Handle
static RingbufHandle_t s_audio_rb  = NULL; // FreeRTOS Ringbuffer

#define CHUNK_SAMPLES      512
#define AUDIO_SAMPLE_RATE  16000

// Configuration variables
#define GAP_DELAY_SAMPLES    (2 * AUDIO_SAMPLE_RATE)  // 2 seconds gap
#define CMD_CAPTURE_SAMPLES  24000                    // Exactly 1.5 seconds at 16kHz

// Pin layout configuration matching the transmitter chip
#define PIN_SLAVE_BCLK     GPIO_NUM_4   
#define PIN_SLAVE_WS       GPIO_NUM_5   
#define PIN_DATA_IN        GPIO_NUM_19  
#define PIN_BT_STATUS      GPIO_NUM_21

// ── State machine 
typedef enum {
    STATE_WAKEWORD,
    STATE_DELAY_GAP,
    STATE_COUNTDOWN,
    STATE_COMMAND_CAPTURE,
} ie_state_t;

static ie_state_t g_state               = STATE_WAKEWORD;
static uint32_t   g_ww_write_pos        = 0;
static uint32_t   g_cmd_write_pos       = 0;
static uint32_t   g_cmd_delay_counter   = 0;
static uint32_t   g_countdown_counter   = 0;

// ── Busy-State Lockout Tracking ──────────────────────────────────────────────
static uint32_t s_busy_lockout_until_ms = 0;

static void set_inference_busy_lockout(uint32_t lockout_duration_ms) {
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    s_busy_lockout_until_ms = now + lockout_duration_ms;
    ESP_LOGI(TAG, "Inference Gated: Muting wakeword classifier for next %" PRIu32 " ms", lockout_duration_ms);
}

static bool is_inference_gated(void) {
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    return (now < s_busy_lockout_until_ms);
}

// ─────────────────────────────────────────────────────────────────────────
// WAKEWORD MODEL — continuous streaming speed-change
#define WW_STRETCH_RATIO        1.5f
#define WW_STRETCH_STEP         (1.0f / WW_STRETCH_RATIO)   // ~0.6667 input-samples per output-sample
#define WW_STRETCH_SCRATCH_LEN  800                        
static float   s_ww_stretch_pos     = 0.0f;
static int16_t s_ww_stretch_prev    = 0;
static bool    s_ww_stretch_primed  = false;

static uint32_t ww_stretch_audio_stream(const uint8_t *in_buf, uint32_t in_len,
                                         uint8_t *out_buf, uint32_t out_buf_cap_bytes)
{
    const int16_t *in_samples = (const int16_t *)in_buf;
    uint32_t in_count = in_len / sizeof(int16_t);
    int16_t *out_samples = (int16_t *)out_buf;
    uint32_t out_cap = out_buf_cap_bytes / sizeof(int16_t);
    uint32_t out_count = 0;

    for (uint32_t i = 0; i < in_count; i++) {
        int16_t cur = in_samples[i];

        if (!s_ww_stretch_primed) {
            s_ww_stretch_prev = cur;
            s_ww_stretch_primed = true;
            continue;
        }

        while (s_ww_stretch_pos < 1.0f) {
            if (out_count >= out_cap) {
                return out_count * sizeof(int16_t);
            }
            float frac = s_ww_stretch_pos;
            float value = (float)s_ww_stretch_prev + ((float)cur - (float)s_ww_stretch_prev) * frac;
            out_samples[out_count++] = (int16_t)value;
            s_ww_stretch_pos += WW_STRETCH_STEP;
        }
        s_ww_stretch_pos -= 1.0f;
        s_ww_stretch_prev = cur;
    }

    return out_count * sizeof(int16_t);
}

// ─────────────────────────────────────────────────────────────────────────
// COMMAND MODEL — one-shot 1.0s capture -> 1.5s stretch
#define CMD_RAW_CAPTURE_SAMPLES   16000   // 1.0 second raw capture at 16kHz
#define CMD_FINAL_OUTPUT_SAMPLES  24000   // stretched to 1.5 seconds (== CMD_CAPTURE_SAMPLES)

static int16_t *g_cmd_raw_capture = NULL; // raw 1.0s capture buffer (pre-stretch)

static void cmd_stretch_capture_to_output(const int16_t *raw_input, int16_t *stretched_out)
{
    float step = (float)(CMD_RAW_CAPTURE_SAMPLES - 1) / (float)(CMD_FINAL_OUTPUT_SAMPLES - 1);

    for (int i = 0; i < CMD_FINAL_OUTPUT_SAMPLES; i++) {
        float pos = i * step;
        uint32_t idx = (uint32_t)pos;
        float frac = pos - (float)idx;

        int16_t s0 = raw_input[idx];
        int16_t s1 = (idx + 1 < CMD_RAW_CAPTURE_SAMPLES) ? raw_input[idx + 1] : raw_input[idx];
        stretched_out[i] = (int16_t)(s0 + (s1 - s0) * frac);
    }
}

// ── Inference → Core state signaling ────────────────────────────────────────
#define IE_EVT_HEADER   0xBB

typedef enum {
    IE_EVT_LISTENING_WAKEWORD   = 0x01, // idle, listening for "hey mio"
    IE_EVT_WAKEWORD_DETECTED    = 0x02,
    IE_EVT_COUNTDOWN_3          = 0x03,
    IE_EVT_COUNTDOWN_2          = 0x04,
    IE_EVT_COUNTDOWN_1          = 0x05,
    IE_EVT_LISTENING_COMMAND    = 0x06, // capturing the 1.0s command clip
    IE_EVT_COMMAND_UNRECOGNIZED = 0x07, // capture done, no confident label
    IE_EVT_COMMAND_TIMEOUT      = 0x08, // capture mostly silent / classifier error
} ie_event_t;

static void ie_send_event(ie_event_t evt)
{
    uint8_t pkt[3] = { IE_EVT_HEADER, (uint8_t)evt, (uint8_t)(IE_EVT_HEADER ^ (uint8_t)evt) };
    uart_write_bytes(IE_UART_NUM, (const char *)pkt, sizeof(pkt));
}

// ── EI signal callbacks 
static int ww_get_data(unsigned int offset, unsigned int length, float *out)
{
    for (size_t i = 0; i < length; i++) {
        uint32_t idx = (g_ww_write_pos + offset + i) % WW_WINDOW_SAMPLES;
        out[i] = (float)g_ww_ring[idx];   
    }
    return 0;
}

static int cmd_get_data(unsigned int offset, unsigned int length, float *out)
{
    for (size_t i = 0; i < length; i++) {
        out[i] = (float)g_cmd_ring[offset + i] ;
    }
    return 0;
}

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

static void i2s_slave_init(void)
{
    ESP_LOGI(TAG, "Initializing Standard Mode I2S Slave Receiver...");

    s_audio_rb = xRingbufferCreate(16000 * 2 * 1.5, RINGBUF_TYPE_NOSPLIT);
    configASSERT(s_audio_rb);

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    i2s_std_config_t rx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,  
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

static void i2s_dma_ingest_task(void *arg)
{
    uint8_t dma_read_buf[640]; 
    size_t bytes_read = 0;

    while (1) {
        if (i2s_channel_read(rx_chan, dma_read_buf, sizeof(dma_read_buf), &bytes_read, portMAX_DELAY) == ESP_OK) {
            if (bytes_read > 0) {
                xRingbufferSend(s_audio_rb, dma_read_buf, bytes_read, pdMS_TO_TICKS(10));
            }
        }
    }
}

static void inference_task(void *arg)
{
    ESP_LOGI(TAG, "Inference task started on Core 1, state=WAKEWORD");
    ie_send_event(IE_EVT_LISTENING_WAKEWORD);

    signal_t ww_signal;
    ww_signal.total_length = WW_WINDOW_SAMPLES;
    ww_signal.get_data     = &ww_get_data;

    signal_t cmd_signal;
    cmd_signal.total_length = CMD_CAPTURE_SAMPLES;   
    cmd_signal.get_data     = &cmd_get_data;

    size_t rb_bytes_received = 0;
    uint32_t last_countdown_second = 0;

    while (1) {
        int16_t *raw_rb_samples = (int16_t *)xRingbufferReceive(s_audio_rb, &rb_bytes_received, portMAX_DELAY);

        if (raw_rb_samples == NULL || rb_bytes_received == 0) {
            continue;
        }

        int samples_read = rb_bytes_received / sizeof(int16_t);

        if (gpio_get_level(PIN_BT_STATUS) == 0) {
            vRingbufferReturnItem(s_audio_rb, (void *)raw_rb_samples);
            if (g_state != STATE_WAKEWORD) {
                ie_send_event(IE_EVT_LISTENING_WAKEWORD);
            }
            g_state = STATE_WAKEWORD;
            g_ww_write_pos = 0;
            g_cmd_write_pos = 0;
            continue;
        }

        for (int chunk_idx = 0; chunk_idx < samples_read; chunk_idx += CHUNK_SAMPLES) {
            int rem = samples_read - chunk_idx;
            int current_chunk_size = (rem < CHUNK_SAMPLES) ? rem : CHUNK_SAMPLES;
            
            memcpy(g_uart_rx_buf, &raw_rb_samples[chunk_idx], current_chunk_size * sizeof(int16_t));

            switch (g_state) {

            case STATE_WAKEWORD: {
                // Check if system is currently locked out executing a command (kansei/kiroku/ibasho)
                if (is_inference_gated()) {
                    memset(g_ww_ring, 0, WW_WINDOW_SAMPLES * sizeof(int16_t));
                    g_ww_write_pos = 0;
                    break; // Skip stretch processing & continuous classifier
                }

                uint8_t ww_stretched_buf[WW_STRETCH_SCRATCH_LEN * sizeof(int16_t)];
                uint32_t ww_stretched_len = ww_stretch_audio_stream(
                    (const uint8_t *)g_uart_rx_buf,
                    current_chunk_size * sizeof(int16_t),
                    ww_stretched_buf,
                    sizeof(ww_stretched_buf));
                int16_t *ww_stretched_samples = (int16_t *)ww_stretched_buf;
                int ww_stretched_count = ww_stretched_len / sizeof(int16_t);

                for (int i = 0; i < ww_stretched_count; i++) {
                    g_ww_ring[g_ww_write_pos % WW_WINDOW_SAMPLES] = ww_stretched_samples[i];
                    g_ww_write_pos++;

                    if (g_ww_write_pos >= WW_WINDOW_SAMPLES && (g_ww_write_pos % WW_SLICE_SAMPLES) == 0) {
                        ei_impulse_result_t ww_result = {};
                        
                        EI_IMPULSE_ERROR err = run_classifier_continuous(&impulse_handle_1036490_1, &ww_signal, &ww_result, false);
                        if (err != EI_IMPULSE_OK) {
                            ESP_LOGE(TAG, "WW classifier error: %d", err);
                            continue;
                        }

                        for (uint32_t l = 0; l < impulse_1036490_1.label_count; l++) {
                            if (strcmp(ww_result.classification[l].label, "hey_mio") == 0) {
                                float conf = ww_result.classification[l].value;
                                if (conf > WW_CONFIDENCE_THRESHOLD) {
                                    ESP_LOGI(TAG, "Wakeword detected (%.3f) → Entering Gap Delay State", conf);
                                    g_cmd_delay_counter = 0;
                                    g_state = STATE_DELAY_GAP;
                                    ie_send_event(IE_EVT_WAKEWORD_DETECTED);
                                }
                                break;
                            }
                        }
                        if (g_state == STATE_DELAY_GAP) break;
                    }
                }
                break;
            }

            case STATE_DELAY_GAP: {
                for (int i = 0; i < current_chunk_size; i++) {
                    g_cmd_delay_counter++;

                    if (g_cmd_delay_counter >= GAP_DELAY_SAMPLES) {
                        ESP_LOGI(TAG, "Gap Complete → Initiating Countdown Sequence...");
                        g_countdown_counter = 0;
                        last_countdown_second = 4;
                        g_state = STATE_COUNTDOWN;
                        break;
                    }
                }
                break;
            }

            case STATE_COUNTDOWN: {
                for (int i = 0; i < current_chunk_size; i++) {
                    g_countdown_counter++;
                    uint32_t current_second = 3 - (g_countdown_counter / AUDIO_SAMPLE_RATE);

                    if (current_second != last_countdown_second && current_second >= 1) {
                        ESP_LOGI(TAG, "SPEAK COMMAND IN... %" PRIu32, current_second);
                        last_countdown_second = current_second;
                        if (current_second == 3)      ie_send_event(IE_EVT_COUNTDOWN_3);
                        else if (current_second == 2) ie_send_event(IE_EVT_COUNTDOWN_2);
                        else if (current_second == 1) ie_send_event(IE_EVT_COUNTDOWN_1);
                    }

                    if (g_countdown_counter >= (3 * AUDIO_SAMPLE_RATE)) {
                        ESP_LOGI(TAG, "★★★ RECORDING STARTED - SPEAK NOW (1.0 Sec Raw Capture) ★★★");
                        memset(g_cmd_raw_capture, 0, CMD_RAW_CAPTURE_SAMPLES * sizeof(int16_t));
                        g_cmd_write_pos = 0;
                        g_state = STATE_COMMAND_CAPTURE;
                        ie_send_event(IE_EVT_LISTENING_COMMAND);
                        break;
                    }
                }
                break;
            }

            case STATE_COMMAND_CAPTURE: {
                for (int i = 0; i < current_chunk_size; i++) {
                    g_cmd_raw_capture[g_cmd_write_pos] = g_uart_rx_buf[i];
                    g_cmd_write_pos++;

                    if (g_cmd_write_pos >= CMD_RAW_CAPTURE_SAMPLES) {
                        ESP_LOGI(TAG, "Raw capture completed. Evaluating audio signal frame...");

                        uint32_t zero_count = 0;
                        for (uint32_t s = 0; s < CMD_RAW_CAPTURE_SAMPLES; s++) {
                            if (g_cmd_raw_capture[s] == 0) {
                                zero_count++;
                            }
                        }
                        float zero_ratio = (float)zero_count / CMD_RAW_CAPTURE_SAMPLES;
                        if (zero_ratio >= 0.75f) {
                            ESP_LOGW(TAG, "Captured buffer dropped: %.1f%% zeros detected. Returning to WAKEWORD.", zero_ratio * 100.0f);
                            ie_send_event(IE_EVT_COMMAND_TIMEOUT);
                            memset(g_ww_ring, 0, WW_WINDOW_SAMPLES * sizeof(int16_t));
                            g_ww_write_pos = 0;
                            g_state = STATE_WAKEWORD;
                            ie_send_event(IE_EVT_LISTENING_WAKEWORD);
                            break;
                        }

                        ESP_LOGI(TAG, "Processing 0.66x stretch to exactly %d features...", CMD_FINAL_OUTPUT_SAMPLES);
                        cmd_stretch_capture_to_output(g_cmd_raw_capture, g_cmd_ring);

                        ei_impulse_result_t cmd_result = {};
                        run_classifier_init(&impulse_handle_1037438_3);
                        EI_IMPULSE_ERROR err = run_classifier(&impulse_handle_1037438_3, &cmd_signal, &cmd_result, false);

                        if (err != EI_IMPULSE_OK) {
                            ESP_LOGE(TAG, "CMD Single Classifier execution failed: %d", err);
                            ie_send_event(IE_EVT_COMMAND_TIMEOUT);
                            memset(g_ww_ring, 0, WW_WINDOW_SAMPLES * sizeof(int16_t));
                            g_ww_write_pos = 0;
                            g_state = STATE_WAKEWORD;
                            ie_send_event(IE_EVT_LISTENING_WAKEWORD);
                            break;
                        }

                        char conf_buf[160];
                        int off = 0;
                        float best_conf = 0.0f;
                        int best_cmd_byte = 0x00;
                        const char *best_label = "none";

                        for (uint32_t c = 0; c < impulse_1037438_3.label_count; c++) {
                            const char *label = cmd_result.classification[c].label;
                            float conf_val = cmd_result.classification[c].value;

                            off += snprintf(conf_buf + off, sizeof(conf_buf) - off, "%s:%.3f ", label, conf_val);

                            if (strcmp(label, "noise") != 0 && strcmp(label, "unknown") != 0) {
                                if (conf_val > best_conf) {
                                    best_conf = conf_val;
                                    best_label = label;
                                    if      (strcmp(label, "kansei") == 0) best_cmd_byte = 0x01;
                                    else if (strcmp(label, "kiroku") == 0) best_cmd_byte = 0x02;
                                    else if (strcmp(label, "ibasho") == 0) best_cmd_byte = 0x03;
                                }
                            }
                        }
                        ESP_LOGI(TAG, "SINGLE-RUN RESULT CONFIDENCE: [%s]", conf_buf);

                        if (best_cmd_byte != 0x00) {
                            ESP_LOGI(TAG, "EXECUTION WINNER: %s Chosen (%.3f)", best_label, best_conf);
                            uint8_t pkt[3] = { 0xAA, (uint8_t)best_cmd_byte, static_cast<uint8_t>(0xAA ^ best_cmd_byte) };
                            uart_write_bytes(IE_UART_NUM, (const char *)pkt, sizeof(pkt));
                            ESP_LOGI(TAG, "Sent UART Packet [AA %02X %02X]", best_cmd_byte, pkt[2]);

                            // Apply command-specific lockout gate to silence wakeword detection
                            if (best_cmd_byte == 0x01) {       // KANSEI
                                set_inference_busy_lockout(30000);  // 30 seconds
                            } else if (best_cmd_byte == 0x02) { // KIROKU
                                set_inference_busy_lockout(160000); // 160 seconds (2.30 min + margin)
                            } else if (best_cmd_byte == 0x03) { // IBASHO
                                set_inference_busy_lockout(5000);   // 5 seconds
                            }

                        } else {
                            ESP_LOGW(TAG, "EVALUATION FAILURE: Frame was dominated entirely by background noise elements.");
                            ie_send_event(IE_EVT_COMMAND_UNRECOGNIZED);
                        }

                        memset(g_ww_ring, 0, WW_WINDOW_SAMPLES * sizeof(int16_t));
                        g_ww_write_pos = 0;
                        g_state = STATE_WAKEWORD;
                        ie_send_event(IE_EVT_LISTENING_WAKEWORD);
                        break;
                    }
                }
                break;
            }
            }
        }
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

    g_cmd_ring = (int16_t *)heap_caps_malloc(
        CMD_CAPTURE_SAMPLES * sizeof(int16_t),
        MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);

    g_cmd_raw_capture = (int16_t *)heap_caps_malloc(
        CMD_RAW_CAPTURE_SAMPLES * sizeof(int16_t),
        MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);

    g_uart_rx_buf = (int16_t *)heap_caps_malloc(
        CHUNK_SAMPLES * sizeof(int16_t),
        MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);

    configASSERT(g_ww_ring);
    configASSERT(g_cmd_ring);
    configASSERT(g_cmd_raw_capture);
    configASSERT(g_uart_rx_buf);

    memset(g_ww_ring,  0, WW_WINDOW_SAMPLES   * sizeof(int16_t));
    memset(g_cmd_ring, 0, CMD_CAPTURE_SAMPLES * sizeof(int16_t));
    memset(g_cmd_raw_capture, 0, CMD_RAW_CAPTURE_SAMPLES * sizeof(int16_t));

    uart_init();       
    i2s_slave_init();  

    xTaskCreatePinnedToCore(i2s_dma_ingest_task, "i2s_dma_ingest", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(inference_task, "inference", 16384, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "Inference engine configuration completely validated and initialized.");
}