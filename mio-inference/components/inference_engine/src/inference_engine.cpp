#include "inference_engine.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
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
extern const ei_impulse_t impulse_1037438_1;

static const char *TAG = "INF_ENGINE";

// ── Buffers ───────────────────────────────────────────────────────────────────
static int16_t *g_ww_ring     = NULL;   // rolling window for wakeword inference
static int16_t *g_cmd_capture = NULL;   // flat 2-second buffer for command inference
static int16_t *g_uart_rx_buf = NULL;   // staging buffer for UART reads

#define UART_RX_BUF_BYTES  4096
#define CHUNK_SAMPLES      512

// ── State machine 
typedef enum {
    STATE_WAKEWORD,
    STATE_CAPTURE,
    STATE_COMMAND,
} ie_state_t;

static ie_state_t g_state         = STATE_WAKEWORD;
static uint32_t   g_ww_write_pos  = 0;
static uint32_t   g_cmd_write_pos = 0;

// ── EI signal callbacks 
// Uses explicit 'unsigned int' properties to guarantee function pointer binding matches on GCC 15
static int ww_get_data(unsigned int offset, unsigned int length, float *out)
{
    for (size_t i = 0; i < length; i++) {
        uint32_t idx = (g_ww_write_pos + offset + i) % WW_WINDOW_SAMPLES;
        out[i] = (float)g_ww_ring[idx] / 32768.0f;
    }
    return 0;
}

static int cmd_get_data(unsigned int offset, unsigned int length, float *out)
{
    for (size_t i = 0; i < length; i++) {
        out[i] = (float)g_cmd_capture[offset + i] / 32768.0f;
    }
    return 0;
}

// ── UART init 
static void uart_init(void)
{
    // Explicit sequential initializers conforming with ESP-IDF v6.0 driver updates
    uart_config_t uart_config = {};
    uart_config.baud_rate = IE_UART_BAUD;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity    = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 122; // Property variable name matched to v6.0 API rules
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(IE_UART_NUM, UART_RX_BUF_BYTES, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(IE_UART_NUM, &uart_config));
    
    ESP_ERROR_CHECK(uart_set_pin(IE_UART_NUM,
                                 IE_UART_TX_PIN, IE_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG,
         "UART=%d  TX=%d  RX=%d",
         IE_UART_NUM,
         IE_UART_TX_PIN,
         IE_UART_RX_PIN);
    ESP_LOGI(TAG, "UART2 init OK @ %d baud", IE_UART_BAUD);
}

// ── Inference task
static void inference_task(void *arg)
{
    ESP_LOGI(TAG, "Inference task started, state=WAKEWORD");

    // Standard variable mapping setup avoiding unsafe macro designator reorder loops
    signal_t ww_signal;
    ww_signal.total_length = WW_WINDOW_SAMPLES;
    ww_signal.get_data     = &ww_get_data;

    signal_t cmd_signal;
    cmd_signal.total_length = CMD_CAPTURE_SAMPLES;
    cmd_signal.get_data     = &cmd_get_data;

    while (1) {
        // ── Read available UART bytes
        size_t avail = 0;
        uart_get_buffered_data_len(IE_UART_NUM, &avail);

        size_t bytes_to_read = avail;
        size_t max_bytes     = CHUNK_SAMPLES * sizeof(int16_t);
        if (bytes_to_read > max_bytes) bytes_to_read = max_bytes;

        int samples_read = 0;
        if (bytes_to_read > 0) {
            int got = uart_read_bytes(IE_UART_NUM,
                                      (uint8_t *)g_uart_rx_buf,
                                      bytes_to_read, 0);
            if (got > 0) samples_read = got / (int)sizeof(int16_t);
        }

        if (samples_read == 0) {
            static int silent_count = 0;
            if (++silent_count % 200 == 0) {
                ESP_LOGI(TAG, "No data yet...");
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        switch (g_state) {

        // ── State A: wakeword 
        case STATE_WAKEWORD: {
            
            for (int i = 0; i < samples_read; i++) {
                g_ww_ring[g_ww_write_pos % WW_WINDOW_SAMPLES] = g_uart_rx_buf[i];
                g_ww_write_pos++;
            }

            if (g_ww_write_pos < WW_WINDOW_SAMPLES) break;

            ei_impulse_result_t ww_result = {};
            
            // Pointer abstraction that satisfies compiler safety evaluations
            const ei_impulse_t *ww_snapshot_ptr = &impulse_1036490_1;
            EI_IMPULSE_ERROR err = run_classifier_continuous(&ww_signal, &ww_result, ww_snapshot_ptr, false);
            if (err != EI_IMPULSE_OK) {
                ESP_LOGE(TAG, "WW classifier error: %d", err);
                break;
            }
            for (uint32_t i = 0; i < impulse_1036490_1.label_count; i++) {
                ESP_LOGI(TAG,
                        "WW [%s] = %.3f",
                        impulse_1036490_1.categories[i],
                        ww_result.classification[i].value);
            }

            for (uint32_t l = 0; l < 2; l++) {
                if (strcmp(ww_result.classification[l].label, "hey_mio") == 0) {
                    float conf = ww_result.classification[l].value;
                    ESP_LOGI(TAG, "hey_mio conf=%.3f", conf);
                    if (conf > WW_CONFIDENCE_THRESHOLD) {
                        // gpio_set_level(LED_PIN, 1); 
                        ESP_LOGI(TAG, "Wakeword detected (%.3f) → CAPTURE", conf);
                        g_cmd_write_pos = 0;
                        memset(g_cmd_capture, 0,
                               CMD_CAPTURE_SAMPLES * sizeof(int16_t));
                        g_state = STATE_CAPTURE;
                    }
                    break;
                }
            }
            break;
        }

        // ── State B: capture 2 seconds 
        case STATE_CAPTURE: {
            uint32_t space = CMD_CAPTURE_SAMPLES - g_cmd_write_pos;
            uint32_t copy  = ((uint32_t)samples_read < space)
                             ? (uint32_t)samples_read : space;
            memcpy(&g_cmd_capture[g_cmd_write_pos],
                   g_uart_rx_buf,
                   copy * sizeof(int16_t));
            g_cmd_write_pos += copy;

            if (g_cmd_write_pos >= CMD_CAPTURE_SAMPLES) {
                ESP_LOGI(TAG, "Capture complete → COMMAND");
                g_state = STATE_COMMAND;
            }
            break;
        }

        // ── State C: command inference ────────────────────────────────────
        case STATE_COMMAND: {
            ei_impulse_result_t cmd_result = {};
            
            // Pointer abstraction that satisfies compiler safety evaluations
            const ei_impulse_t *cmd_snapshot_ptr = &impulse_1037438_1;
            EI_IMPULSE_ERROR err = run_classifier_continuous(&cmd_signal, &cmd_result, cmd_snapshot_ptr, false);
            
            if (err != EI_IMPULSE_OK) {
                ESP_LOGE(TAG, "CMD classifier error: %d", err);
                g_state = STATE_WAKEWORD;
                break;
            }

            float   best_conf  = 0.0f;
            int     best_idx   = -1;
            
            for (uint32_t i = 0; i < impulse_1037438_1.label_count; i++) {
                ESP_LOGI(TAG,
                        "CMD [%s] = %.3f",
                        impulse_1037438_1.categories[i],
                        cmd_result.classification[i].value);
            }

            if (best_idx >= 0 && best_conf > CMD_CONFIDENCE_THRESHOLD) {
                const char *label = cmd_result.classification[best_idx].label;
                ESP_LOGI(TAG, "COMMAND: %s (%.3f)", label, best_conf);

                uint8_t cmd_byte = 0x00;
                if      (strcmp(label, "kansei") == 0) cmd_byte = 0x01;
                else if (strcmp(label, "kiroku") == 0) cmd_byte = 0x02;
                else if (strcmp(label, "ibasho") == 0) cmd_byte = 0x03;

                if (cmd_byte != 0x00) {
                    uint8_t pkt[3] = { 0xAA, cmd_byte, static_cast<uint8_t>(0xAA ^ cmd_byte) };
                    uart_write_bytes(IE_UART_NUM, (const char *)pkt, sizeof(pkt));
                    ESP_LOGI(TAG, "Sent packet [AA %02X %02X]",
                             cmd_byte, pkt[2]);
                }
            } else {
                ESP_LOGI(TAG, "No command above threshold (best=%.3f)", best_conf);
            }

            g_state = STATE_WAKEWORD;
            break;
        }

        }
    } 
}

// ---- Model Testing code block ---- [ refer model_testing/combined_model_test.cpp ]
 


// -- Feature Callback calling --
void inference_engine_init(void)
{

    g_ww_ring = (int16_t *)heap_caps_malloc(
        WW_WINDOW_SAMPLES * sizeof(int16_t),
        MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);

    g_cmd_capture = (int16_t *)heap_caps_malloc(
        CMD_CAPTURE_SAMPLES * sizeof(int16_t),
        MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);

    g_uart_rx_buf = (int16_t *)heap_caps_malloc(
        CHUNK_SAMPLES * sizeof(int16_t),
        MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);

    configASSERT(g_ww_ring);
    configASSERT(g_cmd_capture);
    configASSERT(g_uart_rx_buf);

    memset(g_ww_ring,     0, WW_WINDOW_SAMPLES   * sizeof(int16_t));
    memset(g_cmd_capture, 0, CMD_CAPTURE_SAMPLES * sizeof(int16_t));

    // run_static_test(); Model Test Callback
    uart_init();

    xTaskCreatePinnedToCore(
        inference_task,
        "inference",
        16384, // Allocates 16KB stack allocation to secure processing deep multi-model structures
        NULL,
        5,
        NULL,
        1
    );

    ESP_LOGI(TAG, "Inference engine init complete");
}
