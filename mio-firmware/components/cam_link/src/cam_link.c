#include "cam_link.h"
#include "sd_config.h"
#include "audio_feedback.h"
#include "job_dispatcher.h"  // Header added to access job_dispatcher_cancel_timeout()
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "CAM_LINK";

#define CAM_UART_NUM     UART_NUM_2
#define CAM_UART_BAUD    921600
#define CAM_UART_TX_PIN  32
#define CAM_UART_RX_PIN  33

#define MAGIC_1 0xAA
#define MAGIC_2 0x55
#define FRAME_HEADER_AUDIO 0xCC
#define FRAME_HEADER_EVT   0xBB
#define FRAME_HEADER_CMD   0xAA

static cam_link_event_cb_t s_on_event = NULL;

static uint8_t xor_checksum(const uint8_t *data, uint16_t len) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) sum ^= data[i];
    return sum;
}

void cam_link_set_event_callback(cam_link_event_cb_t cb) {
    s_on_event = cb;
}

void cam_link_send_trigger(core_command_t cmd, int16_t pitch_centideg) {
    if (cmd != CORE_CMD_KANSEI && cmd != CORE_CMD_KIROKU) return;

    uint8_t pitch_hi = (uint8_t)((pitch_centideg >> 8) & 0xFF);
    uint8_t pitch_lo = (uint8_t)(pitch_centideg & 0xFF);
    uint8_t fixed[3] = { (uint8_t)cmd, pitch_hi, pitch_lo };

    uint8_t ssid_len = (uint8_t) strnlen(g_sd_config.wifi_ssid, SD_CFG_WIFI_SSID_MAXLEN - 1);
    uint8_t pass_len = (uint8_t) strnlen(g_sd_config.wifi_password, SD_CFG_WIFI_PASS_MAXLEN - 1);

    uint8_t cksum_buf[3 + 1 + (SD_CFG_WIFI_SSID_MAXLEN - 1) + 1 + (SD_CFG_WIFI_PASS_MAXLEN - 1)];
    uint16_t idx = 0;
    memcpy(&cksum_buf[idx], fixed, sizeof(fixed)); idx += sizeof(fixed);
    cksum_buf[idx++] = ssid_len;
    if (ssid_len > 0) { memcpy(&cksum_buf[idx], g_sd_config.wifi_ssid, ssid_len); idx += ssid_len; }
    cksum_buf[idx++] = pass_len;
    if (pass_len > 0) { memcpy(&cksum_buf[idx], g_sd_config.wifi_password, pass_len); idx += pass_len; }

    uint8_t checksum = xor_checksum(cksum_buf, idx);
    uint8_t header = FRAME_HEADER_CMD;

    uart_write_bytes(CAM_UART_NUM, (const char *)&header, 1);
    uart_write_bytes(CAM_UART_NUM, (const char *)fixed, sizeof(fixed));
    uart_write_bytes(CAM_UART_NUM, (const char *)&ssid_len, 1);
    if (ssid_len > 0) uart_write_bytes(CAM_UART_NUM, g_sd_config.wifi_ssid, ssid_len);
    uart_write_bytes(CAM_UART_NUM, (const char *)&pass_len, 1);
    if (pass_len > 0) uart_write_bytes(CAM_UART_NUM, g_sd_config.wifi_password, pass_len);
    uart_write_bytes(CAM_UART_NUM, (const char *)&checksum, 1);

    uint8_t term[2] = { 0x0D, 0x0A };
    uart_write_bytes(CAM_UART_NUM, (const char *)term, sizeof(term));
}

// Fast byte-by-byte alignment sync in case UART drops bytes mid-stream
static bool sync_to_next_audio_frame(void) {
    uint8_t b;
    while (uart_read_bytes(CAM_UART_NUM, &b, 1, pdMS_TO_TICKS(10)) > 0) {
        if (b == MAGIC_1) {
            if (uart_read_bytes(CAM_UART_NUM, &b, 1, pdMS_TO_TICKS(10)) > 0 && b == MAGIC_2) {
                if (uart_read_bytes(CAM_UART_NUM, &b, 1, pdMS_TO_TICKS(10)) > 0 && b == FRAME_HEADER_AUDIO) {
                    return true; // Synchronized successfully
                }
            }
        }
    }
    return false;
}

static bool cam_link_handle_audio_chunk(void) {
    uint8_t frame_buf[512 + 16];

    // Read remaining header fields (6 bytes): [is_first(1), is_last(1), seq(2), chunk_len(2)]
    int n = uart_read_bytes(CAM_UART_NUM, frame_buf, 6, pdMS_TO_TICKS(50));
    if (n != 6) {
        ESP_LOGW(TAG, "Header read timeout, attempting resync...");
        sync_to_next_audio_frame();
        return false;
    }

    bool is_first      = frame_buf[0] != 0;
    bool is_last       = frame_buf[1] != 0;
    uint16_t seq       = (uint16_t)frame_buf[2] | ((uint16_t)frame_buf[3] << 8);
    uint16_t chunk_len = (uint16_t)frame_buf[4] | ((uint16_t)frame_buf[5] << 8);

    if (chunk_len == 0 || chunk_len > 512) {
        ESP_LOGW(TAG, "Invalid chunk length %u, resyncing...", chunk_len);
        sync_to_next_audio_frame();
        return false;
    }

    // Read payload + checksum(1) + terminators(2)
    size_t remaining_bytes = chunk_len + 1 + 2; 
    n = uart_read_bytes(CAM_UART_NUM, frame_buf, remaining_bytes, pdMS_TO_TICKS(100));
    if (n != remaining_bytes) {
        ESP_LOGW(TAG, "Payload read incomplete (%d/%u), resyncing...", n, (unsigned)remaining_bytes);
        sync_to_next_audio_frame();
        return false;
    }

    uint8_t *payload = frame_buf;

    if (is_first) {
        audio_feedback_direct_stream_start();
        
        // Cancels the job_dispatcher timer as soon as the first byte streams in
        job_dispatcher_cancel_timeout();
    }

    // Push raw PCM directly to Bluetooth ringbuffer
    audio_feedback_push_direct_pcm(payload, chunk_len);

    if (is_last) {
        audio_feedback_direct_stream_end();
        ESP_LOGI(TAG, "Direct audio stream completed cleanly (%u chunks)", seq + 1);
    }

    return true;
}

static void cam_link_rx_task(void *arg) {
    ESP_LOGI(TAG, "CAM link RX task started");
    while (1) {
        uint8_t sync[3];
        // Scan for 3-byte prefix [0xAA, 0x55, 0xCC]
        int n = uart_read_bytes(CAM_UART_NUM, &sync[0], 1, portMAX_DELAY);
        if (n != 1) continue;

        if (sync[0] == MAGIC_1) {
            n = uart_read_bytes(CAM_UART_NUM, &sync[1], 1, pdMS_TO_TICKS(10));
            if (n == 1 && sync[1] == MAGIC_2) {
                n = uart_read_bytes(CAM_UART_NUM, &sync[2], 1, pdMS_TO_TICKS(10));
                if (n == 1 && sync[2] == FRAME_HEADER_AUDIO) {
                    cam_link_handle_audio_chunk();
                    continue;
                }
            }
        }

        if (sync[0] == FRAME_HEADER_EVT) {
            uint8_t body[4];
            n = uart_read_bytes(CAM_UART_NUM, body, sizeof(body), pdMS_TO_TICKS(50));
            if (n == sizeof(body) && xor_checksum(&body[0], 1) == body[1]) {
                if (s_on_event) s_on_event((cam_link_event_t)body[0]);
            }
        }
    }
}

void cam_link_init(void) {
    uart_config_t uart_config = {
        .baud_rate  = CAM_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 16KB Hardware RX Buffer is sufficient
    ESP_ERROR_CHECK(uart_driver_install(CAM_UART_NUM, 16384, 2048, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CAM_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CAM_UART_NUM, CAM_UART_TX_PIN, CAM_UART_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Priority 5 (Normal) prevents starving FreeRTOS Core 0 and Bluetooth driver
    xTaskCreatePinnedToCore(cam_link_rx_task, "cam_link_rx", 4096, NULL, 5, NULL, 0);
}