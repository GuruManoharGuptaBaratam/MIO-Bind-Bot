#include "uart_protocol.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "uart_protocol";

#define PIN_UART_TX     1    
#define PIN_UART_RX     3    
#define UART_PORT_NUM   UART_NUM_0
#define UART_BAUD_RATE  921600

#define DEBUG_LED_PIN   GPIO_NUM_33


uint8_t uart_protocol_checksum(const uint8_t *data, uint16_t len) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) sum ^= data[i];
    return sum;
}

void uart_protocol_init(void) {
    uart_config_t cfg = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, PIN_UART_TX, PIN_UART_RX,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, 4096, 4096, 0, NULL, 0));

    gpio_reset_pin(DEBUG_LED_PIN);
    gpio_set_direction(DEBUG_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DEBUG_LED_PIN, 1);
}

int uart_protocol_read_trigger(cam_trigger_packet_t *out_packet) {
    uint8_t header;
    int len = uart_read_bytes(UART_PORT_NUM, &header, 1, pdMS_TO_TICKS(20));
    if (len <= 0 || header != FRAME_HEADER_CMD) return -1;

    uint8_t fixed[3];
    len = uart_read_bytes(UART_PORT_NUM, fixed, sizeof(fixed), pdMS_TO_TICKS(50));
    if (len != sizeof(fixed)) return -1;

    uint8_t cksum_buf[3 + 1 + UART_MAX_SSID_LEN + 1 + UART_MAX_PASSWORD_LEN];
    uint16_t cksum_idx = 0;
    memcpy(&cksum_buf[cksum_idx], fixed, sizeof(fixed)); cksum_idx += sizeof(fixed);

    uint8_t ssid_len;
    len = uart_read_bytes(UART_PORT_NUM, &ssid_len, 1, pdMS_TO_TICKS(20));
    if (len != 1 || ssid_len > UART_MAX_SSID_LEN) return -1;
    cksum_buf[cksum_idx++] = ssid_len;

    char ssid[UART_MAX_SSID_LEN + 1] = {0};
    if (ssid_len > 0) {
        len = uart_read_bytes(UART_PORT_NUM, (uint8_t *)ssid, ssid_len, pdMS_TO_TICKS(50));
        if (len != ssid_len) return -1;
        memcpy(&cksum_buf[cksum_idx], ssid, ssid_len); cksum_idx += ssid_len;
    }

    uint8_t pass_len;
    len = uart_read_bytes(UART_PORT_NUM, &pass_len, 1, pdMS_TO_TICKS(20));
    if (len != 1 || pass_len > UART_MAX_PASSWORD_LEN) return -1;
    cksum_buf[cksum_idx++] = pass_len;

    char password[UART_MAX_PASSWORD_LEN + 1] = {0};
    if (pass_len > 0) {
        len = uart_read_bytes(UART_PORT_NUM, (uint8_t *)password, pass_len, pdMS_TO_TICKS(50));
        if (len != pass_len) return -1;
        memcpy(&cksum_buf[cksum_idx], password, pass_len); cksum_idx += pass_len;
    }

    uint8_t checksum;
    len = uart_read_bytes(UART_PORT_NUM, &checksum, 1, pdMS_TO_TICKS(20));
    if (len != 1 || uart_protocol_checksum(cksum_buf, cksum_idx) != checksum) return -1;

    uint8_t term[2];
    uart_read_bytes(UART_PORT_NUM, term, sizeof(term), pdMS_TO_TICKS(20));

    out_packet->header = FRAME_HEADER_CMD;
    out_packet->cmd = fixed[0];
    out_packet->pitch_centideg = (int16_t)((fixed[1] << 8) | fixed[2]);
    out_packet->checksum = checksum;
    memcpy(out_packet->ssid, ssid, sizeof(ssid));
    memcpy(out_packet->password, password, sizeof(password));
    out_packet->has_wifi_creds = (ssid_len > 0);

    return 0;
}

// DIRECT AUDIO TRANSMISSION (Pass-through stream without NACKs)
#define MAGIC_1 0xAA
#define MAGIC_2 0x55
#define UART_AUDIO_CHUNK_SIZE 320 // 10ms of 16kHz 16-bit Mono PCM

void uart_protocol_send_audio_stream(const uint8_t *audio_data, size_t audio_len) {
    if (!audio_data || audio_len == 0) return;

    size_t offset = 0;
    uint16_t seq = 0;

    while (offset < audio_len) {
        uint16_t chunk_len = (audio_len - offset > UART_AUDIO_CHUNK_SIZE)
                                  ? UART_AUDIO_CHUNK_SIZE
                                  : (uint16_t)(audio_len - offset);

        bool is_first = (offset == 0);
        bool is_last  = (offset + chunk_len) >= audio_len;

        uint8_t header[9] = {
            MAGIC_1,
            MAGIC_2,
            FRAME_HEADER_AUDIO,
            is_first ? 1 : 0,
            is_last ? 1 : 0,
            (uint8_t)(seq & 0xFF),
            (uint8_t)((seq >> 8) & 0xFF),
            (uint8_t)(chunk_len & 0xFF),
            (uint8_t)((chunk_len >> 8) & 0xFF)
        };

        uint8_t checksum = uart_protocol_checksum(&audio_data[offset], chunk_len);
        uint8_t term[2] = { FRAME_TERM_1, FRAME_TERM_2 };

        uart_write_bytes(UART_PORT_NUM, (const char *)header, sizeof(header));
        uart_write_bytes(UART_PORT_NUM, (const char *)&audio_data[offset], chunk_len);
        uart_write_bytes(UART_PORT_NUM, (const char *)&checksum, 1);
        uart_write_bytes(UART_PORT_NUM, (const char *)term, sizeof(term));

        offset += chunk_len;
        seq++;

        // Rate pacing: 10ms delay per 320 bytes exactly matches Bluetooth SCO consumption rate
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Audio stream sent directly: %u bytes in %u chunks", (unsigned)audio_len, seq);
}

void uart_protocol_send_event(cam_event_type_t event, const uint8_t *payload, uint16_t payload_len) {
    uint8_t frame[64];
    uint16_t idx = 0;
    frame[idx++] = FRAME_HEADER_EVENT;
    frame[idx++] = (uint8_t)event;

    if (payload && payload_len > 0 && payload_len <= (sizeof(frame) - 6)) {
        memcpy(&frame[idx], payload, payload_len);
        idx += payload_len;
    }

    uint8_t checksum = uart_protocol_checksum(&frame[1], idx - 1);
    frame[idx++] = checksum;
    frame[idx++] = FRAME_TERM_1;
    frame[idx++] = FRAME_TERM_2;

    uart_write_bytes(UART_PORT_NUM, (const char *)frame, idx);
}