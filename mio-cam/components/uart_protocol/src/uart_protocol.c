#include "uart_protocol.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "uart_protocol";

// UART link to Core ESP32. These are the CAM's native USB-serial pins
// (UART0) - once wired to Core, the normal `idf.py monitor` serial log
// will not show CAM output unless UART is physically disconnected during
// debugging, or logging is moved to WiFi/telnet later.
// Kept local to this component since it's the only file that needs them -
// main/pin_config.h now only holds the servo pins.
#define PIN_UART_TX     1    // GPIO1 -> Core RX
#define PIN_UART_RX     3    // GPIO3 -> Core TX
#define UART_PORT_NUM   UART_NUM_0
#define UART_BAUD_RATE  115200

// DEBUG ONLY - remove or #ifdef this out once the CAM link is confirmed
// working. AI-Thinker ESP32-CAM's onboard red status LED, active-low.
// Confirm this is GPIO33 on your specific board - some clones wire it to
// GPIO4 (the same pin as the flash LED) instead.
#define DEBUG_LED_PIN   GPIO_NUM_33

uint8_t uart_protocol_checksum(const uint8_t *data, uint16_t len) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum ^= data[i];
    }
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
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, 256, 256, 0, NULL, 0));
    ESP_LOGI(TAG, "UART link to Core ready on TX=%d RX=%d", PIN_UART_TX, PIN_UART_RX);

    gpio_reset_pin(DEBUG_LED_PIN);
    gpio_set_direction(DEBUG_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DEBUG_LED_PIN, 1); // active-low: 1 = off
}

int uart_protocol_read_trigger(cam_trigger_packet_t *out_packet) {
    uint8_t header;
    int len = uart_read_bytes(UART_PORT_NUM, &header, 1, pdMS_TO_TICKS(20));
    if (len <= 0 || header != FRAME_HEADER_CMD) {
        return -1;
    }

    uint8_t fixed[3]; // cmd, pitch_hi, pitch_lo
    len = uart_read_bytes(UART_PORT_NUM, fixed, sizeof(fixed), pdMS_TO_TICKS(50));
    if (len != sizeof(fixed)) {
        ESP_LOGW(TAG, "Incomplete trigger frame fixed body (%d/%d bytes)", len, (int)sizeof(fixed));
        return -1;
    }

    // Checksum now covers fixed body + both variable fields. Max size:
    // 3 (fixed) + 1 (ssid_len) + 32 (ssid) + 1 (pass_len) + 64 (password) = 101 bytes.
    uint8_t cksum_buf[3 + 1 + UART_MAX_SSID_LEN + 1 + UART_MAX_PASSWORD_LEN];
    uint16_t cksum_idx = 0;
    memcpy(&cksum_buf[cksum_idx], fixed, sizeof(fixed));
    cksum_idx += sizeof(fixed);

    uint8_t ssid_len;
    len = uart_read_bytes(UART_PORT_NUM, &ssid_len, 1, pdMS_TO_TICKS(20));
    if (len != 1 || ssid_len > UART_MAX_SSID_LEN) {
        ESP_LOGW(TAG, "Bad ssid_len byte (%d)", ssid_len);
        return -1;
    }
    cksum_buf[cksum_idx++] = ssid_len;

    char ssid[UART_MAX_SSID_LEN + 1] = {0};
    if (ssid_len > 0) {
        len = uart_read_bytes(UART_PORT_NUM, (uint8_t *)ssid, ssid_len, pdMS_TO_TICKS(50));
        if (len != ssid_len) {
            ESP_LOGW(TAG, "Incomplete ssid field (%d/%d bytes)", len, ssid_len);
            return -1;
        }
        memcpy(&cksum_buf[cksum_idx], ssid, ssid_len);
        cksum_idx += ssid_len;
    }

    uint8_t pass_len;
    len = uart_read_bytes(UART_PORT_NUM, &pass_len, 1, pdMS_TO_TICKS(20));
    if (len != 1 || pass_len > UART_MAX_PASSWORD_LEN) {
        ESP_LOGW(TAG, "Bad pass_len byte (%d)", pass_len);
        return -1;
    }
    cksum_buf[cksum_idx++] = pass_len;

    char password[UART_MAX_PASSWORD_LEN + 1] = {0};
    if (pass_len > 0) {
        len = uart_read_bytes(UART_PORT_NUM, (uint8_t *)password, pass_len, pdMS_TO_TICKS(50));
        if (len != pass_len) {
            ESP_LOGW(TAG, "Incomplete password field (%d/%d bytes)", len, pass_len);
            return -1;
        }
        memcpy(&cksum_buf[cksum_idx], password, pass_len);
        cksum_idx += pass_len;
    }

    uint8_t checksum;
    len = uart_read_bytes(UART_PORT_NUM, &checksum, 1, pdMS_TO_TICKS(20));
    if (len != 1) {
        ESP_LOGW(TAG, "Failed to read checksum byte");
        return -1;
    }

    if (uart_protocol_checksum(cksum_buf, cksum_idx) != checksum) {
        ESP_LOGW(TAG, "Checksum mismatch on trigger frame, dropping");
        return -1;
    }

    uint8_t term[2];
    uart_read_bytes(UART_PORT_NUM, term, sizeof(term), pdMS_TO_TICKS(20));

    out_packet->header = FRAME_HEADER_CMD;
    out_packet->cmd = fixed[0];
    out_packet->pitch_centideg = (int16_t)((fixed[1] << 8) | fixed[2]);
    out_packet->checksum = checksum;
    memcpy(out_packet->ssid, ssid, sizeof(ssid));
    memcpy(out_packet->password, password, sizeof(password));
    out_packet->has_wifi_creds = (ssid_len > 0);

    ESP_LOGI(TAG, "Trigger received: cmd=0x%02X pitch=%.2f deg wifi_creds=%s",
             out_packet->cmd, out_packet->pitch_centideg / 100.0f,
             out_packet->has_wifi_creds ? "yes" : "no");

    gpio_set_level(DEBUG_LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(80));
    gpio_set_level(DEBUG_LED_PIN, 1);

    return 0;
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