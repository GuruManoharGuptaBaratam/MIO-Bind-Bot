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

    // FIX 1: Change body array size from 4 to 3 [cmd, pitch_hi, pitch_lo]
    uint8_t body[3]; 
    len = uart_read_bytes(UART_PORT_NUM, body, sizeof(body), pdMS_TO_TICKS(50));
    if (len != sizeof(body)) {
        ESP_LOGW(TAG, "Incomplete trigger frame body (%d/%d bytes)", len, (int)sizeof(body));
        return -1;
    }

    // FIX 2: Explicitly read the 1-byte checksum that follows the body
    uint8_t checksum;
    len = uart_read_bytes(UART_PORT_NUM, &checksum, 1, pdMS_TO_TICKS(20));
    if (len != 1) {
        ESP_LOGW(TAG, "Failed to read checksum byte");
        return -1;
    }

    // FIX 3: Calculate the checksum over the 3 body bytes only
    if (uart_protocol_checksum(body, sizeof(body)) != checksum) {
        ESP_LOGW(TAG, "Checksum mismatch on trigger frame, dropping");
        return -1;
    }

    // FIX 4: Consume the 2 termination bytes (\r\n) cleanly from the buffer
    uint8_t term[2];
    uart_read_bytes(UART_PORT_NUM, term, sizeof(term), pdMS_TO_TICKS(20));

    // Map extracted elements to the output packet structure
    out_packet->header = FRAME_HEADER_CMD;
    out_packet->cmd = body[0];
    out_packet->pitch_centideg = (int16_t)((body[1] << 8) | body[2]);
    out_packet->checksum = checksum;

    ESP_LOGI(TAG, "Trigger received: cmd=0x%02X pitch=%.2f deg", out_packet->cmd, out_packet->pitch_centideg / 100.0f);

    // Visual confirmation via status LED
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