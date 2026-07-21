#include "cam_link.h"
#include "sd_config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "CAM_LINK";

#define CAM_UART_NUM     UART_NUM_2
#define CAM_UART_BAUD    115200
#define CAM_UART_TX_PIN  32
#define CAM_UART_RX_PIN  33

#define FRAME_HEADER_CMD   0xAA
#define FRAME_HEADER_EVT   0xBB

static cam_link_event_cb_t s_on_event = NULL;

static uint8_t xor_checksum(const uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum ^= data[i];
    }
    return sum;
}

void cam_link_set_event_callback(cam_link_event_cb_t cb)
{
    s_on_event = cb;
}

void cam_link_send_trigger(core_command_t cmd, int16_t pitch_centideg)
{
    if (cmd != CORE_CMD_KANSEI && cmd != CORE_CMD_KIROKU) {
        ESP_LOGW(TAG, "cam_link_send_trigger called with non-CAM command 0x%02X, ignoring", cmd);
        return;
    }

    uint8_t pitch_hi = (uint8_t)((pitch_centideg >> 8) & 0xFF);
    uint8_t pitch_lo = (uint8_t)(pitch_centideg & 0xFF);
    uint8_t fixed[3] = { (uint8_t)cmd, pitch_hi, pitch_lo };

    // Pulls whatever sd_config_load() cached at boot. If config.txt had
    // no WIFI_SSID= line, wifi_ssid[0] is '\0' (zeroed by the memset in
    // sd_config_load()) -- ssid_len ends up 0, and CAM correctly falls
    // back to its own hardcoded wifi_credentials.h.
    uint8_t ssid_len = (uint8_t) strnlen(g_sd_config.wifi_ssid, SD_CFG_WIFI_SSID_MAXLEN - 1);
    uint8_t pass_len = (uint8_t) strnlen(g_sd_config.wifi_password, SD_CFG_WIFI_PASS_MAXLEN - 1);

    // Build the checksum buffer: fixed(3) + ssid_len(1) + ssid + pass_len(1) + password
    // Must match the CAM side's uart_protocol_read_trigger() checksum layout exactly.
    uint8_t cksum_buf[3 + 1 + (SD_CFG_WIFI_SSID_MAXLEN - 1) + 1 + (SD_CFG_WIFI_PASS_MAXLEN - 1)];
    uint16_t idx = 0;
    memcpy(&cksum_buf[idx], fixed, sizeof(fixed));
    idx += sizeof(fixed);
    cksum_buf[idx++] = ssid_len;
    if (ssid_len > 0) {
        memcpy(&cksum_buf[idx], g_sd_config.wifi_ssid, ssid_len);
        idx += ssid_len;
    }
    cksum_buf[idx++] = pass_len;
    if (pass_len > 0) {
        memcpy(&cksum_buf[idx], g_sd_config.wifi_password, pass_len);
        idx += pass_len;
    }

    uint8_t checksum = xor_checksum(cksum_buf, idx);

    // Write the frame in pieces -- simpler than building one giant
    // contiguous buffer, and uart_write_bytes() blocks until queued
    // anyway so partial writes aren't a correctness concern here.
    uint8_t header = FRAME_HEADER_CMD;
    uart_write_bytes(CAM_UART_NUM, (const char *)&header, 1);
    uart_write_bytes(CAM_UART_NUM, (const char *)fixed, sizeof(fixed));
    uart_write_bytes(CAM_UART_NUM, (const char *)&ssid_len, 1);
    if (ssid_len > 0) {
        uart_write_bytes(CAM_UART_NUM, g_sd_config.wifi_ssid, ssid_len);
    }
    uart_write_bytes(CAM_UART_NUM, (const char *)&pass_len, 1);
    if (pass_len > 0) {
        uart_write_bytes(CAM_UART_NUM, g_sd_config.wifi_password, pass_len);
    }
    uart_write_bytes(CAM_UART_NUM, (const char *)&checksum, 1);

    uint8_t term[2] = { 0x0D, 0x0A };
    uart_write_bytes(CAM_UART_NUM, (const char *)term, sizeof(term));

    ESP_LOGI(TAG, "Sent trigger cmd=0x%02X pitch=%.2f deg to CAM (wifi_creds=%s)",
             cmd, pitch_centideg / 100.0f, ssid_len > 0 ? "yes" : "no");
}
// NOTE: assumes a zero-payload event frame (event byte + checksum only),
// matching the CAM stub's current uart_protocol_send_event(evt, NULL, 0)
// calls. Once CAM starts sending real payloads (e.g. scene description
// text/audio chunks), this needs a length-prefixed or delimited framing
// instead of the fixed 2-byte body read below.
static void cam_link_rx_task(void *arg)
{
    ESP_LOGI(TAG, "CAM link RX task started");
    while (1) {
        uint8_t header;
        int n = uart_read_bytes(CAM_UART_NUM, &header, 1, portMAX_DELAY);
        if (n != 1 || header != FRAME_HEADER_EVT) {
            continue;
        }

        uint8_t body[2]; // event, checksum
        n = uart_read_bytes(CAM_UART_NUM, body, sizeof(body), pdMS_TO_TICKS(50));
        if (n != sizeof(body)) {
            ESP_LOGW(TAG, "Incomplete CAM event frame - resyncing");
            continue;
        }

        uint8_t evt = body[0];
        uint8_t checksum = body[1];
        if (xor_checksum(&evt, 1) != checksum) {
            ESP_LOGW(TAG, "CAM event checksum mismatch - dropping");
            continue;
        }

        uint8_t term[2];
        uart_read_bytes(CAM_UART_NUM, term, sizeof(term), pdMS_TO_TICKS(20)); // best-effort consume

        ESP_LOGI(TAG, "CAM event: 0x%02X", evt);
        if (s_on_event) {
            s_on_event((cam_link_event_t)evt);
        }
    }
}

void cam_link_init(void)
{
    uart_config_t uart_config = {
        .baud_rate  = CAM_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(CAM_UART_NUM, 2048, 2048, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CAM_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CAM_UART_NUM, CAM_UART_TX_PIN, CAM_UART_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Pinned to core 0, same reasoning as core_uart_receiver.c: keep every
    // task that can call into tft_display_* off core 1, away from the
    // priority-22 audio feeder in hfp_manager.c.
    xTaskCreatePinnedToCore(cam_link_rx_task, "cam_link_rx", 3072, NULL, 10, NULL, 0);

    ESP_LOGI(TAG, "CAM link UART initialized (TX=%d RX=%d, %d baud)",
             CAM_UART_TX_PIN, CAM_UART_RX_PIN, CAM_UART_BAUD);
}