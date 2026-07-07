#include "core_uart_receiver.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CORE_UART_RX";

// Dedicated UART for the inference -> core link only, separate from
// UART_NUM_0 (used by the console/log output). Inference's IE_UART_TX_PIN
// (GPIO17) must be wired to CORE_IE_UART_RX_PIN below.
#define CORE_IE_UART_NUM      UART_NUM_1
#define CORE_IE_UART_BAUD     921600              // must match IE_UART_BAUD in inference_engine.h
#define CORE_IE_UART_RX_PIN   16                   // <-- wire inference GPIO17 here
#define CORE_IE_UART_TX_PIN   UART_PIN_NO_CHANGE   // unused: inference -> core only, for now

#define PKT_HEADER_CMD   0xAA
#define PKT_HEADER_EVT   0xBB

static core_command_cb_t s_on_command = NULL;
static core_event_cb_t   s_on_event   = NULL;

void core_uart_receiver_set_callbacks(core_command_cb_t on_command, core_event_cb_t on_event)
{
    s_on_command = on_command;
    s_on_event   = on_event;
}

static const char *event_name(uint8_t evt)
{
    switch (evt) {
        case CORE_EVT_LISTENING_WAKEWORD:   return "LISTENING_WAKEWORD";
        case CORE_EVT_WAKEWORD_DETECTED:    return "WAKEWORD_DETECTED";
        case CORE_EVT_COUNTDOWN_3:          return "COUNTDOWN_3";
        case CORE_EVT_COUNTDOWN_2:          return "COUNTDOWN_2";
        case CORE_EVT_COUNTDOWN_1:          return "COUNTDOWN_1";
        case CORE_EVT_LISTENING_COMMAND:    return "LISTENING_COMMAND";
        case CORE_EVT_COMMAND_UNRECOGNIZED: return "COMMAND_UNRECOGNIZED";
        case CORE_EVT_COMMAND_TIMEOUT:      return "COMMAND_TIMEOUT";
        default:                            return "UNKNOWN";
    }
}

static const char *command_name(uint8_t cmd)
{
    switch (cmd) {
        case CORE_CMD_KANSEI: return "kansei";
        case CORE_CMD_KIROKU: return "kiroku";
        case CORE_CMD_IBASHO: return "ibasho";
        default:              return "unknown";
    }
}

// Reads one framed packet at a time: [header][payload][checksum].
// Resyncs byte-by-byte on an unrecognized header or a checksum failure
// instead of assuming alignment, so a single dropped/corrupted byte can't
// permanently desync the parser — it just drops that one frame and keeps
// scanning for the next valid header.
static void core_uart_rx_task(void *arg)
{
    ESP_LOGI(TAG, "UART RX task started, listening for inference-engine packets");

    uint8_t header = 0;

    while (1) {
        // 1. Find a valid header byte first.
        int n = uart_read_bytes(CORE_IE_UART_NUM, &header, 1, portMAX_DELAY);
        if (n != 1) {
            continue;
        }
        if (header != PKT_HEADER_CMD && header != PKT_HEADER_EVT) {
            // Not aligned to a frame start — drop and keep scanning.
            continue;
        }

        // 2. Read payload + checksum for this frame. Short timeout: if the
        // rest of the frame doesn't show up promptly, treat the header
        // byte as noise rather than blocking indefinitely.
        uint8_t rest[2];
        n = uart_read_bytes(CORE_IE_UART_NUM, rest, 2, pdMS_TO_TICKS(50));
        if (n != 2) {
            ESP_LOGW(TAG, "Incomplete frame (header 0x%02X) — resyncing", header);
            continue;
        }

        uint8_t payload  = rest[0];
        uint8_t checksum = rest[1];
        uint8_t expected = header ^ payload;

        if (checksum != expected) {
            ESP_LOGW(TAG, "Checksum mismatch (header 0x%02X payload 0x%02X) — dropping frame", header, payload);
            continue;
        }

        // 3. Valid frame — dispatch.
        if (header == PKT_HEADER_CMD) {
            ESP_LOGI(TAG, "Command received: %s (0x%02X)", command_name(payload), payload);
            if (s_on_command) {
                s_on_command((core_command_t)payload);
            }
        } else { // PKT_HEADER_EVT
            ESP_LOGI(TAG, "State event received: %s (0x%02X)", event_name(payload), payload);
            if (s_on_event) {
                s_on_event((core_event_t)payload);
            }
        }
    }
}

void core_uart_receiver_init(void)
{
    uart_config_t uart_config = {
        .baud_rate  = CORE_IE_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(CORE_IE_UART_NUM, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CORE_IE_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CORE_IE_UART_NUM,
                                  CORE_IE_UART_TX_PIN, CORE_IE_UART_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(core_uart_rx_task, "core_uart_rx", 3072, NULL, 10, NULL);

    ESP_LOGI(TAG, "Inference-link UART initialized (RX on GPIO%d, %d baud)",
             CORE_IE_UART_RX_PIN, CORE_IE_UART_BAUD);
}
