#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Mirrors the command byte values sent by inference_engine.cpp's dispatch
// packet: [0xAA, cmd_byte, checksum]
typedef enum {
    CORE_CMD_KANSEI = 0x01,
    CORE_CMD_KIROKU = 0x02,
    CORE_CMD_IBASHO = 0x03,
} core_command_t;

// Mirrors ie_event_t in inference_engine.cpp's state packet:
// [0xBB, event_id, checksum]
typedef enum {
    CORE_EVT_LISTENING_WAKEWORD   = 0x01,
    CORE_EVT_WAKEWORD_DETECTED    = 0x02,
    CORE_EVT_COUNTDOWN_3          = 0x03,
    CORE_EVT_COUNTDOWN_2          = 0x04,
    CORE_EVT_COUNTDOWN_1          = 0x05,
    CORE_EVT_LISTENING_COMMAND    = 0x06,
    CORE_EVT_COMMAND_UNRECOGNIZED = 0x07,
    CORE_EVT_COMMAND_TIMEOUT      = 0x08,
} core_event_t;

typedef void (*core_command_cb_t)(core_command_t cmd);
typedef void (*core_event_cb_t)(core_event_t evt);

// Starts the dedicated UART peripheral + RX task listening for framed
// packets from the inference ESP32. Call once from app_main(); independent
// of BT/HFP init order since it's a separate peripheral.
//
// Wiring required: inference GPIO17 (IE_UART_TX_PIN) -> core GPIO16.
void core_uart_receiver_init(void);

// Register what to do when a command / state-event packet arrives.
// Pass NULL for either to just log that one without acting on it — useful
// before the OLED/display logic exists yet. Safe to call before or after
// core_uart_receiver_init().
void core_uart_receiver_set_callbacks(core_command_cb_t on_command, core_event_cb_t on_event);

#ifdef __cplusplus
}
#endif
