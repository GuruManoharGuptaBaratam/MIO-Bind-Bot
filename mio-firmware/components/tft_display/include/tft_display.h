#pragma once
#include <stdbool.h>
#include "core_uart_receiver.h"
#include "sd_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// Starts the ST7735 panel and shows the initial "BT SEARCHING" screen.
// Call once from app_main(), after nvs/BT init order doesn't matter.
void tft_display_init(void);

// Call directly from hfp_manager.c at the SCO connected/disconnected
// transitions. This gates everything else: while BT is not connected, the
// screen is forced to "BT SEARCHING" and inference-engine events/commands
// below are ignored — mirrors how the inference ESP32 itself already gates
// its whole state machine on the same BT_STATUS line.
void tft_display_on_bt_state(bool connected);

// Wire these two directly into core_uart_receiver_set_callbacks() in
// main.c — no adapting needed, signatures match core_command_cb_t /
// core_event_cb_t exactly.
void tft_display_on_ie_event(core_event_t evt);
void tft_display_on_ie_command(core_command_t cmd);
void tft_display_on_sd_status(sd_boot_status_t status);


#ifdef __cplusplus
}
#endif