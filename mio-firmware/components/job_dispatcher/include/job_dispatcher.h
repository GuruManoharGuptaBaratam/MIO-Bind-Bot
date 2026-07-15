#pragma once
#include "core_uart_receiver.h"

#ifdef __cplusplus
extern "C" {
#endif

// Sets up the IMU and CAM link. Call once from app_main(), after
// tft_display_init() and before core_uart_receiver_init().
void job_dispatcher_init(void);

// Pass these two to core_uart_receiver_set_callbacks() in place of the raw
// tft_display_on_ie_command/tft_display_on_ie_event pair. Each one calls
// the original display callback first, then adds CAM-triggering behavior
// on top - display behavior is unchanged from before.
void job_dispatcher_on_command(core_command_t cmd);
void job_dispatcher_on_event(core_event_t evt);

#ifdef __cplusplus
}
#endif