#pragma once
#include <stdbool.h>
#include "core_uart_receiver.h"
#include "sd_config.h"
#include "audio_feedback.h" // <--- ADD THIS INCLUDE

#ifdef __cplusplus
extern "C" {
#endif

void tft_display_init(void);
void tft_display_on_bt_state(bool connected);
void tft_display_on_ie_event(core_event_t evt);
void tft_display_on_ie_command(core_command_t cmd);
void tft_display_on_sd_status(sd_boot_status_t status);

void tft_display_on_command_processing(sound_id_t working_sound);
void tft_display_on_command_done(sound_id_t done_sound);

void tft_display_on_button_confirm(core_command_t cmd);
void tft_display_on_button_cancelled(core_command_t cmd, bool was_timeout);
void tft_display_on_button_busy(void);
void tft_display_on_camera_failed(void);
// tft_display.h
void tft_display_on_job_timeout(void);

#ifdef __cplusplus
}
#endif