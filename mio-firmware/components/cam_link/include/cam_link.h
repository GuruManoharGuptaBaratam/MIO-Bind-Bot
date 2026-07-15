#pragma once
#include <stdint.h>
#include "core_uart_receiver.h"   // reuses core_command_t (KANSEI/KIROKU/IBASHO)

#ifdef __cplusplus
extern "C" {
#endif

// Mirrors cam_event_t in the CAM-side uart_protocol.h. Keep these two enums
// in sync by hand until both projects share one header file.
typedef enum {
    CAM_LINK_EVT_JOB_STARTED = 0x01,
    CAM_LINK_EVT_JOB_DONE    = 0x02,
    CAM_LINK_EVT_JOB_FAILED  = 0x03,
} cam_link_event_t;

typedef void (*cam_link_event_cb_t)(cam_link_event_t evt);

// Wiring: Core G32 (TX) -> CAM GPIO3 (RX), Core G33 (RX) -> CAM GPIO1 (TX).
// Straight-through - each side's TX goes to the other side's RX.
void cam_link_init(void);

// Sends a trigger frame to CAM: [0xAA][cmd][pitch_hi][pitch_lo][checksum].
// cmd must be CORE_CMD_KANSEI or CORE_CMD_KIROKU - anything else is ignored
// and logged as a warning.
void cam_link_send_trigger(core_command_t cmd, int16_t pitch_centideg);

// Register what to do when CAM reports job started/done/failed.
// Pass NULL to just log it - useful before OLED state wiring exists.
void cam_link_set_event_callback(cam_link_event_cb_t cb);

#ifdef __cplusplus
}
#endif