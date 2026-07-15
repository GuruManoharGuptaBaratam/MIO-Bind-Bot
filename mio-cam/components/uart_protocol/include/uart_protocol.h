#pragma once
#include <stdint.h>

// DRAFT FRAMING - align with Core's inference_engine header once shared.
// Core -> CAM: [0xAA][cmd][pitch_hi][pitch_lo][checksum][0x0D][0x0A]
// CAM  -> Core: [0xBB][event][payload...][checksum][0x0D][0x0A]

#define FRAME_HEADER_CMD    0xAA   // Core -> CAM command frame
#define FRAME_HEADER_EVENT  0xBB   // CAM -> Core reply/event frame
#define FRAME_TERM_1        0x0D
#define FRAME_TERM_2        0x0A

typedef enum {
    CAM_CMD_KANSEI = 0x01,
    CAM_CMD_KIROKU = 0x02,
} cam_cmd_type_t;

typedef enum {
    CAM_EVENT_JOB_STARTED = 0x01,
    CAM_EVENT_JOB_DONE    = 0x02,
    CAM_EVENT_JOB_FAILED  = 0x03,
} cam_event_type_t;

typedef struct {
    uint8_t header;
    uint8_t cmd;
    int16_t pitch_centideg;   // live IMU pitch angle * 100, e.g. 1234 = 12.34 deg
    uint8_t checksum;
} cam_trigger_packet_t;

void uart_protocol_init(void);

// Blocking-ish, short-timeout read of one trigger frame.
// Returns 0 on success, -1 if no valid frame was available.
int uart_protocol_read_trigger(cam_trigger_packet_t *out_packet);

// Sends an event frame back to Core, with optional payload bytes.
void uart_protocol_send_event(cam_event_type_t event, const uint8_t *payload, uint16_t payload_len);

uint8_t uart_protocol_checksum(const uint8_t *data, uint16_t len);