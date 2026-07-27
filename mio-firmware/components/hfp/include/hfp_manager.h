#pragma once

#include <stdint.h>
#include "esp_bt_defs.h"

typedef enum {
    HFP_CODEC_UNKNOWN = 0,
    HFP_CODEC_CVSD = 1,   // 8 kHz Narrowband
    HFP_CODEC_MSBC = 2    // 16 kHz Wideband
} hfp_codec_type_t;

void hfp_init(void);
void hfp_connect(esp_bd_addr_t remote_bda);

// Dynamic sample rate state accessors
uint32_t hfp_get_negotiated_sample_rate(void);
hfp_codec_type_t hfp_get_active_codec(void);