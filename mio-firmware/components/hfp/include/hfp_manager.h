#pragma once

#include "esp_bt_defs.h"

void hfp_init(void);
void hfp_connect(esp_bd_addr_t remote_bda);