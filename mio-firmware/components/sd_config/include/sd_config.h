#ifndef SD_CONFIG_H
#define SD_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define SD_CFG_MAX_CONTACTS      3
#define SD_CFG_NAME_MAXLEN       32
#define SD_CFG_PHONE_MAXLEN      20
#define SD_CFG_PATH_MAXLEN       64
#define SD_CFG_WIFI_SSID_MAXLEN  33   
#define SD_CFG_WIFI_PASS_MAXLEN  65 

// Simplified boot-time outcome for display purposes — collapses the
// various esp_err_t possibilities from sd_config_load() into the three
// states main.c needs to show on screen.
typedef enum {
    SD_BOOT_OK,
    SD_BOOT_NOT_FOUND,
    SD_BOOT_BAD_CONFIG,
} sd_boot_status_t;

typedef struct {
    char     name[SD_CFG_NAME_MAXLEN];
    char     phone[SD_CFG_PHONE_MAXLEN];
    bool     valid;   // true if this contact slot was present in config.txt
} sos_contact_t;

typedef struct {
    uint8_t        bt_mac[6];
    bool           bt_mac_valid;

    sos_contact_t  contacts[SD_CFG_MAX_CONTACTS];

    char           user_img_path[SD_CFG_PATH_MAXLEN];
    uint8_t       *user_img_buf;
    size_t         user_img_len;

    char           wifi_ssid[SD_CFG_WIFI_SSID_MAXLEN];      // NEW: from WIFI_SSID=
    char           wifi_password[SD_CFG_WIFI_PASS_MAXLEN];  // NEW: from WIFI_PASS=

    bool           loaded;
} sd_runtime_config_t;
// Global instance — read this after sd_config_load() returns.
extern sd_runtime_config_t g_sd_config;

// Mounts SD (custom SPI pins), reads /config.txt and /user.jpg (if referenced),
// fills g_sd_config, then unmounts SD. Call once at boot, before BT init.
// Returns ESP_OK even if some optional fields are missing; only fails if
// SD mount fails outright or BT_MAC is absent/malformed.
esp_err_t sd_config_load(void);

// Frees g_sd_config.user_img_buf if allocated. Call if you reload config
// or on graceful shutdown, to avoid a leak on repeated loads.
void sd_config_free(void);

#endif // SD_CONFIG_H