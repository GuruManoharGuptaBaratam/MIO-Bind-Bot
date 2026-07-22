#ifndef SD_CONFIG_H
#define SD_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define SD_CFG_MAX_CONTACTS      3
#define SD_CFG_NAME_MAXLEN       32
#define SD_CFG_PHONE_MAXLEN      20
#define SD_CFG_WIFI_SSID_MAXLEN  33   
#define SD_CFG_WIFI_PASS_MAXLEN  65 

typedef enum {
    SD_BOOT_OK,
    SD_BOOT_NOT_FOUND,
    SD_BOOT_BAD_CONFIG,
} sd_boot_status_t;

typedef struct {
    char     name[SD_CFG_NAME_MAXLEN];
    char     phone[SD_CFG_PHONE_MAXLEN];
    bool     valid;
} sos_contact_t;

typedef struct {
    uint8_t        bt_mac[6];
    bool           bt_mac_valid;

    sos_contact_t  contacts[SD_CFG_MAX_CONTACTS];

    char           wifi_ssid[SD_CFG_WIFI_SSID_MAXLEN];
    char           wifi_password[SD_CFG_WIFI_PASS_MAXLEN];

    bool           loaded;
} sd_runtime_config_t;

extern sd_runtime_config_t g_sd_config;

// Checks NVS cache first using SHA-256 hash validation.
// Reads SD card only if config.txt changed or NVS cache is empty.
esp_err_t sd_config_load(void);

// Clears cached NVS configurations manually if needed
esp_err_t sd_config_clear_nvs_cache(void);

#endif // SD_CONFIG_H