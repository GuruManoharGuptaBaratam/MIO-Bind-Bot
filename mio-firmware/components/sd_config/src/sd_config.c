#include "sd_config.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_rom_crc.h"  // Built-in ROM CRC32 (no mbedtls required!)

static const char *TAG = "sd_config";

#define SD_PIN_MISO  34
#define SD_PIN_CS    21

#define SD_MOUNT_POINT      "/sdcard"
#define SD_CONFIG_FILE      SD_MOUNT_POINT "/config.txt"
#define SD_SPI_HOST         SPI2_HOST

#define SD_CFG_LINE_MAXLEN  160

#define NVS_NAMESPACE       "sd_cfg_cache"
#define NVS_KEY_CONFIG      "runtime_cfg"
#define NVS_KEY_CRC         "cfg_crc32"

sd_runtime_config_t g_sd_config = {0};

// ---------------------------------------------------------------------
// NVS Caching Helpers (Using ROM CRC32)
// ---------------------------------------------------------------------

static esp_err_t compute_file_crc32(const char *filepath, uint32_t *out_crc)
{
    FILE *f = fopen(filepath, "rb");
    if (!f) return ESP_ERR_NOT_FOUND;

    uint32_t crc = 0;
    unsigned char buf[256];
    size_t bytes_read = 0;

    while ((bytes_read = fread(buf, 1, sizeof(buf), f)) > 0) {
        crc = esp_rom_crc32_le(crc, buf, bytes_read);
    }

    fclose(f);
    *out_crc = crc;
    return ESP_OK;
}

static bool read_nvs_cache(uint32_t current_crc)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    uint32_t cached_crc = 0;
    if (nvs_get_u32(handle, NVS_KEY_CRC, &cached_crc) != ESP_OK) {
        nvs_close(handle);
        return false;
    }

    // Check if configuration file changed
    if (current_crc != cached_crc) {
        ESP_LOGI(TAG, "Config file CRC32 changed (0x%08" PRIX32 " != 0x%08" PRIX32 ") -- re-parsing SD card",
                 current_crc, cached_crc);
        nvs_close(handle);
        return false;
    }

    // Hash matches -- load runtime config directly from NVS
    size_t cfg_len = sizeof(sd_runtime_config_t);
    sd_runtime_config_t temp_cfg;
    if (nvs_get_blob(handle, NVS_KEY_CONFIG, &temp_cfg, &cfg_len) != ESP_OK) {
        nvs_close(handle);
        return false;
    }

    nvs_close(handle);
    memcpy(&g_sd_config, &temp_cfg, sizeof(sd_runtime_config_t));
    ESP_LOGI(TAG, "NVS Cache Hit! Configuration loaded instantly from Flash NVS");
    return true;
}

static void save_nvs_cache(uint32_t current_crc)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_u32(handle, NVS_KEY_CRC, current_crc);
        nvs_set_blob(handle, NVS_KEY_CONFIG, &g_sd_config, sizeof(sd_runtime_config_t));
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "NVS Cache Updated with new SD configuration");
    }
}

esp_err_t sd_config_clear_nvs_cache(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_erase_all(handle);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "NVS Cache cleared");
        return ESP_OK;
    }
    return ESP_FAIL;
}

// ---------------------------------------------------------------------
// Parsing & Utility Functions
// ---------------------------------------------------------------------

static bool parse_mac(const char *str, uint8_t out[6])
{
    unsigned int b[6];
    if (sscanf(str, "%x:%x:%x:%x:%x:%x",
               &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) {
        return false;
    }
    for (int i = 0; i < 6; i++) {
        if (b[i] > 0xFF) return false;
        out[i] = (uint8_t)b[i];
    }
    return true;
}

static void trim_newline(char *s)
{
    size_t len = strlen(s);
    while (len > 0 && (s[len - 1] == '\n' || s[len - 1] == '\r')) {
        s[--len] = '\0';
    }
}

static bool split_kv(char *line, char **key, char **value)
{
    char *eq = strchr(line, '=');
    if (!eq) return false;
    *eq = '\0';
    *key = line;
    *value = eq + 1;
    return true;
}

static void apply_kv(const char *key, const char *value)
{
    if (strcmp(key, "BT_MAC") == 0) {
        if (parse_mac(value, g_sd_config.bt_mac)) {
            g_sd_config.bt_mac_valid = true;
        } else {
            ESP_LOGE(TAG, "BT_MAC malformed: '%s'", value);
        }
        return;
    }

    if (strcmp(key, "WIFI_SSID") == 0) {
        strncpy(g_sd_config.wifi_ssid, value, SD_CFG_WIFI_SSID_MAXLEN - 1);
        g_sd_config.wifi_ssid[SD_CFG_WIFI_SSID_MAXLEN - 1] = '\0';
        return;
    }

    if (strcmp(key, "WIFI_PASS") == 0) {
        strncpy(g_sd_config.wifi_password, value, SD_CFG_WIFI_PASS_MAXLEN - 1);
        g_sd_config.wifi_password[SD_CFG_WIFI_PASS_MAXLEN - 1] = '\0';
        return;
    }

    if (strncmp(key, "SOS", 3) == 0 && strlen(key) >= 5) {
        int idx = key[3] - '1';
        if (idx < 0 || idx >= SD_CFG_MAX_CONTACTS) {
            ESP_LOGW(TAG, "Unknown contact key: %s", key);
            return;
        }
        const char *field = key + 4;
        if (strcmp(field, "_NAME") == 0) {
            strncpy(g_sd_config.contacts[idx].name, value, SD_CFG_NAME_MAXLEN - 1);
            g_sd_config.contacts[idx].name[SD_CFG_NAME_MAXLEN - 1] = '\0';
            g_sd_config.contacts[idx].valid = true;
        } else if (strcmp(field, "_PHONE") == 0) {
            strncpy(g_sd_config.contacts[idx].phone, value, SD_CFG_PHONE_MAXLEN - 1);
            g_sd_config.contacts[idx].phone[SD_CFG_PHONE_MAXLEN - 1] = '\0';
            g_sd_config.contacts[idx].valid = true;
        } else {
            ESP_LOGW(TAG, "Unknown contact field: %s", key);
        }
        return;
    }

    ESP_LOGW(TAG, "Unrecognized config key: %s", key);
}

static esp_err_t parse_config_file(void)
{
    FILE *f = fopen(SD_CONFIG_FILE, "r");
    if (!f) {
        ESP_LOGE(TAG, "config.txt not found at %s", SD_CONFIG_FILE);
        return ESP_ERR_NOT_FOUND;
    }

    char line[SD_CFG_LINE_MAXLEN];
    while (fgets(line, sizeof(line), f)) {
        trim_newline(line);
        if (line[0] == '\0' || line[0] == '#') continue;

        char *key, *value;
        if (!split_kv(line, &key, &value)) {
            ESP_LOGW(TAG, "Skipping malformed line: '%s'", line);
            continue;
        }
        apply_kv(key, value);
    }

    fclose(f);
    return ESP_OK;
}

// ---------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------

esp_err_t sd_config_load(void)
{
    memset(&g_sd_config, 0, sizeof(g_sd_config));

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SD_SPI_HOST;
    host.max_freq_khz = 1000;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_PIN_CS;
    slot_config.host_id = SD_SPI_HOST;

    sdmmc_card_t *card;
    esp_err_t ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SD mount failed (%s) -- checking NVS cache fallback...", esp_err_to_name(ret));
        
        nvs_handle_t handle;
        if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
            size_t cfg_len = sizeof(sd_runtime_config_t);
            if (nvs_get_blob(handle, NVS_KEY_CONFIG, &g_sd_config, &cfg_len) == ESP_OK) {
                nvs_close(handle);
                ESP_LOGI(TAG, "Loaded fallback configuration from Flash NVS");
                return ESP_OK;
            }
            nvs_close(handle);
        }
        return ret;
    }

    ESP_LOGI(TAG, "SD mounted OK");

    uint32_t current_crc = 0;
    if (compute_file_crc32(SD_CONFIG_FILE, &current_crc) == ESP_OK) {
        if (read_nvs_cache(current_crc)) {
            g_sd_config.loaded = true;

            esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);
            ESP_LOGI(TAG, "SD unmounted (used NVS cache)");
            return ESP_OK;
        }
    }

    ESP_LOGI(TAG, "Cache Miss -- parsing config directly from SD card");
    ret = parse_config_file();
    if (ret == ESP_OK) {
        g_sd_config.loaded = true;
        save_nvs_cache(current_crc);
    }

    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);
    ESP_LOGI(TAG, "SD unmounted (boot-time read complete)");

    if (!g_sd_config.bt_mac_valid) {
        ESP_LOGE(TAG, "BT_MAC missing or invalid -- required field not set");
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}