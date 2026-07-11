#include "sd_config.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"

static const char *TAG = "sd_config";

// ---- Pin map (Core ESP32, shared SPI2 bus with TFT) ----
// SCK/MOSI are now the SAME physical wires as the TFT (GPIO14 / GPIO13).
// Only CS (own line) and MISO (TFT doesn't use MISO at all) are dedicated
// to the SD card. This makes SD a second *device* on the TFT's existing
// bus instead of a second, separately-initialized SPI hardware bus.
#define SD_PIN_MISO  34
#define SD_PIN_CS    21

#define SD_MOUNT_POINT      "/sdcard"
#define SD_CONFIG_FILE      SD_MOUNT_POINT "/config.txt"
// TFT (st7735_gfx.cpp) already calls spi_bus_initialize(SPI2_HOST, ...)
// in tft_display_init(), which MUST run before sd_config_load(). SD now
// joins that same bus as a second device — no second bus, no init/free
// cycle, no more disturbing the TFT's SPI state.
#define SD_SPI_HOST         SPI2_HOST

#define SD_CFG_LINE_MAXLEN  160

sd_runtime_config_t g_sd_config = {0};

// ---------------------------------------------------------------------
// Helpers
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

// Splits "KEY=VALUE" in place. Returns false if no '=' found.
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

    if (strcmp(key, "USER_IMG") == 0) {
        strncpy(g_sd_config.user_img_path, value, SD_CFG_PATH_MAXLEN - 1);
        g_sd_config.user_img_path[SD_CFG_PATH_MAXLEN - 1] = '\0';
        return;
    }

    // SOS1_NAME / SOS1_PHONE / SOS2_NAME / ... / SOS3_PHONE
    if (strncmp(key, "SOS", 3) == 0 && strlen(key) >= 5) {
        int idx = key[3] - '1';   // SOS1 -> 0, SOS2 -> 1, SOS3 -> 2
        if (idx < 0 || idx >= SD_CFG_MAX_CONTACTS) {
            ESP_LOGW(TAG, "Unknown contact key: %s", key);
            return;
        }
        const char *field = key + 4; // skip "SOSn"
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
        if (line[0] == '\0' || line[0] == '#') continue; // blank line / comment

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

static esp_err_t load_user_image(void)
{
    if (g_sd_config.user_img_path[0] == '\0') {
        ESP_LOGI(TAG, "No USER_IMG field set, skipping face image load");
        return ESP_OK; // optional field
    }

    char full_path[SD_CFG_PATH_MAXLEN + sizeof(SD_MOUNT_POINT)];
    snprintf(full_path, sizeof(full_path), "%s%s", SD_MOUNT_POINT, g_sd_config.user_img_path);

    FILE *f = fopen(full_path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "User image not found at %s", full_path);
        return ESP_ERR_NOT_FOUND;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size <= 0) {
        ESP_LOGE(TAG, "User image is empty or unreadable");
        fclose(f);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t *buf = malloc((size_t)size);
    if (!buf) {
        ESP_LOGE(TAG, "malloc(%ld) failed for user image — check heap budget", size);
        fclose(f);
        return ESP_ERR_NO_MEM;
    }

    size_t read = fread(buf, 1, (size_t)size, f);
    fclose(f);

    if (read != (size_t)size) {
        ESP_LOGE(TAG, "Short read on user image (%d/%ld bytes)", (int)read, size);
        free(buf);
        return ESP_FAIL;
    }

    g_sd_config.user_img_buf = buf;
    g_sd_config.user_img_len = read;
    ESP_LOGI(TAG, "User image loaded: %d bytes", (int)read);
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
    // Bus is shared with the TFT now. SDMMC_FREQ_DEFAULT (~20MHz) assumes
    // clean, short PCB traces — breadboard jumper wires can't reliably
    // carry that; the card mounts but reads start failing (BAD CONFIG).
    // 1MHz is a safe, solid middle ground for jumper-wire prototyping.
    host.max_freq_khz = 4000;

    // NOTE: no spi_bus_initialize() here on purpose. tft_display_init()
    // already called spi_bus_initialize(SPI2_HOST, ...) before this runs.
    // SD is added below as a second *device* on that same bus — calling
    // spi_bus_initialize() again on an already-initialized host would just
    // fail with ESP_ERR_INVALID_STATE, so we skip straight to attaching
    // the device.

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_PIN_CS;
    slot_config.host_id = SD_SPI_HOST;

    sdmmc_card_t *card;
    esp_err_t ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem — is card formatted FAT32?");
        } else {
            ESP_LOGE(TAG, "SD mount failed: %s (check wiring, is card inserted?)",
                     esp_err_to_name(ret));
        }
        // Do NOT call spi_bus_free() here — this bus belongs to the TFT
        // (SPI2_HOST) and must stay alive for the display to keep working.
        return ret;
    }

    ESP_LOGI(TAG, "SD mounted OK");

    ret = parse_config_file();
    if (ret == ESP_OK) {
        (void)load_user_image(); // optional, failure here doesn't fail overall load
        g_sd_config.loaded = true;
    }

    // Insert-before-boot-only design: no further SD access needed after this,
    // so unmount the filesystem/card — but do NOT free the SPI bus itself,
    // since the TFT still owns and needs SPI2_HOST for the rest of runtime.
    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);
    ESP_LOGI(TAG, "SD unmounted (boot-time read complete)");

    if (!g_sd_config.bt_mac_valid) {
        ESP_LOGE(TAG, "BT_MAC missing or invalid — required field not set");
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

void sd_config_free(void)
{
    if (g_sd_config.user_img_buf) {
        free(g_sd_config.user_img_buf);
        g_sd_config.user_img_buf = NULL;
        g_sd_config.user_img_len = 0;
    }
}