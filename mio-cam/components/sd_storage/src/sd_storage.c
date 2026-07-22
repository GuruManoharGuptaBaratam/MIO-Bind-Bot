#include "sd_storage.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/gpio.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <dirent.h>

static const char *TAG = "sd_storage";
#define MOUNT_POINT "/sdcard"
#define BASE_VIDEO_DIR MOUNT_POINT "/video"

// Ring-Buffer Threshold: maintain at least 100 MB free space at all times
#define MIN_FREE_SPACE_MB 100

static sdmmc_card_t *s_card = NULL;
static bool s_mounted = false;

// ---- AVI (MJPEG) muxer state ------------------------------------------
typedef struct { uint32_t offset; uint32_t size; } frame_idx_t;

static FILE *s_video_file = NULL;
static frame_idx_t *s_index = NULL;
static size_t s_index_count = 0;
static size_t s_index_cap = 0;
static uint32_t s_movi_data_start = 0;   // file offset right after 'movi'
static uint32_t s_width = 0, s_height = 0;
static uint8_t  s_fps = 0;

// Internal helper to create directories recursively if they don't exist
static void ensure_directory_exists(const char *path) {
    struct stat st = {0};
    if (stat(path, &st) == -1) {
        mkdir(path, 0755);
    }
}

// Check total free space available on mounted SD card
uint64_t sd_storage_get_free_bytes(void) {
    if (!s_mounted) return 0;
    FATFS *fs;
    DWORD fre_clust;
    if (f_getfree("0:", &fre_clust, &fs) == FR_OK) {
        uint64_t free_bytes = (uint64_t)fre_clust * fs->csize * 512;
        return free_bytes;
    }
    return 0;
}

// Find and delete the oldest .avi file in the video directory tree
// Find and delete the oldest .avi file in the video directory tree
static bool delete_oldest_video_file(const char *base_dir) {
    DIR *dir = opendir(base_dir);
    if (!dir) return false;

    struct dirent *entry;
    char oldest_file[512] = {0};
    time_t oldest_time = 0;
    bool found = false;

    // Scan subdirectories (e.g. /sdcard/video/2026_07 or /sdcard/video/rec)
    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_DIR) {
            if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) continue;

            char subpath[512];
            int ret1 = snprintf(subpath, sizeof(subpath), "%s/%s", base_dir, entry->d_name);
            if (ret1 < 0 || ret1 >= sizeof(subpath)) continue; // Skip truncated paths

            DIR *subdir = opendir(subpath);
            if (!subdir) continue;

            struct dirent *subentry;
            while ((subentry = readdir(subdir)) != NULL) {
                if (strstr(subentry->d_name, ".avi")) {
                    char filepath[512];
                    int ret2 = snprintf(filepath, sizeof(filepath), "%s/%s", subpath, subentry->d_name);
                    if (ret2 < 0 || ret2 >= sizeof(filepath)) continue; // Skip truncated paths

                    struct stat st;
                    if (stat(filepath, &st) == 0) {
                        if (!found || st.st_mtime < oldest_time) {
                            oldest_time = st.st_mtime;
                            strncpy(oldest_file, filepath, sizeof(oldest_file) - 1);
                            found = true;
                        }
                    }
                }
            }
            closedir(subdir);
        }
    }
    closedir(dir);

    if (found) {
        ESP_LOGW(TAG, "ring-buffer: storage low, deleting oldest file: %s", oldest_file);
        if (remove(oldest_file) == 0) {
            return true;
        } else {
            ESP_LOGE(TAG, "failed to delete %s", oldest_file);
        }
    }

    return false;
}
// Enforce ring-buffer capacity policy
static void check_and_enforce_ring_buffer(void) {
    uint64_t min_bytes = (uint64_t)MIN_FREE_SPACE_MB * 1024 * 1024;
    uint64_t free_bytes = sd_storage_get_free_bytes();

    while (free_bytes < min_bytes) {
        ESP_LOGW(TAG, "ring-buffer: free space (%llu MB) < target (%d MB)",
                 free_bytes / (1024 * 1024), MIN_FREE_SPACE_MB);
        
        if (!delete_oldest_video_file(BASE_VIDEO_DIR)) {
            ESP_LOGE(TAG, "ring-buffer: no more files to delete to reclaim space!");
            break;
        }
        free_bytes = sd_storage_get_free_bytes();
    }
}

esp_err_t sd_storage_init(void)
{
    if (s_mounted) return ESP_OK;

    // Explicitly set pull-up modes on 1-bit pins
    gpio_set_pull_mode(GPIO_NUM_2,  GPIO_PULLUP_ONLY);   // D0
    gpio_set_pull_mode(GPIO_NUM_15, GPIO_PULLUP_ONLY);   // CMD
    gpio_set_pull_mode(GPIO_NUM_14, GPIO_PULLUP_ONLY);   // CLK

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = SDMMC_FREQ_PROBING; // Force 400kHz probing speed first

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    // --- CRITICAL FIX FOR SERVO PIN CONFLICT ---
    slot_config.clk = GPIO_NUM_14;
    slot_config.cmd = GPIO_NUM_15;
    slot_config.d0  = GPIO_NUM_2;
    slot_config.d1  = GPIO_NUM_NC; // Disable Data 1 (Flash LED)
    slot_config.d2  = GPIO_NUM_NC; // Disable Data 2 (Servo Tilt on GPIO 12)
    slot_config.d3  = GPIO_NUM_NC; // Disable Data 3 (Servo Pan on GPIO 13)

    esp_err_t err = ESP_FAIL;
    for (int attempt = 1; attempt <= 3; attempt++) {
        err = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config,
                                       &mount_config, &s_card);
        if (err == ESP_OK) break;
        ESP_LOGW(TAG, "mount attempt %d/3 failed: %s -- retrying",
                  attempt, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed after retries: %s", esp_err_to_name(err));
        return err;
    }

    s_mounted = true;
    ESP_LOGI(TAG, "SD mounted at %s", MOUNT_POINT);

    // Initialize root video directories
    ensure_directory_exists(BASE_VIDEO_DIR);

    return ESP_OK;
}

// ---- minimal AVI/MJPEG header writers ----------------------------------
static void write_u32(FILE *f, uint32_t v) { fwrite(&v, 4, 1, f); }
static void write_u16(FILE *f, uint16_t v) { fwrite(&v, 2, 1, f); }
static void write_fourcc(FILE *f, const char *cc) { fwrite(cc, 4, 1, f); }

static void write_avi_header_placeholder(FILE *f, uint32_t w, uint32_t h, uint8_t fps)
{
    write_fourcc(f, "RIFF"); write_u32(f, 0); write_fourcc(f, "AVI ");
    write_fourcc(f, "LIST"); write_u32(f, 4 + 8 + 56 + 8 + 8 + 56 + 8 + 40);
    write_fourcc(f, "hdrl");

    write_fourcc(f, "avih"); write_u32(f, 56);
    write_u32(f, 1000000 / fps);
    write_u32(f, 0);
    write_u32(f, 0);
    write_u32(f, 0x10);
    write_u32(f, 0);
    write_u32(f, 0);
    write_u32(f, 1);
    write_u32(f, 0);
    write_u32(f, w); write_u32(f, h);
    write_u32(f, 0); write_u32(f, 0); write_u32(f, 0); write_u32(f, 0);

    write_fourcc(f, "LIST"); write_u32(f, 4 + 8 + 56 + 8 + 40);
    write_fourcc(f, "strl");

    write_fourcc(f, "strh"); write_u32(f, 56);
    write_fourcc(f, "vids"); write_fourcc(f, "MJPG");
    write_u32(f, 0);
    write_u16(f, 0); write_u16(f, 0);
    write_u32(f, 0);
    write_u32(f, 1);
    write_u32(f, fps);
    write_u32(f, 0);
    write_u32(f, 0);
    write_u32(f, 0);
    write_u32(f, 0xFFFFFFFF);
    write_u32(f, 0);
    write_u16(f, 0); write_u16(f, 0); write_u16(f, w); write_u16(f, h);

    write_fourcc(f, "strf"); write_u32(f, 40);
    write_u32(f, 40); write_u32(f, w); write_u32(f, h);
    write_u16(f, 1); write_u16(f, 24);
    write_fourcc(f, "MJPG");
    write_u32(f, w * h * 3);
    write_u32(f, 0); write_u32(f, 0); write_u32(f, 0); write_u32(f, 0);

    write_fourcc(f, "LIST"); write_u32(f, 0); write_fourcc(f, "movi");
}

esp_err_t sd_storage_open_video(uint32_t width, uint32_t height, uint8_t fps,
                                 char *out_path, size_t out_path_len)
{
    if (!s_mounted) return ESP_ERR_INVALID_STATE;
    if (s_video_file) {
        ESP_LOGW(TAG, "video already open -- closing previous one first");
        sd_storage_close_video();
    }

    // 1. Run capacity management check (reclaim space if needed)
    check_and_enforce_ring_buffer();

    // 2. Build structured date-time path
    time_t now = time(NULL);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    char folder_path[128];
    char file_path[256];

    // If year > 2020, SNTP or RTC time is valid
    if (timeinfo.tm_year > (2020 - 1900)) {
        snprintf(folder_path, sizeof(folder_path), "%s/%04d_%02d",
                 BASE_VIDEO_DIR, timeinfo.tm_year + 1900, timeinfo.tm_mon + 1);
        ensure_directory_exists(folder_path);

        snprintf(file_path, sizeof(file_path), "%s/kiroku_%04d%02d%02d_%02d%02d%02d.avi",
                 folder_path,
                 timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                 timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
        // Fallback structured directory if time is uncalibrated
        static int s_seq = 0;
        snprintf(folder_path, sizeof(folder_path), "%s/rec", BASE_VIDEO_DIR);
        ensure_directory_exists(folder_path);

        snprintf(file_path, sizeof(file_path), "%s/kiroku_%04d.avi", folder_path, s_seq++);
    }

    s_video_file = fopen(file_path, "wb");
    if (!s_video_file) {
        ESP_LOGE(TAG, "failed to open %s", file_path);
        return ESP_FAIL;
    }

    s_width = width; s_height = height; s_fps = fps;
    write_avi_header_placeholder(s_video_file, width, height, fps);
    s_movi_data_start = (uint32_t)ftell(s_video_file);

    s_index_count = 0;
    s_index_cap = 64;
    s_index = malloc(s_index_cap * sizeof(frame_idx_t));
    if (!s_index) {
        fclose(s_video_file);
        s_video_file = NULL;
        return ESP_ERR_NO_MEM;
    }

    if (out_path && out_path_len) {
        strncpy(out_path, file_path, out_path_len - 1);
        out_path[out_path_len - 1] = '\0';
    }

    ESP_LOGI(TAG, "opened %s (%lux%lu @%dfps)", file_path,
             (unsigned long)width, (unsigned long)height, fps);
    return ESP_OK;
}

esp_err_t sd_storage_write_frame(const uint8_t *jpeg_data, size_t jpeg_len)
{
    if (!s_video_file) return ESP_ERR_INVALID_STATE;

    if (s_index_count == s_index_cap) {
        s_index_cap *= 2;
        frame_idx_t *grown = realloc(s_index, s_index_cap * sizeof(frame_idx_t));
        if (!grown) return ESP_ERR_NO_MEM;
        s_index = grown;
    }

    uint32_t chunk_offset = (uint32_t)ftell(s_video_file) - s_movi_data_start;

    write_fourcc(s_video_file, "00dc");
    write_u32(s_video_file, (uint32_t)jpeg_len);
    fwrite(jpeg_data, 1, jpeg_len, s_video_file);
    if (jpeg_len & 1) { uint8_t pad = 0; fwrite(&pad, 1, 1, s_video_file); }

    s_index[s_index_count].offset = chunk_offset;
    s_index[s_index_count].size = (uint32_t)jpeg_len;
    s_index_count++;

    return ESP_OK;
}

esp_err_t sd_storage_close_video(void)
{
    if (!s_video_file) return ESP_ERR_INVALID_STATE;

    uint32_t movi_end = (uint32_t)ftell(s_video_file);

    write_fourcc(s_video_file, "idx1");
    write_u32(s_video_file, s_index_count * 16);
    for (size_t i = 0; i < s_index_count; i++) {
        write_fourcc(s_video_file, "00dc");
        write_u32(s_video_file, 0x10);
        write_u32(s_video_file, s_index[i].offset);
        write_u32(s_video_file, s_index[i].size);
    }

    uint32_t file_end = (uint32_t)ftell(s_video_file);

    fseek(s_video_file, 4, SEEK_SET);
    write_u32(s_video_file, file_end - 8);

    fseek(s_video_file, 48, SEEK_SET);
    write_u32(s_video_file, (uint32_t)s_index_count);

    fseek(s_video_file, 140, SEEK_SET);
    write_u32(s_video_file, (uint32_t)s_index_count);

    fseek(s_video_file, s_movi_data_start - 8, SEEK_SET);
    write_u32(s_video_file, movi_end - (s_movi_data_start - 4));

    fclose(s_video_file);
    s_video_file = NULL;
    free(s_index);
    s_index = NULL;

    ESP_LOGI(TAG, "closed video, %u frames written", (unsigned)s_index_count);
    return ESP_OK;
}