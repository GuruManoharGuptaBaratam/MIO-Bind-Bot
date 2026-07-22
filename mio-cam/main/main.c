#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "uart_protocol.h"
#include "servo_control.h"
#include "wifi_client.h"
#include "wifi_credentials.h"
#include "camera_driver.h"
#include "sd_storage.h"

static const char *TAG = "mio_cam";

#define KANSEI_UPLOAD_URL "http://192.168.1.100:8000/upload"
#define USER_REF_IMAGE_PATH "/sdcard/user_ref.jpg"

// Kiroku Video Recording Parameters
#define KIROKU_VIDEO_FPS      5      // 5 FPS provides smooth playback & stable SD writing
#define KIROKU_DURATION_SEC   150    // 2.30 minutes (150 seconds)
#define TOTAL_KIROKU_FRAMES   (KIROKU_VIDEO_FPS * KIROKU_DURATION_SEC) // 750 frames total

static bool s_camera_ready = false;
static bool s_sd_ready = false;

// Container for on-demand user reference image
typedef struct {
    uint8_t *buf;
    size_t len;
} user_ref_img_t;

// Static module-level handle for user reference image during kansei sweep
static user_ref_img_t s_current_user_ref = {0};

// Helper function to read reference image on-demand from SD Card
static bool load_user_ref_image(user_ref_img_t *ref_img)
{
    if (!s_sd_ready || !ref_img) return false;

    ref_img->buf = NULL;
    ref_img->len = 0;

    FILE *f = fopen(USER_REF_IMAGE_PATH, "rb");
    if (!f) {
        ESP_LOGW(TAG, "kansei: user reference image (%s) not found on SD, proceeding with camera frame only", USER_REF_IMAGE_PATH);
        return false;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size <= 0) {
        fclose(f);
        return false;
    }

    ref_img->buf = (uint8_t *)malloc(size);
    if (!ref_img->buf) {
        ESP_LOGE(TAG, "kansei: failed to allocate RAM for user ref image (%ld bytes)", size);
        fclose(f);
        return false;
    }

    size_t read_bytes = fread(ref_img->buf, 1, size, f);
    fclose(f);

    if (read_bytes != (size_t)size) {
        ESP_LOGE(TAG, "kansei: short read on user ref image");
        free(ref_img->buf);
        ref_img->buf = NULL;
        return false;
    }

    ref_img->len = read_bytes;
    ESP_LOGI(TAG, "kansei: loaded user ref image from SD (%zu bytes)", ref_img->len);
    return true;
}

static void free_user_ref_image(user_ref_img_t *ref_img)
{
    if (ref_img && ref_img->buf) {
        free(ref_img->buf);
        ref_img->buf = NULL;
        ref_img->len = 0;
    }
}

static void on_kansei_angle(uint8_t angle_deg)
{
    if (!s_camera_ready) return;

    camera_fb_t *fb = camera_driver_capture();
    if (!fb) {
        ESP_LOGW(TAG, "kansei: capture failed at angle %d, skipping upload", angle_deg);
        return;
    }

    int status = 0;

    /*
     * TEMPORARY HTTP / VLM PLACEHOLDER:
     * When ready, pass `fb->buf` (live frame) AND `s_current_user_ref.buf` (user image)
     * to your multipart HTTP POST payload handler.
     */
    esp_err_t err = http_send_image_frame(fb->buf, fb->len, KANSEI_UPLOAD_URL, &status);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "kansei: upload failed at angle %d (status=%d)", angle_deg, status);
    } else {
        ESP_LOGI(TAG, "kansei: uploaded frame at angle %d (status=%d, ref_present=%s)",
                 angle_deg, status, (s_current_user_ref.buf) ? "YES" : "NO");
    }

    camera_driver_return(fb);
}

static void handle_kansei(const cam_trigger_packet_t *packet)
{
    ESP_LOGI(TAG, "kansei job started, pitch=%.2f deg", packet->pitch_centideg / 100.0f);
    servo_apply_tilt_correction(packet->pitch_centideg);

    if (packet->has_wifi_creds) {
        wifi_client_set_credentials(packet->ssid, packet->password);
    }

    if (wifi_client_connect(10000) != ESP_OK) {
        ESP_LOGE(TAG, "kansei: wifi connect failed, aborting job");
        uart_protocol_send_event(CAM_EVENT_JOB_FAILED, NULL, 0);
        return;
    }

    if (!s_camera_ready) {
        ESP_LOGE(TAG, "kansei: camera not ready, aborting job");
        wifi_client_disconnect();
        uart_protocol_send_event(CAM_EVENT_JOB_FAILED, NULL, 0);
        return;
    }

    // 1. Read user reference image from SD Card ON-DEMAND for this job only
    load_user_ref_image(&s_current_user_ref);

    // 2. Perform initial capture and pan sweep
    on_kansei_angle(0);
    servo_pan_sweep(on_kansei_angle);

    // 3. Immediately free reference image buffer after sweep completes
    free_user_ref_image(&s_current_user_ref);

    wifi_client_disconnect();
    uart_protocol_send_event(CAM_EVENT_JOB_DONE, NULL, 0);
}

static void handle_kiroku(const cam_trigger_packet_t *packet)
{
    ESP_LOGI(TAG, "kiroku job started: 2.30-min multi-axis recording (%d frames @ %d FPS), pitch=%.2f deg",
             TOTAL_KIROKU_FRAMES, KIROKU_VIDEO_FPS, packet->pitch_centideg / 100.0f);

    if (!s_camera_ready || !s_sd_ready) {
        ESP_LOGE(TAG, "kiroku: camera or SD not ready, aborting job");
        uart_protocol_send_event(CAM_EVENT_JOB_FAILED, NULL, 0);
        return;
    }

    char path[512];
    esp_err_t err = sd_storage_open_video(camera_driver_get_width(),
                                           camera_driver_get_height(),
                                           KIROKU_VIDEO_FPS, path, sizeof(path));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "kiroku: failed to open video file, aborting job");
        uart_protocol_send_event(CAM_EVENT_JOB_FAILED, NULL, 0);
        return;
    }

    float pitch_deg = packet->pitch_centideg / 100.0f;
    int base_tilt = 90 - (int)pitch_deg;

    int tilt_offsets[3] = { -15, 0, 15 };
    int current_pass = 0;

    servo_set_tilt((uint8_t)(base_tilt + tilt_offsets[current_pass]));
    servo_set_pan(20);
    vTaskDelay(pdMS_TO_TICKS(300));

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frame_interval = pdMS_TO_TICKS(1000 / KIROKU_VIDEO_FPS);

    float current_pan_fp = 20.0f;
    int pan_dir = 1;
    int frames_per_pass = TOTAL_KIROKU_FRAMES / 3; // 250 frames (50s) per elevation pass

    for (int frame = 0; frame < TOTAL_KIROKU_FRAMES; frame++) {

        if (frame > 0 && frame % frames_per_pass == 0) {
            current_pass++;
            if (current_pass < 3) {
                int target_tilt = base_tilt + tilt_offsets[current_pass];
                ESP_LOGI(TAG, "kiroku: switching elevation pass %d/3 (tilt=%d deg)", 
                         current_pass + 1, target_tilt);
                servo_set_tilt((uint8_t)target_tilt);
            }
        }

        // Updated speed: 1.5 deg step per frame (7.5 deg/sec at 5 FPS)
        current_pan_fp += (pan_dir * 1.5f);
        if (current_pan_fp >= 170.0f) {
            current_pan_fp = 170.0f;
            pan_dir = -1;
        } else if (current_pan_fp <= 20.0f) {
            current_pan_fp = 20.0f;
            pan_dir = 1;
        }
        servo_set_pan((uint8_t)current_pan_fp);

        camera_fb_t *fb = camera_driver_capture();
        if (fb) {
            esp_err_t w_err = sd_storage_write_frame(fb->buf, fb->len);
            if (w_err != ESP_OK) {
                ESP_LOGW(TAG, "kiroku: SD write failed at frame %d", frame);
            }
            camera_driver_return(fb);
        } else {
            ESP_LOGW(TAG, "kiroku: frame %d capture dropped", frame);
        }

        vTaskDelayUntil(&last_wake_time, frame_interval);
    }

    servo_set_pan(90);
    servo_set_tilt((uint8_t)base_tilt);

    sd_storage_close_video();
    ESP_LOGI(TAG, "kiroku: saved multi-axis recording %s", path);
    uart_protocol_send_event(CAM_EVENT_JOB_DONE, NULL, 0);
}

static void uart_listener_task(void *arg)
{
    cam_trigger_packet_t packet;
    while (1) {
        if (uart_protocol_read_trigger(&packet) == 0) {
            uart_protocol_send_event(CAM_EVENT_JOB_STARTED, NULL, 0);
            if (packet.cmd == CAM_CMD_KANSEI) {
                handle_kansei(&packet);
            } else if (packet.cmd == CAM_CMD_KIROKU) {
                handle_kiroku(&packet);
            } else {
                ESP_LOGW(TAG, "Unknown command 0x%02X", packet.cmd);
                uart_protocol_send_event(CAM_EVENT_JOB_FAILED, NULL, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "MIO CAM booting");

    uart_protocol_init();

    /* 1. MOUNT SD CARD FIRST (Before Servos touch GPIO 12 & 13) */
    esp_err_t sd_err = sd_storage_init();
    s_sd_ready = (sd_err == ESP_OK);
    if (!s_sd_ready) {
        ESP_LOGW(TAG, "SD init failed -- kiroku recording will not work this boot");
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    /* 2. INITIALIZE SERVOS AFTER SD IS MOUNTED */
    servo_control_init();

    vTaskDelay(pdMS_TO_TICKS(150)); 

    /* 3. INITIALIZE CAMERA */
    esp_err_t cam_err = camera_driver_init();
    s_camera_ready = (cam_err == ESP_OK);
    if (!s_camera_ready) {
        ESP_LOGW(TAG, "camera init failed -- kiroku/kansei captures will not work this boot");
    }

    wifi_client_init(WIFI_SSID, WIFI_PASSWORD);
    if (wifi_client_connect(10000) == ESP_OK) {
        ESP_LOGI(TAG, "boot-time priming connect OK");
    }
    wifi_client_disconnect();

    xTaskCreatePinnedToCore(uart_listener_task, "uart_listener", 8192, NULL, 5, NULL, 1);
}