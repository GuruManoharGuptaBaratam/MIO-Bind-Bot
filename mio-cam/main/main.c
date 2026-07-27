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

#define MAX_KANSEI_FRAMES 4

// Kiroku Video Recording Parameters
#define KIROKU_VIDEO_FPS      5      
#define KIROKU_DURATION_SEC   150    
#define TOTAL_KIROKU_FRAMES   (KIROKU_VIDEO_FPS * KIROKU_DURATION_SEC) 

static bool s_camera_ready = false;
static bool s_sd_ready = false;

typedef struct {
    uint8_t *buf;
    size_t len;
} user_ref_img_t;

static user_ref_img_t s_current_user_ref = {0};

// Temporary frame array held in PSRAM during fast sweep
static camera_fb_t *s_kansei_frames[MAX_KANSEI_FRAMES] = {NULL};
static size_t s_kansei_frame_count = 0;

static bool load_user_ref_image(user_ref_img_t *ref_img)
{
    if (!s_sd_ready || !ref_img) return false;

    ref_img->buf = NULL;
    ref_img->len = 0;

    FILE *f = fopen(USER_REF_IMAGE_PATH, "rb");
    if (!f) {
        ESP_LOGW(TAG, "kansei: user reference image not found, proceeding with live frames");
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

// Fast capture callback (Zero Network delays during physical movement)
static void on_kansei_angle_fast(uint8_t angle_deg)
{
    if (!s_camera_ready || s_kansei_frame_count >= MAX_KANSEI_FRAMES) return;

    camera_fb_t *fb = camera_driver_capture();
    if (!fb) {
        ESP_LOGW(TAG, "kansei: capture failed at angle %d", angle_deg);
        return;
    }

    s_kansei_frames[s_kansei_frame_count++] = fb;
    ESP_LOGI(TAG, "kansei: captured frame %d at angle %d deg", (int)s_kansei_frame_count, angle_deg);
}

static void clear_kansei_frames(void)
{
    for (size_t i = 0; i < s_kansei_frame_count; i++) {
        if (s_kansei_frames[i]) {
            camera_driver_return(s_kansei_frames[i]);
            s_kansei_frames[i] = NULL;
        }
    }
    s_kansei_frame_count = 0;
}

static void handle_kansei(const cam_trigger_packet_t *packet)
{
    ESP_LOGI(TAG, "kansei job started, pitch=%.2f deg", packet->pitch_centideg / 100.0f);

    if (!s_camera_ready) {
        ESP_LOGE(TAG, "kansei: camera not ready, aborting");
        uart_protocol_send_event(CAM_EVENT_JOB_FAILED, NULL, 0);
        return;
    }

    // 1. Vertical Horizon Lock before starting horizontal panning
    servo_apply_tilt_correction(packet->pitch_centideg);

    // 2. Perform fast continuous 4-frame sweep (~1.2s total physical execution time)
    s_kansei_frame_count = 0;
    servo_pan_sweep(on_kansei_angle_fast);

    // 3. Connect to Wi-Fi after physical movement finishes (servo resting at 90 deg)
    if (packet->has_wifi_creds) {
        wifi_client_set_credentials(packet->ssid, packet->password);
    }

    if (wifi_client_connect(10000) != ESP_OK) {
        ESP_LOGE(TAG, "kansei: wifi connect failed, aborting upload");
        clear_kansei_frames();
        uart_protocol_send_event(CAM_EVENT_JOB_FAILED, NULL, 0);
        return;
    }

    // 4. Read user reference image on-demand from SD card
    load_user_ref_image(&s_current_user_ref);

    // 5. Send all frames in 1 single HTTP multipart POST
    int status = 0;
    esp_err_t err = http_send_batch_kansei_frames(
        s_kansei_frames, s_kansei_frame_count,
        s_current_user_ref.buf, s_current_user_ref.len,
        KANSEI_UPLOAD_URL, &status
    );

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "kansei: batch upload failed (status=%d)", status);
        uart_protocol_send_event(CAM_EVENT_JOB_FAILED, NULL, 0);
    } else {
        ESP_LOGI(TAG, "kansei: batch uploaded %d frames successfully", (int)s_kansei_frame_count);
        uart_protocol_send_event(CAM_EVENT_JOB_DONE, NULL, 0);
    }

    // 6. Cleanup PSRAM buffers and drop Wi-Fi connection
    free_user_ref_image(&s_current_user_ref);
    clear_kansei_frames();
    wifi_client_disconnect();
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
    int frames_per_pass = TOTAL_KIROKU_FRAMES / 3;

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

    /* 1. MOUNT SD CARD FIRST */
    esp_err_t sd_err = sd_storage_init();
    s_sd_ready = (sd_err == ESP_OK);
    if (!s_sd_ready) {
        ESP_LOGW(TAG, "SD init failed -- kiroku recording disabled");
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    /* 2. INITIALIZE SERVOS */
    servo_control_init();

    vTaskDelay(pdMS_TO_TICKS(150)); 

    /* 3. INITIALIZE CAMERA */
    esp_err_t cam_err = camera_driver_init();
    s_camera_ready = (cam_err == ESP_OK);
    if (!s_camera_ready) {
        ESP_LOGW(TAG, "camera init failed");
    }

    wifi_client_init(WIFI_SSID, WIFI_PASSWORD);
    if (wifi_client_connect(10000) == ESP_OK) {
        ESP_LOGI(TAG, "boot-time priming connect OK");
    }
    wifi_client_disconnect();

    xTaskCreatePinnedToCore(uart_listener_task, "uart_listener", 8192, NULL, 5, NULL, 1);
}