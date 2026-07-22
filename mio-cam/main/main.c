#include <stdio.h>
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
#define KIROKU_VIDEO_FPS  2   // matches roughly one frame per servo hold

static bool s_camera_ready = false;
static bool s_sd_ready = false;

static void on_kansei_angle(uint8_t angle_deg)
{
    if (!s_camera_ready) return;

    camera_fb_t *fb = camera_driver_capture();
    if (!fb) {
        ESP_LOGW(TAG, "kansei: capture failed at angle %d, skipping upload", angle_deg);
        return;
    }

    int status = 0;
    esp_err_t err = http_send_image_frame(fb->buf, fb->len, KANSEI_UPLOAD_URL, &status);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "kansei: upload failed at angle %d (status=%d)", angle_deg, status);
    } else {
        ESP_LOGI(TAG, "kansei: uploaded frame at angle %d (status=%d)", angle_deg, status);
    }

    camera_driver_return(fb);
}

static void on_kiroku_angle(uint8_t angle_deg)
{
    if (!s_camera_ready || !s_sd_ready) return;

    camera_fb_t *fb = camera_driver_capture();
    if (!fb) {
        ESP_LOGW(TAG, "kiroku: capture failed at angle %d, skipping frame", angle_deg);
        return;
    }

    esp_err_t err = sd_storage_write_frame(fb->buf, fb->len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "kiroku: SD write failed at angle %d", angle_deg);
    } else {
        ESP_LOGI(TAG, "kiroku: wrote frame at angle %d (%u bytes)", angle_deg, (unsigned)fb->len);
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
    // else: keep using whatever's currently set (wifi_credentials.h
    // fallback from boot, or the last job's SD-sourced credentials)

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

    on_kansei_angle(0);   // initial capture, before the sweep starts
    servo_pan_sweep(on_kansei_angle);

    wifi_client_disconnect();
    uart_protocol_send_event(CAM_EVENT_JOB_DONE, NULL, 0);
}

static void handle_kiroku(const cam_trigger_packet_t *packet)
{
    ESP_LOGI(TAG, "kiroku job started, pitch=%.2f deg", packet->pitch_centideg / 100.0f);
    servo_apply_tilt_correction(packet->pitch_centideg);

    if (!s_camera_ready || !s_sd_ready) {
        ESP_LOGE(TAG, "kiroku: camera or SD not ready, aborting job");
        uart_protocol_send_event(CAM_EVENT_JOB_FAILED, NULL, 0);
        return;
    }

    char path[64];
    esp_err_t err = sd_storage_open_video(camera_driver_get_width(),
                                           camera_driver_get_height(),
                                           KIROKU_VIDEO_FPS, path, sizeof(path));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "kiroku: failed to open video file, aborting job");
        uart_protocol_send_event(CAM_EVENT_JOB_FAILED, NULL, 0);
        return;
    }

    on_kiroku_angle(0);   // initial capture, before the sweep starts
    servo_pan_sweep(on_kiroku_angle);

    sd_storage_close_video();
    ESP_LOGI(TAG, "kiroku: saved %s", path);
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