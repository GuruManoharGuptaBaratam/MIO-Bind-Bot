#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "uart_protocol.h"
#include "servo_control.h"
#include "wifi_client.h"
#include "wifi_credentials.h"

static const char *TAG = "mio_cam";

#define KANSEI_UPLOAD_URL "http://192.168.1.100:8000/upload"

static void on_kansei_angle(uint8_t angle_deg)
{
    ESP_LOGI(TAG, "kansei: at angle %d deg (capture + HTTP upload goes here)", angle_deg);
    // TODO: esp_camera_fb_get() + http_send_image_frame() once camera
    // capture is wired back in. wifi_client_connect() must already have
    // succeeded before handle_kansei() calls servo_pan_sweep(), so the
    // link is live for every callback here -- no need to reconnect per-angle.
}

static void on_kiroku_angle(uint8_t angle_deg)
{
    ESP_LOGI(TAG, "kiroku: at angle %d deg (video frame capture goes here)", angle_deg);
    // TODO: write a video frame to the SD card here
}

static void handle_kansei(int16_t pitch_centideg)
{
    ESP_LOGI(TAG, "kansei job started, pitch=%.2f deg", pitch_centideg / 100.0f);
    servo_apply_tilt_correction(pitch_centideg);

    if (wifi_client_connect(10000) != ESP_OK) {
        ESP_LOGE(TAG, "kansei: wifi connect failed, aborting job");
        uart_protocol_send_event(CAM_EVENT_JOB_FAILED, NULL, 0);
        return;
    }

    servo_pan_sweep(on_kansei_angle);

    wifi_client_disconnect();

    // TODO: once all angles are captured, send the scene description back
    // to Core via uart_protocol_send_event() with real payload data.
    uart_protocol_send_event(CAM_EVENT_JOB_DONE, NULL, 0);
}

static void handle_kiroku(int16_t pitch_centideg)
{
    ESP_LOGI(TAG, "kiroku job started, pitch=%.2f deg", pitch_centideg / 100.0f);
    servo_apply_tilt_correction(pitch_centideg);
    servo_pan_sweep(on_kiroku_angle);
    // TODO: record 2-3 min continuous video to SD, auto-stop, then report done
    uart_protocol_send_event(CAM_EVENT_JOB_DONE, NULL, 0);
}

static void uart_listener_task(void *arg)
{
    cam_trigger_packet_t packet;
    while (1) {
        if (uart_protocol_read_trigger(&packet) == 0) {
            uart_protocol_send_event(CAM_EVENT_JOB_STARTED, NULL, 0);
            if (packet.cmd == CAM_CMD_KANSEI) {
                handle_kansei(packet.pitch_centideg);
            } else if (packet.cmd == CAM_CMD_KIROKU) {
                handle_kiroku(packet.pitch_centideg);
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
    servo_control_init();

    /* Registers WiFi driver + event handlers only -- radio stays off,
     * no current spike yet. */
    wifi_client_init(WIFI_SSID, WIFI_PASSWORD);

    /* One-time priming connect, done here in the quiet boot window
     * before uart_listener_task exists and before any job is running.
     * This is deliberately the ONLY place a full RF calibration should
     * ever happen -- it forces the expensive first-time calibration
     * write to occur while nothing else is contending for flash/cache,
     * so every later per-job connect (triggered from inside
     * handle_kansei) uses the now-cached calibration data and does the
     * much lighter "partial calibration" instead. */
    if (wifi_client_connect(10000) == ESP_OK) {
        ESP_LOGI(TAG, "boot-time priming connect OK");
    } else {
        ESP_LOGW(TAG, "boot-time priming connect failed (will retry per-job)");
    }
    wifi_client_disconnect();

    xTaskCreatePinnedToCore(uart_listener_task, "uart_listener", 8192, NULL, 5, NULL, 1);
}