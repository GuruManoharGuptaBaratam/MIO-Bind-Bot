#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "uart_protocol.h"
#include "servo_control.h"
#include "wifi_client.h"
#include "wifi_credentials.h"

static const char *TAG = "mio_cam";

/* TODO: replace with your MacBook's actual LAN IP/port + upload route */
#define KANSEI_UPLOAD_URL "http://192.168.1.100:8000/upload"

static void on_kansei_angle(uint8_t angle_deg)
{
    /* NOTE: this ESP_LOGx call goes out over the same UART0 wire as
     * uart_protocol's binary packets to Core (see pin_config.h /
     * CONFIG_CONSOLE_UART_NUM=0). Safe for now while debugging with
     * UART disconnected from Core, but should be removed or the
     * console redirected before Core is wired up for real. */
    ESP_LOGI(TAG, "kansei: capturing at angle %d deg", angle_deg);

    /* Assumes esp_camera_init() has already run somewhere before this
     * point (not shown in the uploaded main.c) -- if that's handled
     * elsewhere, ignore this comment. If it isn't happening anywhere
     * yet, this call will return NULL every time. */
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == NULL) {
        ESP_LOGE(TAG, "kansei: camera capture failed at angle %d", angle_deg);
        return;
    }

    int status = 0;
    esp_err_t err = http_send_image_frame(fb->buf, fb->len, KANSEI_UPLOAD_URL, &status);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "kansei: upload failed at angle %d (http status %d)", angle_deg, status);
    }

    esp_camera_fb_return(fb);
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

    /* Connect once for the whole sweep, not once per angle -- fewer
     * RF calibration/reconnect cycles, less time with the radio on. */
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

    /* Registers WiFi driver + event handlers only -- radio stays off
     * until handle_kansei() calls wifi_client_connect(). */
    wifi_client_init(WIFI_SSID, WIFI_PASSWORD);

    /* Bumped from 4096: this task now also does camera capture +
     * esp_http_client on top of servo/UART work. */
    xTaskCreate(uart_listener_task, "uart_listener", 8192, NULL, 5, NULL);
}