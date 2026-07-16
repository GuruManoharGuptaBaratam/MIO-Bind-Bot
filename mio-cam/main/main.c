#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "uart_protocol.h"
#include "servo_control.h"

static const char *TAG = "mio_cam";

static void on_kansei_angle(uint8_t angle_deg)
{
    ESP_LOGI(TAG, "kansei: at angle %d deg (capture + HTTP upload goes here)", angle_deg);
    // TODO: capture a frame here and send it via HTTP once WiFi is wired in
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
    servo_pan_sweep(on_kansei_angle);
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
    xTaskCreate(uart_listener_task, "uart_listener", 4096, NULL, 5, NULL);
}