#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "uart_protocol.h"

static const char *TAG = "mio_cam";

static void handle_kansei(int16_t pitch_centideg)
{
    ESP_LOGI(TAG, "kansei job started, pitch=%.2f deg (servo/camera/wifi logic goes here)",
             pitch_centideg / 100.0f);
    // TODO: apply tilt offset, sweep servos, capture + HTTP upload
    uart_protocol_send_event(CAM_EVENT_JOB_DONE, NULL, 0);
}

static void handle_kiroku(int16_t pitch_centideg)
{
    ESP_LOGI(TAG, "kiroku job started, pitch=%.2f deg (servo/camera/SD record logic goes here)",
             pitch_centideg / 100.0f);
    // TODO: apply tilt offset, sweep servos, record 2-3 min to SD, auto-stop
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
    xTaskCreate(uart_listener_task, "uart_listener", 4096, NULL, 5, NULL);
}