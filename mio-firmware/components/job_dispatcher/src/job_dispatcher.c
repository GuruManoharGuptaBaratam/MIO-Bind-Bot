#include "job_dispatcher.h"
#include "mpu6050.h"
#include "cam_link.h"
#include "tft_display.h"
#include "esp_log.h"

static const char *TAG = "JOB_DISPATCH";

// TODO: this is where "disable mic until job done" gating belongs once
// that's ready - e.g. set a busy flag on CAM_LINK_EVT_JOB_STARTED and
// check it before acting on further LISTENING_WAKEWORD events, clearing
// the flag on CAM_LINK_EVT_JOB_DONE/FAILED.
static void on_cam_link_event(cam_link_event_t evt)
{
    switch (evt) {
        case CAM_LINK_EVT_JOB_STARTED:
            ESP_LOGI(TAG, "CAM job started");
            break;
        case CAM_LINK_EVT_JOB_DONE:
            ESP_LOGI(TAG, "CAM job done");
            break;
        case CAM_LINK_EVT_JOB_FAILED:
            ESP_LOGW(TAG, "CAM job failed");
            break;
    }
    // TODO: route to tft_display for "recording...", "saved" OLED states
    // once that state machine exists.
}

void job_dispatcher_init(void)
{
    if (mpu6050_init() != ESP_OK) {
        ESP_LOGW(TAG, "IMU init failed - pitch will be sent as 0 until fixed");
    }
    cam_link_init();
    cam_link_set_event_callback(on_cam_link_event);
}

void job_dispatcher_on_command(core_command_t cmd)
{
    // Preserve existing display behavior for every command.
    tft_display_on_ie_command(cmd);

    if (cmd == CORE_CMD_KANSEI || cmd == CORE_CMD_KIROKU) {
        int16_t pitch_centideg = 0;
        if (mpu6050_read_pitch_centideg(&pitch_centideg) != ESP_OK) {
            ESP_LOGW(TAG, "Pitch read failed, sending 0 deg to CAM");
        }
        cam_link_send_trigger(cmd, pitch_centideg);
    }
    // CORE_CMD_IBASHO intentionally not forwarded to CAM.
}

void job_dispatcher_on_event(core_event_t evt)
{
    tft_display_on_ie_event(evt);
}