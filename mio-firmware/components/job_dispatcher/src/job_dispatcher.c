#include "job_dispatcher.h"
#include "mpu6050.h"
#include "cam_link.h"
#include "tft_display.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

static const char *TAG = "JOB_DISPATCH";

static bool s_is_system_busy = false;
static TimerHandle_t s_job_timeout_timer = NULL;

static void clear_busy_state(void)
{
    s_is_system_busy = false;
    if (s_job_timeout_timer && xTimerIsTimerActive(s_job_timeout_timer)) {
        xTimerStop(s_job_timeout_timer, 0);
    }
    ESP_LOGI(TAG, "System State: IDLE (Ready for next wakeword)");
}

static void on_job_timeout_callback(TimerHandle_t xTimer)
{
    ESP_LOGW(TAG, "Job timeout reached -- forcefully unlocking system busy state");
    clear_busy_state();
}

static void set_busy_state_with_timeout(uint32_t timeout_ms)
{
    s_is_system_busy = true;
    ESP_LOGI(TAG, "System State: BUSY (Gating further wakeword/commands for %lu ms)", (unsigned long)timeout_ms);

    if (!s_job_timeout_timer) {
        s_job_timeout_timer = xTimerCreate("job_timeout", pdMS_TO_TICKS(timeout_ms),
                                           pdFALSE, NULL, on_job_timeout_callback);
    } else {
        xTimerChangePeriod(s_job_timeout_timer, pdMS_TO_TICKS(timeout_ms), 0);
    }
    xTimerStart(s_job_timeout_timer, 0);
}

static void on_cam_link_event(cam_link_event_t evt)
{
    switch (evt) {
        case CAM_LINK_EVT_JOB_STARTED:
            ESP_LOGI(TAG, "CAM job started");
            break;
        case CAM_LINK_EVT_JOB_DONE:
            ESP_LOGI(TAG, "CAM job done -- unlocking system");
            clear_busy_state();
            break;
        case CAM_LINK_EVT_JOB_FAILED:
            ESP_LOGW(TAG, "CAM job failed -- unlocking system");
            clear_busy_state();
            break;
    }
}

void job_dispatcher_init(void)
{
    if (mpu6050_init() != ESP_OK) {
        ESP_LOGW(TAG, "IMU init failed - pitch will be sent as 0 until fixed");
    }
    cam_link_init();
    cam_link_set_event_callback(on_cam_link_event);
    s_is_system_busy = false;
}

void job_dispatcher_on_command(core_command_t cmd)
{
    // Ignore any new incoming command if a command (kansei/kiroku/ibasho) is already running!
    if (s_is_system_busy) {
        ESP_LOGW(TAG, "System BUSY -- ignoring new command 0x%02X", cmd);
        return;
    }

    // Preserve existing display behavior for every command.
    tft_display_on_ie_command(cmd);

    if (cmd == CORE_CMD_KANSEI) {
        // Kansei takes ~10-15s for sweep & upload -> 30s safeguard timeout
        set_busy_state_with_timeout(30000);

        int16_t pitch_centideg = 0;
        if (mpu6050_read_pitch_centideg(&pitch_centideg) != ESP_OK) {
            ESP_LOGW(TAG, "Pitch read failed, sending 0 deg to CAM");
        }
        cam_link_send_trigger(cmd, pitch_centideg);

    } else if (cmd == CORE_CMD_KIROKU) {
        // Kiroku takes 2.30 mins (150s) -> 160s safeguard timeout
        set_busy_state_with_timeout(160000);

        int16_t pitch_centideg = 0;
        if (mpu6050_read_pitch_centideg(&pitch_centideg) != ESP_OK) {
            ESP_LOGW(TAG, "Pitch read failed, sending 0 deg to CAM");
        }
        cam_link_send_trigger(cmd, pitch_centideg);

    } else if (cmd == CORE_CMD_IBASHO) {
        // Ibasho is local-only (e.g. 5-second UI/audio execution)
        set_busy_state_with_timeout(5000);
        ESP_LOGI(TAG, "Executing IBASHO command locally");
    }
}

void job_dispatcher_on_event(core_event_t evt)
{
    tft_display_on_ie_event(evt);
}

bool job_dispatcher_is_busy(void)
{
    return s_is_system_busy;
}