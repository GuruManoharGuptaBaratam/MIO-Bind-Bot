#include "job_dispatcher.h"
#include "mpu6050.h"
#include "cam_link.h"
#include "tft_display.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "audio_feedback.h"

static const char *TAG = "JOB_DISPATCH";

static bool s_is_system_busy = false;
static core_command_t s_current_cmd = (core_command_t)0xFF;
static TimerHandle_t s_job_timeout_timer = NULL;

static void clear_busy_state(void)
{
    s_is_system_busy = false;
    s_current_cmd = (core_command_t)0xFF;
    if (s_job_timeout_timer && xTimerIsTimerActive(s_job_timeout_timer)) {
        xTimerStop(s_job_timeout_timer, 0);
    }
    ESP_LOGI(TAG, "System State: IDLE (Ready for next wakeword)");
}

static void on_job_timeout_callback(TimerHandle_t xTimer)
{
    ESP_LOGW(TAG, "Job timeout reached -- forcefully unlocking system busy state");
    
    // Notify display module to halt periodic reminder timer & reset state
    tft_display_on_job_timeout();
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
            
            // Trigger completion screen & completion audio based on active command
            if (s_current_cmd == CORE_CMD_KANSEI) {
                tft_display_on_command_done(SND_SCENE_DONE);
            } else if (s_current_cmd == CORE_CMD_KIROKU) {
                tft_display_on_command_done(SND_RECORDING_SAVED);
            }
            
            clear_busy_state();
            break;

        case CAM_LINK_EVT_JOB_FAILED:
            ESP_LOGW(TAG, "CAM job failed -- unlocking system");
            
            // Reset state, stop reminder timer, and play error feedback audio
            clear_busy_state();
            tft_display_on_camera_failed();
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
    s_current_cmd = (core_command_t)0xFF;
}

void job_dispatcher_on_command(core_command_t cmd)
{
    // Ignore any new incoming command if a command is already running
    if (s_is_system_busy) {
        ESP_LOGW(TAG, "System BUSY -- ignoring new command 0x%02X", cmd);
        tft_display_on_button_busy();
        return;
    }

    if (cmd == CORE_CMD_KANSEI) {
        s_current_cmd = cmd;

        // 1. Show "CMD: KANSEI" on display & play command detection audio
        tft_display_on_ie_command(cmd);

        // 2. Hold for 1.5s to allow user to see/hear command detection
        vTaskDelay(pdMS_TO_TICKS(1500));
        
        // 3. Starts "SWEEPING..." display state & initiates periodic reminder loop
        tft_display_on_command_processing(SND_SCENE_PROCESSING);
        set_busy_state_with_timeout(300000); // 120s timeout window

        int16_t pitch_centideg = 0;
        if (mpu6050_read_pitch_centideg(&pitch_centideg) != ESP_OK) {
            ESP_LOGW(TAG, "Pitch read failed, sending 0 deg to CAM");
        }
        cam_link_send_trigger(cmd, pitch_centideg);

    } else if (cmd == CORE_CMD_KIROKU) {
        s_current_cmd = cmd;

        // 1. Show "CMD: KIROKU" on display & play command detection audio
        tft_display_on_ie_command(cmd);

        // 2. Hold for 1.5s to allow user to see/hear command detection
        vTaskDelay(pdMS_TO_TICKS(1500));
        
        // 3. Starts "RECORDING..." display state & initiates periodic reminder loop
        tft_display_on_command_processing(SND_RECORDING_STARTED);
        set_busy_state_with_timeout(180000); // 180s timeout window

        int16_t pitch_centideg = 0;
        if (mpu6050_read_pitch_centideg(&pitch_centideg) != ESP_OK) {
            ESP_LOGW(TAG, "Pitch read failed, sending 0 deg to CAM");
        }
        cam_link_send_trigger(cmd, pitch_centideg);

    } else if (cmd == CORE_CMD_IBASHO) {
        s_current_cmd = cmd;
        tft_display_on_ie_command(cmd);
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