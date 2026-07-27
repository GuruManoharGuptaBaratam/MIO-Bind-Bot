#include "servo_control.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "SERVO";

#define PIN_SERVO_PAN   13   // GPIO13 -> MG90S #1 (pan)
#define PIN_SERVO_TILT  12   // GPIO12 -> MG90S #2 (tilt)

#define SERVO_LEDC_MODE       LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_TIMER      LEDC_TIMER_0
#define SERVO_LEDC_RES_BITS   LEDC_TIMER_16_BIT
#define SERVO_LEDC_FREQ_HZ    50
#define SERVO_PAN_CHANNEL     LEDC_CHANNEL_0
#define SERVO_TILT_CHANNEL    LEDC_CHANNEL_1

#define SERVO_MIN_PULSE_US    500
#define SERVO_MAX_PULSE_US    2500
#define SERVO_PERIOD_US       20000

#define SERVO_PAN_ANGLE_MIN    0
#define SERVO_PAN_ANGLE_MAX    180
#define SERVO_TILT_ANGLE_MIN   0
#define SERVO_TILT_ANGLE_MAX   180

#define BASE_TILT_ANGLE_DEG   90

static const uint8_t PAN_SWEEP_ANGLES[] = { 15, 63, 111, 155 };
#define PAN_SWEEP_STEPS  (sizeof(PAN_SWEEP_ANGLES) / sizeof(PAN_SWEEP_ANGLES[0]))

#define PAN_SLEW_STEP_DEG        4
#define PAN_SLEW_STEP_DELAY_MS   10

// 1 second (1000ms) delay at each angle to give camera full exposure stability
#define PAN_SWEEP_ARRIVE_SETTLE_MS  1000   
#define PAN_SWEEP_HOLD_MS           100   

#define TILT_SLEW_STEP_DEG        4
#define TILT_SLEW_STEP_DELAY_MS   10

static uint8_t clamp_pan_angle(int angle_deg)
{
    if (angle_deg < SERVO_PAN_ANGLE_MIN) return SERVO_PAN_ANGLE_MIN;
    if (angle_deg > SERVO_PAN_ANGLE_MAX) return SERVO_PAN_ANGLE_MAX;
    return (uint8_t)angle_deg;
}

static uint8_t clamp_tilt_angle(int angle_deg)
{
    if (angle_deg < SERVO_TILT_ANGLE_MIN) return SERVO_TILT_ANGLE_MIN;
    if (angle_deg > SERVO_TILT_ANGLE_MAX) return SERVO_TILT_ANGLE_MAX;
    return (uint8_t)angle_deg;
}

static uint32_t angle_to_duty(uint8_t angle_deg)
{
    uint32_t pulse_us = SERVO_MIN_PULSE_US +
        ((uint32_t)angle_deg * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180;
    uint32_t max_duty = (1 << SERVO_LEDC_RES_BITS) - 1;
    return (uint32_t)(((uint64_t)pulse_us * max_duty) / SERVO_PERIOD_US);
}

static void servo_write(ledc_channel_t channel, uint8_t angle_deg)
{
    uint32_t duty = angle_to_duty(angle_deg);
    ledc_set_duty(SERVO_LEDC_MODE, channel, duty);
    ledc_update_duty(SERVO_LEDC_MODE, channel);
}

static uint8_t s_pan_current_angle  = 90;
static uint8_t s_tilt_current_angle = BASE_TILT_ANGLE_DEG;

static void servo_slew_to(ledc_channel_t channel, uint8_t *current_angle,
                           uint8_t target_angle, uint8_t step_deg, uint32_t step_delay_ms)
{
    int current = *current_angle;
    int target  = target_angle;

    if (current == target) {
        return;
    }

    int direction = (target > current) ? 1 : -1;

    while (current != target) {
        int remaining = abs(target - current);
        int step = (remaining < step_deg) ? remaining : step_deg;
        current += direction * step;

        servo_write(channel, (uint8_t)current);
        vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    }

    *current_angle = (uint8_t)target;
}

void servo_control_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = SERVO_LEDC_MODE,
        .timer_num       = SERVO_LEDC_TIMER,
        .duty_resolution = SERVO_LEDC_RES_BITS,
        .freq_hz         = SERVO_LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t pan_cfg = {
        .gpio_num   = PIN_SERVO_PAN,
        .speed_mode = SERVO_LEDC_MODE,
        .channel    = SERVO_PAN_CHANNEL,
        .timer_sel  = SERVO_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pan_cfg));

    ledc_channel_config_t tilt_cfg = {
        .gpio_num   = PIN_SERVO_TILT,
        .speed_mode = SERVO_LEDC_MODE,
        .channel    = SERVO_TILT_CHANNEL,
        .timer_sel  = SERVO_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&tilt_cfg));

    servo_write(SERVO_PAN_CHANNEL, s_pan_current_angle);
    servo_write(SERVO_TILT_CHANNEL, s_tilt_current_angle);

    ESP_LOGI(TAG, "Servos ready (pan=GPIO%d tilt=GPIO%d)", PIN_SERVO_PAN, PIN_SERVO_TILT);
}

void servo_set_pan(uint8_t angle_deg)
{
    uint8_t clamped = clamp_pan_angle(angle_deg);
    servo_slew_to(SERVO_PAN_CHANNEL, &s_pan_current_angle, clamped,
                  PAN_SLEW_STEP_DEG, PAN_SLEW_STEP_DELAY_MS);
}

void servo_set_tilt(uint8_t angle_deg)
{
    uint8_t clamped = clamp_tilt_angle(angle_deg);
    servo_slew_to(SERVO_TILT_CHANNEL, &s_tilt_current_angle, clamped,
                  TILT_SLEW_STEP_DEG, TILT_SLEW_STEP_DELAY_MS);
}

void servo_apply_tilt_correction(int16_t pitch_centideg)
{
    float pitch_deg = pitch_centideg / 100.0f;

    float horizon_correction = -pitch_deg;
    float surface_sensitivity_scaler = 1.5f; 
    float height_correction = -(pitch_deg * surface_sensitivity_scaler);

    int target = (int)(BASE_TILT_ANGLE_DEG + horizon_correction + height_correction);

    uint8_t clamped = clamp_tilt_angle(target);
    servo_slew_to(SERVO_TILT_CHANNEL, &s_tilt_current_angle, clamped,
                  TILT_SLEW_STEP_DEG, TILT_SLEW_STEP_DELAY_MS);

    ESP_LOGI(TAG, "Horizon Fit: pitch=%.2f deg -> servo=%d deg", pitch_deg, clamped);
}

void servo_pan_sweep(servo_sweep_cb_t on_each_angle)
{
    for (size_t i = 0; i < PAN_SWEEP_STEPS; i++) {
        uint8_t angle = PAN_SWEEP_ANGLES[i];
        servo_set_pan(angle);

        // Allow 1 second pause at the angle for servo stability & crisp image exposure
        vTaskDelay(pdMS_TO_TICKS(PAN_SWEEP_ARRIVE_SETTLE_MS));

        if (on_each_angle) {
            on_each_angle(angle);
        }
        vTaskDelay(pdMS_TO_TICKS(PAN_SWEEP_HOLD_MS));
    }
    servo_set_pan(90);
}