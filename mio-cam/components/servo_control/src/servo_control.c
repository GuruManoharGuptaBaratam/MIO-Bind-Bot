#include "servo_control.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdlib.h>   // abs()

static const char *TAG = "SERVO";

// Pins - kept local to this component, same convention as uart_protocol.c.
#define PIN_SERVO_PAN   13   // GPIO13 -> MG90S #1 orange (pan)
#define PIN_SERVO_TILT  12   // GPIO12 -> MG90S #2 orange (tilt)

#define SERVO_LEDC_MODE       LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_TIMER      LEDC_TIMER_0
#define SERVO_LEDC_RES_BITS   LEDC_TIMER_16_BIT
#define SERVO_LEDC_FREQ_HZ    50            // standard hobby servo frame rate
#define SERVO_PAN_CHANNEL     LEDC_CHANNEL_0
#define SERVO_TILT_CHANNEL    LEDC_CHANNEL_1

#define SERVO_MIN_PULSE_US    500
#define SERVO_MAX_PULSE_US    2500
#define SERVO_PERIOD_US       20000         // 1/50Hz

// Safe angle ranges - bench-tested per servo.
// PAN (new unit): moved cleanly through the full 0-180 test with no stall
// found - 10/170 leaves a small margin anyway rather than running at the
// absolute electrical extremes continuously.
// TILT (surviving unit): stalled at 50/160 - last clean angle one 5deg step
// in is 55/155, backed off another 5deg margin -> 60/150.
#define SERVO_PAN_ANGLE_MIN    0
#define SERVO_PAN_ANGLE_MAX    180
#define SERVO_TILT_ANGLE_MIN   60
#define SERVO_TILT_ANGLE_MAX   150

// "Standard view" calibration - the fixed tilt angle used when pitch
// correction is zero (device held level). Comfortably within the new
// 60-150 tilt range.
#define BASE_TILT_ANGLE_DEG   90

// Pan sweep pattern - recomputed for the real 10-170 (160deg) PAN range,
// assuming a 66deg camera FOV (standard AI-Thinker OV2640 default lens):
// centers at min+FOV/2, mid, max-FOV/2 = 43, 90, 137. Each frame overlaps
// its neighbor by ~19deg - solid coverage, no gaps, not excessively
// redundant. A 4th frame doesn't fit this range without heavy overlap
// (160deg total isn't enough room for 4 non-redundant 66deg frames) - can
// revisit if you want denser redundancy anyway.
static const uint8_t PAN_SWEEP_ANGLES[] = { 20, 90, 170 };
#define PAN_SWEEP_STEPS  (sizeof(PAN_SWEEP_ANGLES) / sizeof(PAN_SWEEP_ANGLES[0]))

// CCTV-style slow pan: move in small steps with a short delay between each,
// instead of one full-speed jump. ~2deg/25ms -> roughly 25deg/sec traverse
// speed, which reads as a deliberate pan rather than a snap. Tune to taste.
#define PAN_SLEW_STEP_DEG        2
#define PAN_SLEW_STEP_DELAY_MS   25

// Time to hold still AFTER arriving at each sweep angle before firing the
// capture callback, and to hold there after, so the frame capture (once
// wired in) has a stable, blur-free target and time to actually complete
// the HTTP upload. 300ms was mostly just waiting after an already-finished
// snap-move; this is now genuine "hold for capture" time.
#define PAN_SWEEP_ARRIVE_SETTLE_MS  150   // let mechanical vibration die down
#define PAN_SWEEP_HOLD_MS           800   // window for capture/upload to run in

// Generic tilt correction slew: smaller steps, since this can run per-job
// (or, once continuous IMU streaming lands, at several Hz) and should never
// visibly jump even for a modest correction.
#define TILT_SLEW_STEP_DEG        2
#define TILT_SLEW_STEP_DELAY_MS   20

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

// We have no position feedback from these servos, so "current angle" here
// just means "the last angle we commanded." That's all we need to slew
// smoothly between successive targets - it's the jump-to-a-brand-new-target
// behavior we're trying to get rid of, not physical position tracking.
static uint8_t s_pan_current_angle  = 90;
static uint8_t s_tilt_current_angle = BASE_TILT_ANGLE_DEG;

// Move a channel from its last known commanded angle to a new target in
// small steps, instead of one instant full-speed jump. This is what turns
// "snap to position" into "ease/pan to position."
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

    // Start centered.
    //
    // NOTE on the boot "snap": this first servo_write() is the one place in
    // this file that is still an instant jump, and it has to be - the servo
    // has received no PWM signal yet, so there is no "current angle" to
    // slew from. Whatever physical position it happened to rest in, it
    // will move to 90/BASE_TILT_ANGLE_DEG at full speed the moment power +
    // signal are applied. That one-time jump at boot is normal servo
    // behavior, not a bug. What WAS a bug is that every move after this one
    // was also an instant jump (via servo_apply_tilt_correction ->
    // servo_write); that's fixed below - all runtime moves now slew.
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

    // NOTE: sign here is a placeholder - whether tilting the wearer forward
    // should raise or lower the camera's compensating angle depends on
    // which way your MPU6050's axis is mounted relative to the tilt servo.
    // Verify with a physical test (tilt the device a known way, confirm the
    // servo moves the direction that keeps the framing level) and flip the
    // sign below if it moves the wrong way.
    int target = (int)(BASE_TILT_ANGLE_DEG - pitch_deg);

    uint8_t clamped = clamp_tilt_angle(target);
    servo_slew_to(SERVO_TILT_CHANNEL, &s_tilt_current_angle, clamped,
                  TILT_SLEW_STEP_DEG, TILT_SLEW_STEP_DELAY_MS);

    ESP_LOGI(TAG, "Tilt correction: pitch=%.2f deg -> servo=%d deg", pitch_deg, clamped);
}

void servo_pan_sweep(servo_sweep_cb_t on_each_angle)
{
    for (size_t i = 0; i < PAN_SWEEP_STEPS; i++) {
        uint8_t angle = PAN_SWEEP_ANGLES[i];

        // servo_set_pan() now slews there slowly (CCTV-style) instead of
        // snapping - see PAN_SLEW_STEP_DEG/PAN_SLEW_STEP_DELAY_MS above.
        servo_set_pan(angle);

        // Let residual mechanical vibration from the move die down before
        // we consider the camera "stopped."
        vTaskDelay(pdMS_TO_TICKS(PAN_SWEEP_ARRIVE_SETTLE_MS));

        if (on_each_angle) {
            on_each_angle(angle);
        }

        // Hold here so the capture (once wired in) has a genuinely stable
        // window to grab the frame and kick off the HTTP upload, rather
        // than immediately starting the next slew.
        vTaskDelay(pdMS_TO_TICKS(PAN_SWEEP_HOLD_MS));
    }
    // Return to center after the sweep - also a slow slew now, not a snap.
    servo_set_pan(90);
}