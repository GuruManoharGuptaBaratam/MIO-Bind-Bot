#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Wiring: GPIO13 -> MG90S #1 (pan) orange, GPIO12 -> MG90S #2 (tilt) orange.
// Standard hobby servo PWM: 50Hz, ~500-2500us pulse width mapped to 0-180 deg.
void servo_control_init(void);

// Moves a servo to an absolute angle, clamped internally to a safe range
// (not the full 0-180) to avoid stalling the MG90S against its mechanical
// end-stops - narrow that range further once you've bench-tested your
// specific servos' real limits.
void servo_set_pan(uint8_t angle_deg);
void servo_set_tilt(uint8_t angle_deg);

// Applies the live IMU pitch reading (pitch_centideg = pitch_deg * 100,
// same units as the trigger packet) as a correction on top of the fixed
// "standard view" base tilt angle. Sign of the correction (add vs subtract)
// is a placeholder until verified against your MPU's actual mounting
// orientation - see the comment in servo_control.c.
void servo_apply_tilt_correction(int16_t pitch_centideg);

// Sweeps pan through a fixed set of standard-view angles, settling briefly
// at each one, calling on_each_angle(angle_deg) once settled. This is the
// shared motion for both kansei and kiroku - what happens at each angle
// (HTTP capture vs video frame) is the caller's job via the callback.
typedef void (*servo_sweep_cb_t)(uint8_t angle_deg);
void servo_pan_sweep(servo_sweep_cb_t on_each_angle);

#ifdef __cplusplus
}
#endif