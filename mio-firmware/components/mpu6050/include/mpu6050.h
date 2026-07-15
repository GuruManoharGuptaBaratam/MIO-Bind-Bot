#pragma once
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Wiring: SDA -> GPIO19, SCL -> GPIO22, VCC -> 3.3V, GND -> GND,
// AD0 -> GND (sets I2C address to 0x68).
esp_err_t mpu6050_init(void);

// Averages a short burst of accelerometer samples (~130ms) and returns
// pitch in centidegrees (pitch_deg * 100) - matches the CAM trigger
// packet's pitch field directly.
//
// This is a gravity-vector reading, not gyro-fused, so it's only accurate
// when the wearer is roughly still - which is the case at trigger time.
// Which raw axis maps to "forward" pitch depends on how the board sits
// relative to the CAM's tilt axis - verify with a physical tilt test
// before trusting the sign/magnitude.
esp_err_t mpu6050_read_pitch_centideg(int16_t *out_pitch_centideg);

#ifdef __cplusplus
}
#endif