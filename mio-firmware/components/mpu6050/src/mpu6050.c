#include "mpu6050.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MPU6050";

#define MPU6050_I2C_NUM       I2C_NUM_0
#define MPU6050_SDA_PIN       19
#define MPU6050_SCL_PIN       22
#define MPU6050_I2C_FREQ_HZ   400000
#define MPU6050_ADDR          0x68

#define REG_PWR_MGMT_1    0x6B
#define REG_ACCEL_XOUT_H  0x3B

#define PITCH_SAMPLE_COUNT     16
#define PITCH_SAMPLE_DELAY_MS  8   // ~130ms total - keeps trigger latency reasonable

static esp_err_t mpu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(MPU6050_I2C_NUM, MPU6050_ADDR, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

static esp_err_t mpu_read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t reg = REG_ACCEL_XOUT_H;
    uint8_t data[6];
    esp_err_t err = i2c_master_write_read_device(MPU6050_I2C_NUM, MPU6050_ADDR,
                                                  &reg, 1, data, sizeof(data),
                                                  pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        return err;
    }
    *ax = (int16_t)((data[0] << 8) | data[1]);
    *ay = (int16_t)((data[2] << 8) | data[3]);
    *az = (int16_t)((data[4] << 8) | data[5]);
    return ESP_OK;
}

esp_err_t mpu6050_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MPU6050_SDA_PIN,
        .scl_io_num = MPU6050_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MPU6050_I2C_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(MPU6050_I2C_NUM, &conf);
    if (err != ESP_OK) return err;

    err = i2c_driver_install(MPU6050_I2C_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) return err;

    // Chip boots in sleep mode - wake it up.
    err = mpu_write_reg(REG_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 not responding on I2C (SDA=%d SCL=%d)", MPU6050_SDA_PIN, MPU6050_SCL_PIN);
        return err;
    }

    ESP_LOGI(TAG, "MPU6050 ready (SDA=%d SCL=%d)", MPU6050_SDA_PIN, MPU6050_SCL_PIN);
    return ESP_OK;
}

esp_err_t mpu6050_read_pitch_centideg(int16_t *out_pitch_centideg)
{
    double sum_pitch = 0.0;
    int good_samples = 0;

    for (int i = 0; i < PITCH_SAMPLE_COUNT; i++) {
        int16_t ax, ay, az;
        if (mpu_read_accel_raw(&ax, &ay, &az) == ESP_OK) {
            double pitch = atan2((double)ax, sqrt((double)ay * ay + (double)az * az)) * 180.0 / M_PI;
            sum_pitch += pitch;
            good_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(PITCH_SAMPLE_DELAY_MS));
    }

    if (good_samples == 0) {
        ESP_LOGW(TAG, "No valid accelerometer samples - IMU unreachable?");
        return ESP_FAIL;
    }

    double avg_pitch = sum_pitch / good_samples;
    *out_pitch_centideg = (int16_t)(avg_pitch * 100.0);

    ESP_LOGI(TAG, "Pitch: %.2f deg (%d/%d good samples)", avg_pitch, good_samples, PITCH_SAMPLE_COUNT);
    return ESP_OK;
}