#include "camera_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "camera_driver";

// AI-Thinker ESP32-CAM pin map (TY-OV3660 module pinout).
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27
#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

#define CAM_FRAME_SIZE   FRAMESIZE_CIF
#define CAM_JPEG_QUALITY 12
#define CAM_FB_COUNT     1
#define CAM_WARMUP_FRAMES 5

static bool s_initialized = false;

esp_err_t camera_driver_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    camera_config_t config = {
        .pin_pwdn     = CAM_PIN_PWDN,
        .pin_reset    = CAM_PIN_RESET,
        .pin_xclk     = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7       = CAM_PIN_D7,
        .pin_d6       = CAM_PIN_D6,
        .pin_d5       = CAM_PIN_D5,
        .pin_d4       = CAM_PIN_D4,
        .pin_d3       = CAM_PIN_D3,
        .pin_d2       = CAM_PIN_D2,
        .pin_d1       = CAM_PIN_D1,
        .pin_d0       = CAM_PIN_D0,
        .pin_vsync    = CAM_PIN_VSYNC,
        .pin_href     = CAM_PIN_HREF,
        .pin_pclk     = CAM_PIN_PCLK,

        // Lower XCLK frequency to 10MHz to fix missing JPEG SOI markers on OV3660
        .xclk_freq_hz = 10000000,
        .ledc_timer   = LEDC_TIMER_1,
        .ledc_channel = LEDC_CHANNEL_2,

        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = CAM_FRAME_SIZE,
        .jpeg_quality = CAM_JPEG_QUALITY,
        .fb_count     = CAM_FB_COUNT,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: %s", esp_err_to_name(err));
        return err;
    }

    // Give the OV3660 sensor extra time to settle internal power registers
    vTaskDelay(pdMS_TO_TICKS(200));

    for (int i = 0; i < CAM_WARMUP_FRAMES; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            esp_camera_fb_return(fb);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    s_initialized = true;
    ESP_LOGI(TAG, "camera ready (CIF, JPEG, fb_count=%d, %d warm-up frames discarded)",
              CAM_FB_COUNT, CAM_WARMUP_FRAMES);
    return ESP_OK;
}

camera_fb_t *camera_driver_capture(void)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "call camera_driver_init() first");
        return NULL;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGW(TAG, "frame capture failed");
        return NULL;
    }

    if (fb->len < 2 || fb->buf[0] != 0xFF || fb->buf[1] != 0xD8) {
        ESP_LOGW(TAG, "dropped corrupt frame (missing SOI, len=%u)", (unsigned)fb->len);
        esp_camera_fb_return(fb);
        return NULL;
    }

    return fb;
}

void camera_driver_return(camera_fb_t *fb)
{
    if (fb) {
        esp_camera_fb_return(fb);
    }
}

int camera_driver_get_width(void)
{
    switch (CAM_FRAME_SIZE) {
        case FRAMESIZE_CIF: return 400;
        default: return 400;
    }
}

int camera_driver_get_height(void)
{
    switch (CAM_FRAME_SIZE) {
        case FRAMESIZE_CIF: return 296;
        default: return 296;
    }
}