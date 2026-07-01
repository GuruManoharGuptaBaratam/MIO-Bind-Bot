#include "i2s_tx.h"
#include "driver/i2s_std.h"  // Modern ESP-IDF standard header
#include "esp_log.h"
#include "freertos/FreeRTOS.h" // <--- ADD THIS
#include "freertos/task.h"

static const char *TAG = "MIO_I2S_TX";
static i2s_chan_handle_t tx_chan = NULL; // New style channel handler

#define PIN_BCLK           GPIO_NUM_4   
#define PIN_WS             GPIO_NUM_5   
#define PIN_DATA_OUT       GPIO_NUM_18  

void init_i2s_master_tx(void) {
    ESP_LOGI(TAG, "Initializing Modern I2S Standard Master Transmitter...");

    // 1. Allocate resources for a brand-new TX channel controller channel
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, NULL));

    // 2. Setup standard structural specs matching your voice processing rates
    i2s_std_config_t tx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000), // 16kHz for ML Models
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .bclk = PIN_BCLK,
            .ws = PIN_WS,
            .dout = PIN_DATA_OUT,
            .din = I2S_GPIO_UNUSED,
            .mclk = I2S_GPIO_UNUSED
        },
    };

    // 3. Initialize and startup the system state machine
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &tx_std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
    
    ESP_LOGI(TAG, "Modern I2S Master Channel Enabled cleanly.");
}

void stream_audio_over_i2s(const uint8_t *pcm_data, size_t len) {
    if (tx_chan == NULL) return;
    
    size_t bytes_written = 0;
    // Uses the updated modern channel write mechanism API
    i2s_channel_write(tx_chan, pcm_data, len, &bytes_written, portMAX_DELAY);
}