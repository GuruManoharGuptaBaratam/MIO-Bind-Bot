#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "nvs_flash.h"
#include "esp_gap_bt_api.h"
#include "hfp_manager.h"
#include "core_uart_receiver.h"
#include "tft_display.h"
#include "sd_config.h"
#include "esp_timer.h"
#include "job_dispatcher.h"
#include "audio_feedback.h"

static const char *TAG = "MIO_BT";

// Default/fallback earbuds address — overwritten by SD/NVS config at boot if
// BT_MAC is present and valid. Kept as a fallback so the bot is still
// bench-testable with SD removed.
static esp_bd_addr_t CMF_BUDS_ADDR = {
    0x3C,
    0xB0,
    0xED,
    0xE5,
    0x2C,
    0xEF
};

static void bt_gap_callback(
    esp_bt_gap_cb_event_t event,
    esp_bt_gap_cb_param_t *param)
{
    ESP_LOGI(TAG, "GAP Event: %d", event);

    switch(event)
    {
        case ESP_BT_GAP_DISC_RES_EVT:
        {
            char bda_str[18];

            snprintf(
                bda_str,
                sizeof(bda_str),
                "%02X:%02X:%02X:%02X:%02X:%02X",
                param->disc_res.bda[0],
                param->disc_res.bda[1],
                param->disc_res.bda[2],
                param->disc_res.bda[3],
                param->disc_res.bda[4],
                param->disc_res.bda[5]
            );

            ESP_LOGI(TAG, "Device Found: %s", bda_str);
            if (memcmp(
                    param->disc_res.bda,
                    CMF_BUDS_ADDR,
                    ESP_BD_ADDR_LEN) == 0)
            {
                ESP_LOGI(TAG, "CMF BUDS FOUND");

                esp_bt_gap_cancel_discovery();
                hfp_connect(CMF_BUDS_ADDR);
            }

            for (int i = 0; i < param->disc_res.num_prop; i++)
            {
                if (param->disc_res.prop[i].type ==
                    ESP_BT_GAP_DEV_PROP_BDNAME)
                {
                    ESP_LOGI(
                        TAG,
                        "Device Name: %s",
                        (char *)param->disc_res.prop[i].val
                    );
                }
            }
            break;
        }

        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            ESP_LOGI(TAG,
                     "Discovery State Changed: %d",
                     param->disc_st_chg.state);
            break;

        case ESP_BT_GAP_PIN_REQ_EVT: {
            ESP_LOGI(TAG, "PIN REQUEST — auto-replying with legacy PIN 0000");
            esp_bt_pin_code_t pin_code = {'0','0','0','0'};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            break;
        }

        case ESP_BT_GAP_CFM_REQ_EVT:
            ESP_LOGI(TAG, "CONFIRMATION REQUEST (num_val=%lu) — auto-accepting",
                     param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;

        case ESP_BT_GAP_AUTH_CMPL_EVT:
            ESP_LOGI(TAG, "AUTH COMPLETE");
            break;

        default:
            break;
    }
}

void app_main(void)
{
    // 1. Initialize NVS Flash first (required for NVS config caching)
    ESP_ERROR_CHECK(nvs_flash_init());

    // 2. Display must be up first — everything below may show status on screen
    tft_display_init();

    // 2b. audio_feedback must exist before the SD status check below, since
    // that's the first thing in boot that triggers a feedback sound.
    if (!audio_feedback_init()) {
        ESP_LOGE(TAG, "audio_feedback_init failed -- state feedback disabled");
    }

    // 3. Read SD config
    ESP_LOGI(TAG, "Reading SD config");
    ESP_LOGI(TAG, "SD load: START, tick=%lld", esp_timer_get_time());
    esp_err_t sd_ret = sd_config_load();
    ESP_LOGI(TAG, "SD load: END, tick=%lld, ret=%s", esp_timer_get_time(), esp_err_to_name(sd_ret));

    if (sd_ret == ESP_OK) {
        // Physical SD card mounted and config loaded successfully
        if (g_sd_config.bt_mac_valid) {
            memcpy(CMF_BUDS_ADDR, g_sd_config.bt_mac, sizeof(CMF_BUDS_ADDR));
            ESP_LOGI(TAG, "CMF_BUDS_ADDR overridden from SD config");
        }
        tft_display_on_sd_status(SD_BOOT_OK);
    } 
    else if (sd_ret == ESP_ERR_NOT_FOUND) {
        // Physical SD card missing or mount failed
        if (g_sd_config.bt_mac_valid) {
            memcpy(CMF_BUDS_ADDR, g_sd_config.bt_mac, sizeof(CMF_BUDS_ADDR));
            ESP_LOGI(TAG, "SD missing: using cached NVS CMF_BUDS_ADDR");
        } else {
            ESP_LOGW(TAG, "SD card missing and no NVS cache available — using default address");
        }
        tft_display_on_sd_status(SD_BOOT_NOT_FOUND);
    } 
    else {
        // SD card mounted, but config is invalid or missing required BT_MAC
        if (g_sd_config.bt_mac_valid) {
            memcpy(CMF_BUDS_ADDR, g_sd_config.bt_mac, sizeof(CMF_BUDS_ADDR));
        }
        ESP_LOGW(TAG, "SD mounted but config invalid or missing BT_MAC");
        tft_display_on_sd_status(SD_BOOT_BAD_CONFIG);
    }

    // 4. Initialize Bluetooth Stack
    ESP_LOGI(TAG, "Starting Bluetooth");

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_LOGI(TAG, "Controller initialized");

    esp_err_t ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    ESP_LOGI(TAG, "Enable returned: %s", esp_err_to_name(ret));

    if (ret != ESP_OK) {
        return;
    }

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_LOGI(TAG, "Bluetooth Ready");

    ESP_ERROR_CHECK(esp_bt_gap_register_callback(bt_gap_callback));
    ESP_ERROR_CHECK(esp_bt_gap_set_device_name("MIO"));
    ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE));

    // 5. Initialize HFP Profile before starting discovery
    hfp_init();

    ESP_LOGI(TAG, "MIO Discoverable");
    ESP_ERROR_CHECK(
        esp_bt_gap_start_discovery(
            ESP_BT_INQ_MODE_GENERAL_INQUIRY,
            10,
            0
        )
    );

    ESP_LOGI(TAG, "Started Bluetooth Scan");

    // 6. Initialize Core UART Receiver & Job Dispatcher
    job_dispatcher_init();
    core_uart_receiver_set_callbacks(job_dispatcher_on_command, job_dispatcher_on_event);
    core_uart_receiver_init();
}