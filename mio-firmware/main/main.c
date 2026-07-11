#include <stdio.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "nvs_flash.h"
#include "esp_gap_bt_api.h"
#include "hfp_manager.h"
#include <string.h>
#include "core_uart_receiver.h"
#include "tft_display.h"
#include "sd_config.h"
#include "esp_timer.h"

static const char *TAG = "MIO_BT";

// Default/fallback earbuds address — overwritten by SD config at boot if
// BT_MAC is present and valid. Kept as a fallback so the bot is still
// bench-testable with SD removed.
static  esp_bd_addr_t CMF_BUDS_ADDR = {
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
    ESP_ERROR_CHECK(
        nvs_flash_init()
    );

    // Display must be up first — everything below this point may need to
    // show status on screen (SD result, then BT/inference states as before).
    tft_display_init();

    // SD is read once, before anything else touches Bluetooth or the
    // inference link. Insert-before-boot only — no re-read after this.
    ESP_LOGI(TAG, "Reading SD config");
    ESP_LOGI(TAG, "SD load: START, tick=%lld", esp_timer_get_time());
    esp_err_t sd_ret = sd_config_load();
    ESP_LOGI(TAG, "SD load: END, tick=%lld, ret=%s", esp_timer_get_time(), esp_err_to_name(sd_ret));

    if (sd_ret == ESP_OK && g_sd_config.bt_mac_valid) {
        memcpy(CMF_BUDS_ADDR, g_sd_config.bt_mac, sizeof(CMF_BUDS_ADDR));
        ESP_LOGI(TAG, "CMF_BUDS_ADDR overridden from SD config");
        tft_display_on_sd_status(SD_BOOT_OK);
    } else if (sd_ret == ESP_ERR_NOT_FOUND || sd_ret == ESP_ERR_INVALID_RESPONSE
               || sd_ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SD card not detected/mountable — using fallback CMF_BUDS_ADDR");
        tft_display_on_sd_status(SD_BOOT_NOT_FOUND);
    } else {
        ESP_LOGW(TAG, "SD mounted but config invalid/missing BT_MAC — using fallback CMF_BUDS_ADDR");
        tft_display_on_sd_status(SD_BOOT_BAD_CONFIG);
    }
    // tft_display_on_sd_status(SD_BOOT_NOT_FOUND); // test one

    ESP_LOGI(TAG, "Starting Bluetooth");

    esp_bt_controller_config_t bt_cfg =
        BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(
        esp_bt_controller_init(&bt_cfg)
    );

    ESP_LOGI(TAG, "Controller initialized");

    esp_err_t ret =
        esp_bt_controller_enable(
            ESP_BT_MODE_BTDM
        );

    ESP_LOGI(TAG,
             "Enable returned: %s",
             esp_err_to_name(ret));

    if(ret != ESP_OK)
    {
        return;
    }

    ESP_ERROR_CHECK(
        esp_bluedroid_init()
    );

    ESP_ERROR_CHECK(
        esp_bluedroid_enable()
    );

    ESP_LOGI(TAG, "Bluetooth Ready");
    // ble_scan_start();

    ESP_ERROR_CHECK(
        esp_bt_gap_register_callback(
            bt_gap_callback
        )
    );

    ESP_ERROR_CHECK(
        esp_bt_gap_set_device_name(
            "MIO"
        )
    );

    ESP_ERROR_CHECK(
        esp_bt_gap_set_scan_mode(
            ESP_BT_CONNECTABLE,
            ESP_BT_GENERAL_DISCOVERABLE
        )
    );

    // hfp_init() must run before discovery starts, so the HFP profile is
    // fully registered by the time a matching device triggers hfp_connect()
    // from inside the GAP callback.
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

    // Independent peripheral from BT/HFP — order relative to hfp_init()
    // doesn't matter. Routes every inference-engine command/event straight
    // to the display.
    core_uart_receiver_set_callbacks(tft_display_on_ie_command, tft_display_on_ie_event);
    core_uart_receiver_init();
}