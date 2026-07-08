#include "ble_manager.h"
#include <stdio.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "nvs_flash.h"
#include "esp_gap_bt_api.h"
#include "hfp_manager.h"
#include <string.h>
#include "hfp_manager.h"
#include "core_uart_receiver.h"
#include "tft_display.h"

static const char *TAG = "MIO_BT";

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
        case ESP_BT_GAP_PIN_REQ_EVT:
            ESP_LOGI(TAG, "PIN REQUEST");
            break;

        case ESP_BT_GAP_CFM_REQ_EVT:
            ESP_LOGI(TAG, "CONFIRMATION REQUEST");
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

    ESP_LOGI(TAG, "MIO Discoverable");
    ESP_ERROR_CHECK(
    esp_bt_gap_start_discovery(
        ESP_BT_INQ_MODE_GENERAL_INQUIRY,
        10,
        0
    )
    );

    ESP_LOGI(TAG, "Started Bluetooth Scan");

    // Display must be up before any inference-link packets can be shown.
    tft_display_init();

    // Independent peripheral from BT/HFP — order relative to hfp_init()
    // doesn't matter. Routes every inference-engine command/event straight
    // to the display.
    core_uart_receiver_set_callbacks(tft_display_on_ie_command, tft_display_on_ie_event);
    core_uart_receiver_init();

    hfp_init();
}