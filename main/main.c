#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "nvs_flash.h"
#include "esp_gap_bt_api.h"

static const char *TAG = "MIO_BT";

static void bt_gap_callback(
    esp_bt_gap_cb_event_t event,
    esp_bt_gap_cb_param_t *param)
{
    switch(event)
    {
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
}