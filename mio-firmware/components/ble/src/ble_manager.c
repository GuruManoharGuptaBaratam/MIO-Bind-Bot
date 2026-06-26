#include "ble_manager.h"

#include "esp_log.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"

static const char *TAG = "MIO_BLE";

static void ble_gap_callback(
    esp_gap_ble_cb_event_t event,
    esp_ble_gap_cb_param_t *param)
{
    switch(event)
    {
        case ESP_GAP_BLE_SCAN_RESULT_EVT:

            if(param->scan_rst.search_evt ==
               ESP_GAP_SEARCH_INQ_RES_EVT)
            {
                ESP_LOGI(
                    TAG,
                    "BLE Device Found"
                );
            }

            break;

        default:
            break;
    }
}

void ble_scan_start(void)
{
    ESP_ERROR_CHECK(
        esp_ble_gap_register_callback(
            ble_gap_callback
        )
    );

    ESP_LOGI(
        TAG,
        "BLE Scanner Ready"
    );
}