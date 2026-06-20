#include "hfp_manager.h"

#include "esp_log.h"
#include "esp_hf_ag_api.h"

static const char *TAG = "MIO_HFP";

static void hfp_callback(
    esp_hf_cb_event_t event,
    esp_hf_cb_param_t *param)
{
    switch(event)
    {
        case ESP_HF_PROF_STATE_EVT:
        {
            ESP_LOGI(
                TAG,
                "HFP Profile State Event"
            );
            break;
        }

        case ESP_HF_CONNECTION_STATE_EVT:
        {
            ESP_LOGI(
                TAG,
                "HFP Connection State: %d, Peer: %02X:%02X:%02X:%02X:%02X:%02X",
                param->conn_stat.state,
                param->conn_stat.remote_bda[0],
                param->conn_stat.remote_bda[1],
                param->conn_stat.remote_bda[2],
                param->conn_stat.remote_bda[3],
                param->conn_stat.remote_bda[4],
                param->conn_stat.remote_bda[5]
            );
            break;
        }

        case ESP_HF_AUDIO_STATE_EVT:
        {
            ESP_LOGI(
                TAG,
                "HFP Audio State: %d",
                param->audio_stat.state
            );
            break;
        }

        case ESP_HF_CIND_RESPONSE_EVT:
        {
            ESP_LOGI(
                TAG,
                "CIND Request From Headset"
            );

            esp_err_t ret =
                esp_hf_ag_cind_response(
                    param->cind_rep.remote_addr,

                    ESP_HF_CALL_STATUS_NO_CALLS,
                    ESP_HF_CALL_SETUP_STATUS_IDLE,

                    ESP_HF_NETWORK_STATE_AVAILABLE,

                    5,

                    ESP_HF_ROAMING_STATUS_INACTIVE,

                    5,

                    ESP_HF_CALL_HELD_STATUS_NONE
                );

            ESP_LOGI(
                TAG,
                "CIND Response Result: %s",
                esp_err_to_name(ret)
            );

            break;
        }

        case ESP_HF_VOLUME_CONTROL_EVT:
        {
            ESP_LOGI(
                TAG,
                "Volume Change: type=%d volume=%d",
                param->volume_control.type,
                param->volume_control.volume
            );
            break;
        }

        case ESP_HF_NREC_RESPONSE_EVT:
        {
            ESP_LOGI(
                TAG,
                "NREC Event"
            );
            break;
        }

        case ESP_HF_CNUM_RESPONSE_EVT:
        {
            ESP_LOGI(
                TAG,
                "Subscriber Number Request"
            );
            break;
        }

        case ESP_HF_UNAT_RESPONSE_EVT:
        {
            ESP_LOGI(
                TAG,
                "Unknown AT Command"
            );
            break;
        }

        default:
        {
            ESP_LOGI(
                TAG,
                "HFP Event: %d",
                event
            );
            break;
        }
    }
}
void hfp_init(void)
{
    ESP_LOGI(TAG, "Initializing HFP");

    esp_err_t ret;

    ret = esp_hf_ag_register_callback(
        hfp_callback
    );

    ESP_LOGI(TAG,
             "Register Callback: %s",
             esp_err_to_name(ret));

    ret = esp_hf_ag_init();

    ESP_LOGI(TAG,
             "HFP Init: %s",
             esp_err_to_name(ret));
}
void hfp_connect(esp_bd_addr_t remote_bda)
{
    esp_err_t ret =
        esp_hf_ag_slc_connect(remote_bda);

    ESP_LOGI(
        TAG,
        "SLC Connect Result: %s",
        esp_err_to_name(ret)
    );
}