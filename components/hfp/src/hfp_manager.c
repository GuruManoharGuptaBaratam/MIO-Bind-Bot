#include "hfp_manager.h"
#include <string.h>
#include "esp_log.h"
#include "esp_hf_ag_api.h"
#include "esp_gap_bt_api.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "MIO_HFP";

static esp_bd_addr_t g_remote_bda = {0};
static bool g_audio_started = false;

static QueueHandle_t s_audio_queue = NULL;
static TaskHandle_t s_audio_task_handle = NULL;
#define AUDIO_QUEUE_LEN 20

typedef struct {
    esp_hf_sync_conn_hdl_t handle;
    uint8_t *data;
    uint32_t len;
} mio_audio_msg_t;

static void mio_audio_processing_task(void *pvParameters)
{
    ESP_LOGI(TAG, "MIO Dedicated Audio Task Started on APP CPU");
    mio_audio_msg_t msg;

    while (1) {
        if (xQueueReceive(s_audio_queue, &msg, portMAX_DELAY) == pdTRUE) {
            if (msg.data && msg.len > 0) {
    // Dump first 10 bytes as hex to see if data changes with voice
                ESP_LOGI(TAG, "★ AUDIO ★ %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                    msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4],
                    msg.data[5], msg.data[6], msg.data[7], msg.data[8], msg.data[9]);
            }

            esp_hf_ag_outgoing_data_ready();

            if (msg.data) {
                free(msg.data);
            }
        }
    }
}

static void incoming_data_callback(const uint8_t *buf, uint32_t len)
{
    if (!s_audio_queue || !buf || len == 0) {
        return;
    }

    uint8_t *copy = malloc(len);
    if (!copy) return;
    memcpy(copy, buf, len);

    mio_audio_msg_t msg = {
        .handle = 0,
        .data = copy,
        .len = len
    };

    if (xQueueSend(s_audio_queue, &msg, 0) != pdTRUE) {
        free(copy);
    }
}

static uint32_t outgoing_data_callback(uint8_t *buf, uint32_t len)
{
    memset(buf, 0, len);
    return len;
}

static void hfp_callback(
    esp_hf_cb_event_t event,
    esp_hf_cb_param_t *param)
{
    switch(event)
    {
        case ESP_HF_PROF_STATE_EVT:
            ESP_LOGI(TAG, "HFP Profile State Event");
            break;

        case ESP_HF_CONNECTION_STATE_EVT:
            ESP_LOGI(TAG, "SLC STATE: %d", param->conn_stat.state);
            memcpy(g_remote_bda, param->conn_stat.remote_bda, ESP_BD_ADDR_LEN);

            if(param->conn_stat.state == ESP_HF_CONNECTION_STATE_SLC_CONNECTED)
            {
                ESP_LOGI(TAG, "SLC ESTABLISHED — opening audio");
                if(!g_audio_started)
                {
                    g_audio_started = true;
                    esp_err_t ret = esp_hf_ag_audio_connect(g_remote_bda);
                    ESP_LOGI(TAG, "Audio connect result: %s", esp_err_to_name(ret));
                }
            }
            else if(param->conn_stat.state == ESP_HF_CONNECTION_STATE_DISCONNECTED)
            {
                memset(g_remote_bda, 0, ESP_BD_ADDR_LEN);
                g_audio_started = false;
            }
            break;

        case ESP_HF_AUDIO_STATE_EVT:
            switch(param->audio_stat.state)
            {
                case ESP_HF_AUDIO_STATE_CONNECTING:
                    ESP_LOGI(TAG, "SCO: CONNECTING");
                    break;

                case ESP_HF_AUDIO_STATE_CONNECTED:
                    ESP_LOGI(TAG, "SCO: CONNECTED CVSD");
                    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                    esp_bt_sleep_disable();
                    break;

                case ESP_HF_AUDIO_STATE_CONNECTED_MSBC:
                    ESP_LOGI(TAG, "SCO: CONNECTED mSBC");
                    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                    esp_bt_sleep_disable();
                    break;

                case ESP_HF_AUDIO_STATE_DISCONNECTED:
                    ESP_LOGI(TAG, "SCO: DISCONNECTED — attempting reconnect");
                    g_audio_started = false;
                    vTaskDelay(pdMS_TO_TICKS(500));
                    if (memcmp(g_remote_bda, (esp_bd_addr_t){0}, ESP_BD_ADDR_LEN) != 0) {
                        g_audio_started = true;
                        esp_err_t r = esp_hf_ag_audio_connect(g_remote_bda);
                        ESP_LOGI(TAG, "SCO reconnect attempt: %s", esp_err_to_name(r));
                    }
                    break;
            }
            break;

        case ESP_HF_CIND_RESPONSE_EVT:
            ESP_LOGI(TAG, "CIND Request From Headset");
            esp_err_t ret_cind = esp_hf_ag_cind_response(
                param->cind_rep.remote_addr,
                ESP_HF_CALL_STATUS_NO_CALLS,
                ESP_HF_CALL_SETUP_STATUS_IDLE,
                ESP_HF_NETWORK_STATE_AVAILABLE,
                5,
                ESP_HF_ROAMING_STATUS_INACTIVE,
                5,
                ESP_HF_CALL_HELD_STATUS_NONE
            );
            ESP_LOGI(TAG, "CIND Response Result: %s", esp_err_to_name(ret_cind));
            break;

        case ESP_HF_VOLUME_CONTROL_EVT:
            ESP_LOGI(TAG, "Volume Change: type=%d volume=%d",
                     param->volume_control.type, param->volume_control.volume);
            break;

        case ESP_HF_UNAT_RESPONSE_EVT: {
            const char *cmd = param->unat_rep.unat;
            ESP_LOGI(TAG, "Unknown AT: %s", cmd);

            if (strncmp(cmd, "+XAPL=", 6) == 0) {
                esp_hf_ag_unknown_at_send(g_remote_bda, "+XAPL=iPhone,7");
            } else if (strncmp(cmd, "+IPHONEACCEV", 12) == 0) {
                esp_hf_ag_unknown_at_send(g_remote_bda, "OK");
            } else if (strncmp(cmd, "+CGMI", 5) == 0) {
                esp_hf_ag_unknown_at_send(g_remote_bda, "Apple");
            } else {
                esp_hf_ag_unknown_at_send(g_remote_bda, "OK");
            }
            break;
        }

        case ESP_HF_WBS_RESPONSE_EVT:
            ESP_LOGI(TAG, "WBS RESPONSE codec=%d", param->wbs_rep.codec);
            break;

        case ESP_HF_BCS_RESPONSE_EVT:
            ESP_LOGI(TAG, "BCS RECEIVED mode=%d (Handled)", param->bcs_rep.mode);
            break;

        default:
            break;
    }
}

void hfp_init(void)
{
    ESP_LOGI(TAG, "Initializing HFP Stack with Multithread Offloading");

    esp_bt_sleep_disable();

    s_audio_queue = xQueueCreate(AUDIO_QUEUE_LEN, sizeof(mio_audio_msg_t));

    xTaskCreatePinnedToCore(
        mio_audio_processing_task,
        "mio_audio_tsk",
        4096,
        NULL,
        22,
        &s_audio_task_handle,
        1
    );

    esp_hf_ag_register_callback(hfp_callback);
    esp_hf_ag_init();
    esp_hf_ag_register_data_callback(incoming_data_callback, outgoing_data_callback);
}

void hfp_connect(esp_bd_addr_t remote_bda)
{
    esp_hf_ag_slc_connect(remote_bda);
}