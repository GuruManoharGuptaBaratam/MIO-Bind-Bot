#include "tft_display.h"
#include "st7735_gfx.h"
#include "sd_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "TFT_DISPLAY";

#define TFT_PIN_SCLK   14
#define TFT_PIN_MOSI   13
#define TFT_PIN_CS     27
#define TFT_PIN_DC     26
#define TFT_PIN_RST    25
#define TFT_WIDTH      160
#define TFT_HEIGHT     128

#define ST77_CUSTOM_DARKGREY  0x4208 

static ST7735_GFX *s_gfx = nullptr;
static bool s_bt_connected = false;
static SemaphoreHandle_t s_gfx_mutex = nullptr;

#define NOTIFY_TIMEOUT_MS          2000  
#define COMMAND_HOLD_TIMEOUT_MS    3000  
static esp_timer_handle_t s_state_timeout_timer = nullptr;

typedef enum {
    TFT_STATE_IDLE_MIO = 0,         
    TFT_STATE_BT_CONNECTED,         
    TFT_STATE_BT_DISCONNECTED,      
    TFT_STATE_WAKEWORD_DETECTED,    
    TFT_STATE_COUNTDOWN_3,
    TFT_STATE_COUNTDOWN_2,
    TFT_STATE_COUNTDOWN_1,
    TFT_STATE_LISTENING_COMMAND,
    TFT_STATE_COMMAND_UNRECOGNIZED, 
    TFT_STATE_COMMAND_TIMEOUT,      
    TFT_STATE_CMD_KANSEI,            
    TFT_STATE_CMD_KIROKU,            
    TFT_STATE_CMD_IBASHO,            
    TFT_STATE_SD_OK,                 
    TFT_STATE_SD_NOT_FOUND,          
    TFT_STATE_SD_BAD_CONFIG,         
} tft_state_t;

typedef struct {
    tft_state_t state;
    const char *label;   
    uint16_t color;
} tft_state_entry_t;

static const tft_state_entry_t kStateTable[] = {
    { TFT_STATE_IDLE_MIO,              "MIO",              ST77_WHITE  },
    { TFT_STATE_BT_CONNECTED,          "CONNECTED",        ST77_GREEN  },
    { TFT_STATE_BT_DISCONNECTED,       "DISCONNECTED",     ST77_RED    },
    { TFT_STATE_WAKEWORD_DETECTED,     "HEY MIO!",         ST77_CYAN   },
    { TFT_STATE_COUNTDOWN_3,           "SPEAK IN 3",       ST77_WHITE  },
    { TFT_STATE_COUNTDOWN_2,           "SPEAK IN 2",       ST77_WHITE  },
    { TFT_STATE_COUNTDOWN_1,           "SPEAK IN 1",       ST77_WHITE  },
    { TFT_STATE_LISTENING_COMMAND,     "LISTENING...",     ST77_CYAN   },
    { TFT_STATE_COMMAND_UNRECOGNIZED,  "RETRY",            ST77_ORANGE },
    { TFT_STATE_COMMAND_TIMEOUT,       "TIMEOUT",          ST77_ORANGE },
    { TFT_STATE_CMD_KANSEI,            "CMD: KANSEI",      ST77_GREEN  },
    { TFT_STATE_CMD_KIROKU,            "CMD: KIROKU",      ST77_GREEN  },
    { TFT_STATE_CMD_IBASHO,            "CMD: IBASHO",      ST77_GREEN  },
    { TFT_STATE_SD_OK,                 "SD: OK",           ST77_GREEN  },
    { TFT_STATE_SD_NOT_FOUND,          "SD: MISSING",      ST77_RED    },
    { TFT_STATE_SD_BAD_CONFIG,         "SD: BAD CONFIG",   ST77_ORANGE },
};
#define STATE_TABLE_LEN (sizeof(kStateTable) / sizeof(kStateTable[0]))

static void request_state(tft_state_t state);
static void state_timeout_callback(void *arg);

static void render_state(tft_state_t state)
{
    if (!s_gfx) return;

    const tft_state_entry_t *entry = nullptr;
    for (size_t i = 0; i < STATE_TABLE_LEN; i++) {
        if (kStateTable[i].state == state) {
            entry = &kStateTable[i];
            break;
        }
    }
    if (!entry) return;

    xSemaphoreTake(s_gfx_mutex, portMAX_DELAY);

    // Clear Screen completely once per state change
    s_gfx->fillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, ST77_BLACK);

    // Render text label centered cleanly on screen
    s_gfx->setTextColor(entry->color);
    s_gfx->setTextSize(2);
    
    // Simple rough centering: center scale-2 text (each character is ~12 pixels wide)
    int label_len = strlen(entry->label);
    int x_pos = (TFT_WIDTH - (label_len * 12)) / 2;
    if (x_pos < 4) x_pos = 4; // safety bound

    s_gfx->setCursor(x_pos, (TFT_HEIGHT / 2) - 8);
    s_gfx->print(entry->label);

    // Failsafe sub-text for idle state
    if (state == TFT_STATE_IDLE_MIO) {
        s_gfx->setTextColor(ST77_CUSTOM_DARKGREY);
        s_gfx->setTextSize(1);
        s_gfx->setCursor(TFT_WIDTH / 2 - 15, (TFT_HEIGHT / 2) + 16);
        s_gfx->print("SYSTEM ONLINE"); 
    }

    xSemaphoreGive(s_gfx_mutex);
    ESP_LOGI(TAG, "Display State Switched -> %s", entry->label);
}

static void state_timeout_callback(void *arg)
{
    request_state(TFT_STATE_IDLE_MIO);
}

static void request_state(tft_state_t state)
{
    if (s_state_timeout_timer && esp_timer_is_active(s_state_timeout_timer)) {
        esp_timer_stop(s_state_timeout_timer);
    }

    render_state(state);

    bool is_command_state = (state == TFT_STATE_WAKEWORD_DETECTED ||
                             state == TFT_STATE_COMMAND_UNRECOGNIZED ||
                             state == TFT_STATE_COMMAND_TIMEOUT ||
                             state == TFT_STATE_CMD_KANSEI ||
                             state == TFT_STATE_CMD_KIROKU ||
                             state == TFT_STATE_CMD_IBASHO);

    bool is_notification_state = (state == TFT_STATE_BT_CONNECTED || 
                                  state == TFT_STATE_BT_DISCONNECTED ||
                                  state == TFT_STATE_SD_OK ||
                                  state == TFT_STATE_SD_NOT_FOUND ||
                                  state == TFT_STATE_SD_BAD_CONFIG);

    if (s_state_timeout_timer) {
        if (is_command_state) {
            esp_timer_start_once(s_state_timeout_timer, COMMAND_HOLD_TIMEOUT_MS * 1000);
        } else if (is_notification_state) {
            esp_timer_start_once(s_state_timeout_timer, NOTIFY_TIMEOUT_MS * 1000);
        }
    }
}

void tft_display_init(void)
{
    static st7735_gfx_config_t cfg = {
        .pin_sclk = TFT_PIN_SCLK,
        .pin_mosi = TFT_PIN_MOSI,
        .pin_cs   = TFT_PIN_CS,
        .pin_dc   = TFT_PIN_DC,
        .pin_rst  = TFT_PIN_RST,
        .width    = TFT_WIDTH,
        .height   = TFT_HEIGHT,
        .xstart   = 0,
        .ystart   = 0,
    };
    static ST7735_GFX gfx(cfg);
    s_gfx = &gfx;

    s_gfx_mutex = xSemaphoreCreateMutex();

    const esp_timer_create_args_t timeout_timer_args = {
        .callback = &state_timeout_callback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "state_timeout",
        .skip_unhandled_events = false
    };
    esp_timer_create(&timeout_timer_args, &s_state_timeout_timer);

    if (!s_gfx->begin()) {
        ESP_LOGE(TAG, "ST7735 init failed");
        return;
    }

    render_state(TFT_STATE_IDLE_MIO);
}

void tft_display_on_bt_state(bool connected)
{
    s_bt_connected = connected;
    request_state(connected ? TFT_STATE_BT_CONNECTED : TFT_STATE_BT_DISCONNECTED);
}

void tft_display_on_sd_status(sd_boot_status_t status)
{
    switch (status) {
        case SD_BOOT_OK:         request_state(TFT_STATE_SD_OK);         break;
        case SD_BOOT_NOT_FOUND:  request_state(TFT_STATE_SD_NOT_FOUND);  break;
        case SD_BOOT_BAD_CONFIG: request_state(TFT_STATE_SD_BAD_CONFIG); break;
    }
}

void tft_display_on_ie_event(core_event_t evt)
{
    if (!s_bt_connected) return;
    switch (evt) {
        case CORE_EVT_LISTENING_WAKEWORD:                                           break; 
        case CORE_EVT_WAKEWORD_DETECTED:    request_state(TFT_STATE_WAKEWORD_DETECTED);    break;
        case CORE_EVT_COUNTDOWN_3:          request_state(TFT_STATE_COUNTDOWN_3);          break;
        case CORE_EVT_COUNTDOWN_2:          request_state(TFT_STATE_COUNTDOWN_2);          break;
        case CORE_EVT_COUNTDOWN_1:          request_state(TFT_STATE_COUNTDOWN_1);          break;
        case CORE_EVT_LISTENING_COMMAND:    request_state(TFT_STATE_LISTENING_COMMAND);    break;
        case CORE_EVT_COMMAND_UNRECOGNIZED: request_state(TFT_STATE_COMMAND_UNRECOGNIZED); break;
        case CORE_EVT_COMMAND_TIMEOUT:      request_state(TFT_STATE_COMMAND_TIMEOUT);      break;
    }
}

void tft_display_on_ie_command(core_command_t cmd)
{
    if (!s_bt_connected) return;
    switch (cmd) {
        case CORE_CMD_KANSEI: request_state(TFT_STATE_CMD_KANSEI); break;
        case CORE_CMD_KIROKU: request_state(TFT_STATE_CMD_KIROKU); break;
        case CORE_CMD_IBASHO: request_state(TFT_STATE_CMD_IBASHO); break;
    }
}