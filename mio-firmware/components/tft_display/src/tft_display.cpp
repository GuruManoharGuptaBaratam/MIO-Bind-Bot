#include "tft_display.h"
#include "st7735_gfx.h"
#include "sd_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "audio_feedback.h"

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

#define NOTIFY_TIMEOUT_MS              2000  // Display "DONE" / "SAVED" / "BUSY" for 2s then back to IDLE
#define COMMAND_HOLD_TIMEOUT_MS        3000  
#define KANSEI_PROCESSING_TIMEOUT_MS   60000 // Failsafe timeout for KANSEI processing
#define KIROKU_PROCESSING_TIMEOUT_MS  180000 // Failsafe timeout for KIROKU processing
#define REMINDER_INTERVAL_MS           3000  // Repeated reminder interval set to 3s

static esp_timer_handle_t s_state_timeout_timer = nullptr;
static esp_timer_handle_t s_reminder_timer = nullptr;
static sound_id_t s_current_processing_sound = SND_SCENE_PROCESSING;
static bool s_is_processing_active = false; // Tracks if a background processing job is running

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
    TFT_STATE_KANSEI_PROCESSING,     
    TFT_STATE_KIROKU_PROCESSING,     
    TFT_STATE_KANSEI_DONE,           
    TFT_STATE_KIROKU_SAVED,          
    TFT_STATE_SD_OK,                 
    TFT_STATE_SD_NOT_FOUND,          
    TFT_STATE_SD_BAD_CONFIG,        
    TFT_STATE_CONFIRM_KANSEI,
    TFT_STATE_CONFIRM_KIROKU,
    TFT_STATE_CONFIRM_IBASHO,
    TFT_STATE_CMD_CANCELLED,
    TFT_STATE_BTN_BUSY, 
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
    { TFT_STATE_KANSEI_PROCESSING,     "SWEEPING...",      ST77_CYAN   },
    { TFT_STATE_KIROKU_PROCESSING,     "RECORDING...",     ST77_CYAN   },
    { TFT_STATE_KANSEI_DONE,           "KANSEI DONE",      ST77_GREEN  },
    { TFT_STATE_KIROKU_SAVED,          "RECORD SAVED",     ST77_GREEN  },
    { TFT_STATE_SD_OK,                 "SD: OK",           ST77_GREEN  },
    { TFT_STATE_SD_NOT_FOUND,          "SD: MISSING",      ST77_RED    },
    { TFT_STATE_SD_BAD_CONFIG,         "SD: BAD CONFIG",   ST77_ORANGE },
    { TFT_STATE_CONFIRM_KANSEI,        "KANSEI? Y/N",      ST77_ORANGE },
    { TFT_STATE_CONFIRM_KIROKU,        "KIROKU? Y/N",      ST77_ORANGE },
    { TFT_STATE_CONFIRM_IBASHO,        "IBASHO? Y/N",      ST77_ORANGE },
    { TFT_STATE_CMD_CANCELLED,         "CANCELLED",        ST77_RED    },
    { TFT_STATE_BTN_BUSY,              "SYSTEM BUSY",      ST77_ORANGE },
};
#define STATE_TABLE_LEN (sizeof(kStateTable) / sizeof(kStateTable[0]))

static tft_state_t s_current_state = TFT_STATE_IDLE_MIO;

static void request_state(tft_state_t state);
static void state_timeout_callback(void *arg);
static void reminder_timer_callback(void *arg);

static void stop_reminder_timer(void)
{
    if (s_reminder_timer && esp_timer_is_active(s_reminder_timer)) {
        esp_timer_stop(s_reminder_timer);
    }
}

static void start_reminder_timer(void)
{
    stop_reminder_timer();
    if (s_reminder_timer) {
        esp_timer_start_periodic(s_reminder_timer, REMINDER_INTERVAL_MS * 1000);
    }
}

static void reminder_timer_callback(void *arg)
{
    if (s_current_state == TFT_STATE_KANSEI_PROCESSING || s_current_state == TFT_STATE_KIROKU_PROCESSING) {
        audio_feedback_play(s_current_processing_sound);
    } else {
        stop_reminder_timer();
    }
}

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

    s_gfx->fillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, ST77_BLACK);

    s_gfx->setTextColor(entry->color);
    s_gfx->setTextSize(2);
    
    int label_len = strlen(entry->label);
    int x_pos = (TFT_WIDTH - (label_len * 12)) / 2;
    if (x_pos < 4) x_pos = 4; 

    s_gfx->setCursor(x_pos, (TFT_HEIGHT / 2) - 8);
    s_gfx->print(entry->label);

    if (state == TFT_STATE_IDLE_MIO) {
        s_gfx->setTextColor(ST77_CUSTOM_DARKGREY);
        s_gfx->setTextSize(1);
        s_gfx->setCursor(TFT_WIDTH / 2 - 38, (TFT_HEIGHT / 2) + 16);
        s_gfx->print("SYSTEM ONLINE"); 
    }

    xSemaphoreGive(s_gfx_mutex);
    ESP_LOGI(TAG, "Display State Switched -> %s", entry->label);
}

static void state_timeout_callback(void *arg)
{
    ESP_LOGI(TAG, "State timeout reached");

    bool was_command_flow_completed = (s_current_state == TFT_STATE_KANSEI_DONE ||
                                       s_current_state == TFT_STATE_KIROKU_SAVED ||
                                       s_current_state == TFT_STATE_CMD_IBASHO);

    // If camera job hits max timeout during active processing
    if (s_current_state == TFT_STATE_KANSEI_PROCESSING || s_current_state == TFT_STATE_KIROKU_PROCESSING) {
        s_is_processing_active = false;
        stop_reminder_timer();
        audio_feedback_play(SND_CAM_JOB_ERROR);
        request_state(TFT_STATE_IDLE_MIO);
        return;
    }

    // Resume periodic loop if returning from a temporary interrupt (e.g., SYSTEM BUSY)
    if (s_is_processing_active) {
        tft_state_t target_state = (s_current_processing_sound == SND_SCENE_PROCESSING) ? 
                                   TFT_STATE_KANSEI_PROCESSING : TFT_STATE_KIROKU_PROCESSING;
        
        request_state(target_state);
        start_reminder_timer(); // Resume 3s periodic loop
        return;
    }

    // Default transition back to IDLE
    stop_reminder_timer();
    request_state(TFT_STATE_IDLE_MIO);

    if (was_command_flow_completed) {
        audio_feedback_play(SND_BACK_TO_IDLE);
    }
}

static void request_state(tft_state_t state)
{
    if (s_state_timeout_timer && esp_timer_is_active(s_state_timeout_timer)) {
        esp_timer_stop(s_state_timeout_timer);
    }

    s_current_state = state;
    render_state(state);

    bool is_command_state = (state == TFT_STATE_WAKEWORD_DETECTED ||
                             state == TFT_STATE_COMMAND_UNRECOGNIZED ||
                             state == TFT_STATE_COMMAND_TIMEOUT ||
                             state == TFT_STATE_CMD_KANSEI ||
                             state == TFT_STATE_CMD_KIROKU ||
                             state == TFT_STATE_CMD_IBASHO ||
                             state == TFT_STATE_CONFIRM_KANSEI ||
                             state == TFT_STATE_CONFIRM_KIROKU ||
                             state == TFT_STATE_CONFIRM_IBASHO);

    bool is_notification_state = (state == TFT_STATE_BT_CONNECTED || 
                                  state == TFT_STATE_BT_DISCONNECTED ||
                                  state == TFT_STATE_SD_OK ||
                                  state == TFT_STATE_SD_NOT_FOUND ||
                                  state == TFT_STATE_SD_BAD_CONFIG ||
                                  state == TFT_STATE_KANSEI_DONE ||
                                  state == TFT_STATE_KIROKU_SAVED ||
                                  state == TFT_STATE_CMD_CANCELLED ||
                                  state == TFT_STATE_BTN_BUSY);

    if (s_state_timeout_timer) {
        if (state == TFT_STATE_KANSEI_PROCESSING) {
            esp_timer_start_once(s_state_timeout_timer, KANSEI_PROCESSING_TIMEOUT_MS * 1000);
        } else if (state == TFT_STATE_KIROKU_PROCESSING) {
            esp_timer_start_once(s_state_timeout_timer, KIROKU_PROCESSING_TIMEOUT_MS * 1000);
        } else if (is_command_state) {
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

    const esp_timer_create_args_t reminder_timer_args = {
        .callback = &reminder_timer_callback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "reminder_timer",
        .skip_unhandled_events = false
    };
    esp_timer_create(&reminder_timer_args, &s_reminder_timer);

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
    audio_feedback_play(connected ? SND_WIFI_CONNECTED : SND_WIFI_DISCONNECTED);
}

void tft_display_on_sd_status(sd_boot_status_t status)
{
    switch (status) {
        case SD_BOOT_OK:
            request_state(TFT_STATE_SD_OK);
            audio_feedback_defer_until_connected(SND_SD_OK);
            break;
        case SD_BOOT_NOT_FOUND:
            request_state(TFT_STATE_SD_NOT_FOUND);
            audio_feedback_defer_until_connected(SND_SD_MISSING);
            break;
        case SD_BOOT_BAD_CONFIG:
            request_state(TFT_STATE_SD_BAD_CONFIG);
            audio_feedback_defer_until_connected(SND_SD_BAD_CONFIG);
            break;
    }
}

void tft_display_on_ie_event(core_event_t evt)
{
    if (!s_bt_connected) return;
    switch (evt) {
        case CORE_EVT_LISTENING_WAKEWORD:
            break;
        case CORE_EVT_WAKEWORD_DETECTED:
            request_state(TFT_STATE_WAKEWORD_DETECTED);
            audio_feedback_play(SND_WAKEWORD_OK);
            break;
        case CORE_EVT_COUNTDOWN_3:
            request_state(TFT_STATE_COUNTDOWN_3);
            audio_feedback_play(SND_COUNTDOWN_3);
            break;
        case CORE_EVT_COUNTDOWN_2:
            request_state(TFT_STATE_COUNTDOWN_2);
            audio_feedback_play(SND_COUNTDOWN_2);
            break;
        case CORE_EVT_COUNTDOWN_1:
            request_state(TFT_STATE_COUNTDOWN_1);
            audio_feedback_play(SND_COUNTDOWN_1);
            break;
        case CORE_EVT_LISTENING_COMMAND:
            request_state(TFT_STATE_LISTENING_COMMAND);
            break;
        case CORE_EVT_COMMAND_UNRECOGNIZED:
            request_state(TFT_STATE_COMMAND_UNRECOGNIZED);
            audio_feedback_play(SND_CMD_UNKNOWN);
            break;
        case CORE_EVT_COMMAND_TIMEOUT:
            request_state(TFT_STATE_COMMAND_TIMEOUT);
            audio_feedback_play(SND_COMMAND_TIMEOUT);
            break;
    }
}

void tft_display_on_ie_command(core_command_t cmd)
{
    if (!s_bt_connected) return;
    switch (cmd) {
        case CORE_CMD_KANSEI:
            request_state(TFT_STATE_CMD_KANSEI);
            audio_feedback_play(SND_CMD_KANSEI);
            break;
        case CORE_CMD_KIROKU:
            request_state(TFT_STATE_CMD_KIROKU);
            audio_feedback_play(SND_CMD_KIROKU);
            break;
        case CORE_CMD_IBASHO:
            request_state(TFT_STATE_CMD_IBASHO);
            audio_feedback_play(SND_CMD_IBASHO);
            break;
    }
}

void tft_display_on_command_processing(sound_id_t working_sound)
{
    if (!s_bt_connected) return;

    s_current_processing_sound = working_sound;
    s_is_processing_active = true;

    if (working_sound == SND_SCENE_PROCESSING) {
        request_state(TFT_STATE_KANSEI_PROCESSING);
    } else if (working_sound == SND_RECORDING_STARTED) {
        request_state(TFT_STATE_KIROKU_PROCESSING);
    }

    // Play initial notification immediately
    audio_feedback_play(working_sound);

    // Start repeating reminder timer (every 3 seconds)
    start_reminder_timer();
}

void tft_display_on_command_done(sound_id_t done_sound)
{
    if (!s_bt_connected) return;

    // Clear processing flag and stop repeating reminder
    s_is_processing_active = false;
    stop_reminder_timer();

    if (done_sound == SND_SCENE_DONE) {
        request_state(TFT_STATE_KANSEI_DONE);
    } else if (done_sound == SND_RECORDING_SAVED) {
        request_state(TFT_STATE_KIROKU_SAVED);
    }

    // Play completion sound
    audio_feedback_play(done_sound);
}

void tft_display_on_camera_failed(void)
{
    ESP_LOGE(TAG, "Camera Job Failed triggered");
    
    // Clear processing flag and stop reminder timer
    s_is_processing_active = false;
    stop_reminder_timer();

    // Play error prompt immediately
    audio_feedback_play(SND_CAM_JOB_ERROR);

    // Reset back to idle
    request_state(TFT_STATE_IDLE_MIO);
}

void tft_display_on_job_timeout(void)
{
    ESP_LOGW(TAG, "Dispatch job timeout reached - stopping loop and resetting state");

    // Clear active processing state and stop timer
    s_is_processing_active = false;
    stop_reminder_timer();

    // Play camera job error sound
    audio_feedback_play(SND_CAM_JOB_ERROR);

    // Reset back to IDLE display state
    request_state(TFT_STATE_IDLE_MIO);
}

void tft_display_on_button_confirm(core_command_t cmd)
{
    if (!s_bt_connected) return;
    switch (cmd) {
        case CORE_CMD_KANSEI:
            request_state(TFT_STATE_CONFIRM_KANSEI);
            audio_feedback_play(SND_CONFIRM_KANSEI);
            break;
        case CORE_CMD_KIROKU:
            request_state(TFT_STATE_CONFIRM_KIROKU);
            audio_feedback_play(SND_CONFIRM_KIROKU);
            break;
        case CORE_CMD_IBASHO:
            request_state(TFT_STATE_CONFIRM_IBASHO);
            audio_feedback_play(SND_CONFIRM_IBASHO);
            break;
    }
}

void tft_display_on_button_cancelled(core_command_t cmd, bool was_timeout)
{
    if (!s_bt_connected) return;
    request_state(TFT_STATE_CMD_CANCELLED);
    if (was_timeout) {
        audio_feedback_play(SND_CANCEL_TIMEOUT);
        return;
    }
    switch (cmd) {
        case CORE_CMD_KANSEI: audio_feedback_play(SND_CANCEL_KANSEI); break;
        case CORE_CMD_KIROKU: audio_feedback_play(SND_CANCEL_KIROKU); break;
        case CORE_CMD_IBASHO: audio_feedback_play(SND_CANCEL_IBASHO); break;
    }
}

void tft_display_on_button_busy(void)
{
    if (!s_bt_connected) return;

    // Pause periodic reminder loop during temporary interruption
    stop_reminder_timer();

    request_state(TFT_STATE_BTN_BUSY);
    audio_feedback_play(SND_BUTTON_BUSY);
}