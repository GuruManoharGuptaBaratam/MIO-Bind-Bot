#include "tft_display.h"
#include "st7735_gfx.h"
#include "sd_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "audio_feedback.h"
#include <math.h>

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
static bool s_wifi_connected = false; // NEW: mirrors ESP32-CAM wifi link state, for idle status row
static bool s_sd_ok = false;          // NEW: latched from tft_display_on_sd_status(), for idle status row
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

// ---------------------------------------------------------------------------
// Visual design layer (NEW). No new tasks/timers/heap use — the processing
// "pulse" animation below is driven entirely by the reminder_timer that
// already existed and already fires every REMINDER_INTERVAL_MS.
// ---------------------------------------------------------------------------
static uint8_t s_anim_frame = 0; // 1 byte. advanced by the existing reminder timer only.

typedef enum {
    ICON_NONE = 0,
    ICON_LOGO,       // idle
    ICON_BT_ON,
    ICON_BT_OFF,
    ICON_EAR,        // wakeword
    ICON_MIC,        // listening for command
    ICON_WARN,       // unrecognized / timeout / cancelled / sd problem
    ICON_APERTURE,   // kansei (scene / camera)
    ICON_REC,        // kiroku (recording)
    ICON_PIN,        // ibasho (location)
    ICON_CHECK,      // done / saved / ok
    ICON_SD,         // sd ok
    ICON_QMARK,      // confirm y/n
    ICON_BUSY,       // system busy
} tft_icon_t;

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
    tft_icon_t icon;
} tft_state_entry_t;

static const tft_state_entry_t kStateTable[] = {
    { TFT_STATE_IDLE_MIO,              "MIO",              ST77_BLUE,   ICON_LOGO     },
    { TFT_STATE_BT_CONNECTED,          "CONNECTED",        ST77_GREEN,  ICON_BT_ON    },
    { TFT_STATE_BT_DISCONNECTED,       "DISCONNECTED",     ST77_RED,    ICON_BT_OFF   },
    { TFT_STATE_WAKEWORD_DETECTED,     "HEY MIO!",         ST77_CYAN,   ICON_EAR      },
    { TFT_STATE_COUNTDOWN_3,           "SPEAK NOW",        ST77_WHITE,  ICON_NONE     }, // ring+digit, drawn separately
    { TFT_STATE_COUNTDOWN_2,           "SPEAK NOW",        ST77_WHITE,  ICON_NONE     },
    { TFT_STATE_COUNTDOWN_1,           "SPEAK NOW",        ST77_WHITE,  ICON_NONE     },
    { TFT_STATE_LISTENING_COMMAND,     "LISTENING...",     ST77_CYAN,   ICON_MIC      },
    { TFT_STATE_COMMAND_UNRECOGNIZED,  "RETRY",            ST77_ORANGE, ICON_WARN     },
    { TFT_STATE_COMMAND_TIMEOUT,       "TIMEOUT",          ST77_ORANGE, ICON_WARN     },
    { TFT_STATE_CMD_KANSEI,            "CMD: KANSEI",      ST77_GREEN,  ICON_APERTURE },
    { TFT_STATE_CMD_KIROKU,            "CMD: KIROKU",      ST77_GREEN,  ICON_REC      },
    { TFT_STATE_CMD_IBASHO,            "CMD: IBASHO",      ST77_GREEN,  ICON_PIN      },
    { TFT_STATE_KANSEI_PROCESSING,     "SWEEPING...",      ST77_CYAN,   ICON_APERTURE },
    { TFT_STATE_KIROKU_PROCESSING,     "RECORDING...",     ST77_CYAN,   ICON_REC      },
    { TFT_STATE_KANSEI_DONE,           "KANSEI DONE",      ST77_GREEN,  ICON_CHECK    },
    { TFT_STATE_KIROKU_SAVED,          "RECORD SAVED",     ST77_GREEN,  ICON_CHECK    },
    { TFT_STATE_SD_OK,                 "SD: OK",           ST77_GREEN,  ICON_SD       },
    { TFT_STATE_SD_NOT_FOUND,          "SD: MISSING",      ST77_RED,    ICON_WARN     },
    { TFT_STATE_SD_BAD_CONFIG,         "SD: BAD CONFIG",   ST77_ORANGE, ICON_WARN     },
    { TFT_STATE_CONFIRM_KANSEI,        "KANSEI? Y/N",      ST77_ORANGE, ICON_QMARK    },
    { TFT_STATE_CONFIRM_KIROKU,        "KIROKU? Y/N",      ST77_ORANGE, ICON_QMARK    },
    { TFT_STATE_CONFIRM_IBASHO,        "IBASHO? Y/N",      ST77_ORANGE, ICON_QMARK    },
    { TFT_STATE_CMD_CANCELLED,         "CANCELLED",        ST77_RED,    ICON_WARN     },
    { TFT_STATE_BTN_BUSY,              "SYSTEM BUSY",      ST77_ORANGE, ICON_BUSY     },
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

// ---------------------------------------------------------------------------
// Icon primitives. All vector-drawn (no bitmaps) -> zero flash/RAM overhead
// beyond the tiny bit of drawing code itself. Each icon is ~15px, drawn once
// per full state redraw, so cost per state change stays negligible.
// ---------------------------------------------------------------------------
static void draw_icon(tft_icon_t icon, int16_t cx, int16_t cy, uint16_t color)
{
    switch (icon) {
        case ICON_LOGO:
            s_gfx->drawCircle(cx, cy, 15, color);
            s_gfx->drawCircle(cx, cy, 8, color);
            break;

        case ICON_BT_ON:
            s_gfx->drawCircle(cx, cy, 13, color);
            s_gfx->fillCircle(cx, cy, 3, color);
            break;

        case ICON_BT_OFF:
            s_gfx->drawCircle(cx, cy, 13, color);
            s_gfx->drawLine(cx - 9, cy - 9, cx + 9, cy + 9, color);
            s_gfx->drawLine(cx - 9, cy + 9, cx + 9, cy - 9, color);
            break;

        case ICON_EAR:
        case ICON_MIC:
            s_gfx->fillCircle(cx, cy - 5, 7, color);
            s_gfx->fillRect(cx - 7, cy - 5, 14, 9, color);
            s_gfx->drawLine(cx, cy + 6, cx, cy + 13, color);
            s_gfx->drawLine(cx - 5, cy + 13, cx + 5, cy + 13, color);
            break;

        case ICON_APERTURE: {
            s_gfx->drawCircle(cx, cy, 14, color);
            for (int i = 0; i < 6; i++) {
                float a = i * (2.0f * (float)M_PI / 6.0f);
                int16_t x1 = cx + (int16_t)(11.0f * cosf(a));
                int16_t y1 = cy + (int16_t)(11.0f * sinf(a));
                int16_t x2 = cx + (int16_t)(4.0f * cosf(a + 0.5f));
                int16_t y2 = cy + (int16_t)(4.0f * sinf(a + 0.5f));
                s_gfx->drawLine(x1, y1, x2, y2, color);
            }
            break;
        }

        case ICON_REC:
            s_gfx->drawCircle(cx, cy, 14, color);
            s_gfx->fillCircle(cx, cy, 8, color);
            break;

        case ICON_PIN:
            s_gfx->fillCircle(cx, cy - 4, 9, color);
            s_gfx->fillCircle(cx, cy - 4, 4, ST77_BLACK);
            s_gfx->fillTriangle(cx - 8, cy, cx + 8, cy, cx, cy + 14, color);
            break;

        case ICON_CHECK:
            s_gfx->drawLine(cx - 9, cy, cx - 2, cy + 8, color);
            s_gfx->drawLine(cx - 2, cy + 8, cx + 10, cy - 9, color);
            s_gfx->drawLine(cx - 9, cy + 1, cx - 2, cy + 9, color);
            s_gfx->drawLine(cx - 2, cy + 9, cx + 10, cy - 8, color);
            break;

        case ICON_WARN:
            s_gfx->drawLine(cx, cy - 14, cx - 13, cy + 11, color);
            s_gfx->drawLine(cx, cy - 14, cx + 13, cy + 11, color);
            s_gfx->drawLine(cx - 13, cy + 11, cx + 13, cy + 11, color);
            s_gfx->drawLine(cx, cy - 5, cx, cy + 3, color);
            s_gfx->fillCircle(cx, cy + 7, 1, color);
            break;

        case ICON_SD:
            s_gfx->drawRect(cx - 10, cy - 13, 20, 26, color);
            s_gfx->drawLine(cx - 10, cy - 5, cx + 10, cy - 5, color);
            break;

        case ICON_QMARK:
            s_gfx->setTextColor(color);
            s_gfx->setTextSize(3);
            s_gfx->setCursor(cx - 7, cy - 12);
            s_gfx->print("?");
            break;

        case ICON_BUSY:
            s_gfx->fillTriangle(cx - 10, cy - 13, cx + 10, cy - 13, cx, cy, color);
            s_gfx->fillTriangle(cx - 10, cy + 13, cx + 10, cy + 13, cx, cy, color);
            break;

        case ICON_NONE:
        default:
            break;
    }
}

// Corner "viewfinder" brackets + bottom accent bar. Drawn once per full
// redraw only (not animated) -> cheap, gives the display a distinct
// device-HUD identity instead of a flat OLED-style block of text.
static void draw_hud_frame(uint16_t accent_color)
{
    const uint16_t g = ST77_CUSTOM_DARKGREY;
    const int len = 8;

    s_gfx->drawLine(2, 2, 2 + len, 2, g);
    s_gfx->drawLine(2, 2, 2, 2 + len, g);

    s_gfx->drawLine(TFT_WIDTH - 3 - len, 2, TFT_WIDTH - 3, 2, g);
    s_gfx->drawLine(TFT_WIDTH - 3, 2, TFT_WIDTH - 3, 2 + len, g);

    s_gfx->drawLine(2, TFT_HEIGHT - 3, 2 + len, TFT_HEIGHT - 3, g);
    s_gfx->drawLine(2, TFT_HEIGHT - 3 - len, 2, TFT_HEIGHT - 3, g);

    s_gfx->drawLine(TFT_WIDTH - 3 - len, TFT_HEIGHT - 3, TFT_WIDTH - 3, TFT_HEIGHT - 3, g);
    s_gfx->drawLine(TFT_WIDTH - 3, TFT_HEIGHT - 3 - len, TFT_WIDTH - 3, TFT_HEIGHT - 3, g);

    s_gfx->fillRect(0, TFT_HEIGHT - 3, TFT_WIDTH, 3, accent_color);
}

// Countdown ring + big digit. Single deterministic redraw per event
// (3 -> 2 -> 1 are separate states already driven by CORE_EVT_COUNTDOWN_x),
// no extra loop needed for the "3,2,1" motion.
static void draw_countdown_ring(int16_t cx, int16_t cy, int count, uint16_t color)
{
    s_gfx->drawCircle(cx, cy, 18, color);
    s_gfx->drawCircle(cx, cy, 17, color);
    char buf[2] = { (char)('0' + count), '\0' };
    s_gfx->setTextColor(color);
    s_gfx->setTextSize(3);
    s_gfx->setCursor(cx - 8, cy - 12);
    s_gfx->print(buf);
}

// Top status row: SD / WF / BL tags, red/green per connection state.
// Only drawn on the idle screen (called from render_state when state ==
// TFT_STATE_IDLE_MIO), so it costs nothing on any other state.
static void draw_status_row(void)
{
    const uint16_t ok_color  = ST77_GREEN;
    const uint16_t bad_color = ST77_RED;
    const int16_t y = 13;

    s_gfx->setTextSize(1);

    s_gfx->setTextColor(s_sd_ok ? ok_color : bad_color);
    s_gfx->setCursor(14, y);
    s_gfx->print("SD");

    s_gfx->setTextColor(s_wifi_connected ? ok_color : bad_color);
    s_gfx->setCursor(TFT_WIDTH / 2 - 6, y);
    s_gfx->print("WF");

    s_gfx->setTextColor(s_bt_connected ? ok_color : bad_color);
    s_gfx->setCursor(TFT_WIDTH - 14 - 12, y);
    s_gfx->print("BL");
}

#define DOTS_Y   112
#define DOTS_CX  (TFT_WIDTH / 2)

// Partial-redraw progress pulse used only while KANSEI/KIROKU processing.
// This is called from reminder_timer_callback(), which already fires every
// REMINDER_INTERVAL_MS on its own -- no new timer, no new task, and the
// redraw touches a 48x8px strip only, so it stays fast even on slow SPI.
static void draw_progress_dots(uint8_t frame, uint16_t color)
{
    s_gfx->fillRect(DOTS_CX - 24, DOTS_Y - 4, 48, 8, ST77_BLACK);
    int active = (frame % 3) + 1;
    for (int i = 0; i < 3; i++) {
        int16_t dx = DOTS_CX - 16 + i * 16;
        if (i < active) {
            s_gfx->fillCircle(dx, DOTS_Y, 3, color);
        } else {
            s_gfx->drawCircle(dx, DOTS_Y, 3, color);
        }
    }
}

static void reminder_timer_callback(void *arg)
{
    if (s_current_state == TFT_STATE_KANSEI_PROCESSING || s_current_state == TFT_STATE_KIROKU_PROCESSING) {
        audio_feedback_play(s_current_processing_sound);

        const tft_state_entry_t *entry = nullptr;
        for (size_t i = 0; i < STATE_TABLE_LEN; i++) {
            if (kStateTable[i].state == s_current_state) {
                entry = &kStateTable[i];
                break;
            }
        }
        if (entry && s_gfx) {
            s_anim_frame++;
            xSemaphoreTake(s_gfx_mutex, portMAX_DELAY);
            draw_progress_dots(s_anim_frame, entry->color);
            xSemaphoreGive(s_gfx_mutex);
        }
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
    draw_hud_frame(entry->color);

    if (state == TFT_STATE_IDLE_MIO) {
        draw_status_row();
    }

    const int16_t icon_cx = TFT_WIDTH / 2;
    const int16_t icon_cy = (state == TFT_STATE_IDLE_MIO) ? 44 : 38;

    bool is_countdown = (state == TFT_STATE_COUNTDOWN_3 ||
                          state == TFT_STATE_COUNTDOWN_2 ||
                          state == TFT_STATE_COUNTDOWN_1);

    if (is_countdown) {
        int digit = (state == TFT_STATE_COUNTDOWN_3) ? 3 :
                    (state == TFT_STATE_COUNTDOWN_2) ? 2 : 1;
        draw_countdown_ring(icon_cx, icon_cy, digit, entry->color);
    } else {
        draw_icon(entry->icon, icon_cx, icon_cy, entry->color);
    }

    // Label -- auto-shrinks to size 1 for long strings so nothing clips
    // off the 160px-wide panel (SD: BAD CONFIG previously overflowed).
    s_gfx->setTextColor(entry->color);
    int label_len = strlen(entry->label);
    int text_size = (label_len > 11) ? 1 : 2;
    s_gfx->setTextSize(text_size);
    int char_w = text_size * 6;
    int x_pos = (TFT_WIDTH - (label_len * char_w)) / 2;
    if (x_pos < 2) x_pos = 2;
    s_gfx->setCursor(x_pos, 68);
    s_gfx->print(entry->label);

    if (state == TFT_STATE_IDLE_MIO) {
        s_gfx->setTextColor(ST77_CUSTOM_DARKGREY);
        s_gfx->setTextSize(1);
        s_gfx->setCursor(TFT_WIDTH / 2 - 38, 92);
        s_gfx->print("SYSTEM ONLINE"); 
    }

    if (state == TFT_STATE_KANSEI_PROCESSING || state == TFT_STATE_KIROKU_PROCESSING) {
        s_anim_frame = 0;
        draw_progress_dots(s_anim_frame, entry->color);
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
    s_sd_ok = (status == SD_BOOT_OK);

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

// NEW: mirrors ESP32-CAM wifi link state onto the idle-screen "WF" tag.
// Call this from wherever Core learns the CAM's wifi status (e.g. the
// existing UART/status relay path) -- it only affects the idle screen and
// does not touch the voice/button command state machine.
void tft_display_on_wifi_state(bool connected)
{
    s_wifi_connected = connected;
    if (s_current_state == TFT_STATE_IDLE_MIO) {
        render_state(TFT_STATE_IDLE_MIO); // direct refresh, idle has no timeout timer to disturb
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