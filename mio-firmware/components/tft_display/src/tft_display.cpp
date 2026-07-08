#include "tft_display.h"
#include "st7735_gfx.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "TFT_DISPLAY";

// Physical wiring — see project pin map.
#define TFT_PIN_SCLK   14
#define TFT_PIN_MOSI   13
#define TFT_PIN_CS     27
#define TFT_PIN_DC     26
#define TFT_PIN_RST    25
#define TFT_WIDTH      160
#define TFT_HEIGHT     128

static ST7735_GFX *s_gfx = nullptr;
static bool s_bt_connected = false;

// Two different FreeRTOS contexts can end up drawing to the same SPI panel
static SemaphoreHandle_t s_gfx_mutex = nullptr;

// True only while the moving-eye idle screen is rendering frames
static volatile bool s_idle_active = false;

// Auto-timeout configurations
#define NOTIFY_TIMEOUT_MS          2000  // For standard alerts (e.g. Bluetooth changes)
#define COMMAND_HOLD_TIMEOUT_MS    3000  // For critical interactions/commands as requested
static esp_timer_handle_t s_state_timeout_timer = nullptr;

// ── Every state the display can show ────────────────────────────────────────
typedef enum {
    TFT_STATE_IDLE_MIO = 0,         // Moving eyes baseline idle state
    TFT_STATE_BT_CONNECTED,         // Transient — shows symbol then drops to eyes
    TFT_STATE_BT_DISCONNECTED,      // Transient — shows symbol then drops to eyes
    TFT_STATE_WAKEWORD_DETECTED,    // Transient (3s hold) — drops to eyes
    TFT_STATE_COUNTDOWN_3,
    TFT_STATE_COUNTDOWN_2,
    TFT_STATE_COUNTDOWN_1,
    TFT_STATE_LISTENING_COMMAND,
    TFT_STATE_COMMAND_UNRECOGNIZED, // Transient (3s hold) — drops to eyes
    TFT_STATE_COMMAND_TIMEOUT,      // Transient (3s hold) — drops to eyes
    TFT_STATE_CMD_KANSEI,           // Transient (3s hold) — drops to eyes
    TFT_STATE_CMD_KIROKU,           // Transient (3s hold) — drops to eyes
    TFT_STATE_CMD_IBASHO,           // Transient (3s hold) — drops to eyes
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
};
#define STATE_TABLE_LEN (sizeof(kStateTable) / sizeof(kStateTable[0]))

// Forward declarations
static void request_state(tft_state_t state);
static void state_timeout_callback(void *arg);

// Custom vector drawing for crisp, professional Bluetooth symbols
static void draw_bluetooth_symbol(int cx, int cy, uint16_t color, bool connected)
{
    // Draw classic sharp Bluetooth icon outline
    s_gfx->drawLine(cx, cy - 20, cx, cy + 20, color);
    s_gfx->drawLine(cx, cy - 20, cx + 10, cy - 10, color);
    s_gfx->drawLine(cx + 10, cy - 10, cx - 10, cy + 10, color);
    s_gfx->drawLine(cx - 10, cy + 10, cx + 10, cy + 10, color);
    s_gfx->drawLine(cx + 10, cy + 10, cx, cy + 20, color);

    if (!connected) {
        // Draw a prominent striking slash across the icon to symbolize disconnected state
        s_gfx->drawLine(cx - 18, cy - 15, cx + 18, cy + 15, ST77_RED);
        s_gfx->drawLine(cx - 18, cy - 14, cx + 18, cy + 16, ST77_RED); // Thicken line slightly
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
    if (!entry) {
        ESP_LOGW(TAG, "No table entry for state %d — skipping render", (int)state);
        return;
    }

    xSemaphoreTake(s_gfx_mutex, portMAX_DELAY);

    // Clear Screen
    s_gfx->fillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, ST77_BLACK);

    if (state == TFT_STATE_IDLE_MIO) {
        // Position MIO branding text at bottom edge
        s_gfx->setTextColor(entry->color);
        s_gfx->setTextSize(1);
        s_gfx->setCursor(TFT_WIDTH / 2 - 12, TFT_HEIGHT - 14);
        s_gfx->print(entry->label);
    } 
    else if (state == TFT_STATE_BT_CONNECTED || state == TFT_STATE_BT_DISCONNECTED) {
        // Dedicated center graphic layout for Bluetooth status changes
        bool is_conn = (state == TFT_STATE_BT_CONNECTED);
        draw_bluetooth_symbol(TFT_WIDTH / 2, TFT_HEIGHT / 2 - 15, entry->color, is_conn);
        
        s_gfx->setTextColor(entry->color);
        s_gfx->setTextSize(1);
        // Center text string under symbol footprint
        int text_offset = (state == TFT_STATE_BT_CONNECTED) ? 27 : 36;
        s_gfx->setCursor(TFT_WIDTH / 2 - text_offset, TFT_HEIGHT / 2 + 18);
        s_gfx->print(entry->label);
    } 
    else {
        // Text alignment layout for general operational labels
        s_gfx->setTextColor(entry->color);
        s_gfx->setTextSize(2);
        s_gfx->setCursor(4, TFT_HEIGHT / 2 - 8);
        s_gfx->print(entry->label);
    }

    xSemaphoreGive(s_gfx_mutex);

    ESP_LOGI(TAG, "Display -> %s", entry->label);
}

// ── Idle eye animation ───────────────────────────────────────────────────
#define EYE_W        30
#define EYE_H        40
#define EYE_Y        30                          
#define EYE_L_X      35                           
#define EYE_R_X      95                           
#define PUPIL_W      10
#define PUPIL_H_OPEN 16
#define PUPIL_H_BLINK 3

#define EYES_CLEAR_X   20
#define EYES_CLEAR_Y   25
#define EYES_CLEAR_W   120
#define EYES_CLEAR_H   50

typedef struct { int8_t pupil_dx; bool blink; } eye_frame_t;
static const eye_frame_t kEyeFrames[] = {
    { 0, false}, {-8, false}, { 0, false},
    { 8, false}, { 0, false}, { 0, true },
};
#define EYE_FRAME_COUNT (sizeof(kEyeFrames) / sizeof(kEyeFrames[0]))

static void draw_eye(int x, int dx, int pupil_h)
{
    s_gfx->fillRect(x, EYE_Y, EYE_W, EYE_H, ST77_WHITE);
    int pupil_x = x + (EYE_W - PUPIL_W) / 2 + dx;
    int pupil_y = EYE_Y + (EYE_H - pupil_h) / 2;
    s_gfx->fillRect(pupil_x, pupil_y, PUPIL_W, pupil_h, ST77_BLACK);
}

static void draw_idle_eyes_frame(const eye_frame_t &f)
{
    xSemaphoreTake(s_gfx_mutex, portMAX_DELAY);
    if (!s_idle_active) {
        xSemaphoreGive(s_gfx_mutex);
        return;
    }
    s_gfx->fillRect(EYES_CLEAR_X, EYES_CLEAR_Y, EYES_CLEAR_W, EYES_CLEAR_H, ST77_BLACK);
    int pupil_h = f.blink ? PUPIL_H_BLINK : PUPIL_H_OPEN;
    draw_eye(EYE_L_X, f.pupil_dx, pupil_h);
    draw_eye(EYE_R_X, f.pupil_dx, pupil_h);
    xSemaphoreGive(s_gfx_mutex);
}

static void idle_eyes_task(void *arg)
{
    size_t frame = 0;
    while (1) {
        if (s_idle_active) {
            draw_idle_eyes_frame(kEyeFrames[frame]);
            frame = (frame + 1) % EYE_FRAME_COUNT;
            vTaskDelay(pdMS_TO_TICKS(kEyeFrames[frame].blink ? 250 : 900));
        } else {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

// ── State Management Engine ──────────────────────────────────────────────
static void state_timeout_callback(void *arg)
{
    ESP_LOGI(TAG, "State timeout reached. Reverting to moving eyes baseline.");
    // Uniformly revert back to the dynamic moving eyes screen regardless of source trace
    request_state(TFT_STATE_IDLE_MIO);
}

static void request_state(tft_state_t state)
{
    // Terminate running hardware timers immediately on new state arrivals
    if (s_state_timeout_timer && esp_timer_is_active(s_state_timeout_timer)) {
        esp_timer_stop(s_state_timeout_timer);
    }

    // Toggle active eye rendering context variables relative to base view target
    if (state != TFT_STATE_IDLE_MIO) {
        s_idle_active = false;
    }

    render_state(state);

    if (state == TFT_STATE_IDLE_MIO) {
        s_idle_active = true;
    }

    // Isolate notification alerts vs processing commands for tailored timeout schedules
    bool is_command_state = (state == TFT_STATE_WAKEWORD_DETECTED ||
                             state == TFT_STATE_COMMAND_UNRECOGNIZED ||
                             state == TFT_STATE_COMMAND_TIMEOUT ||
                             state == TFT_STATE_CMD_KANSEI ||
                             state == TFT_STATE_CMD_KIROKU ||
                             state == TFT_STATE_CMD_IBASHO);

    bool is_notification_state = (state == TFT_STATE_BT_CONNECTED || 
                                  state == TFT_STATE_BT_DISCONNECTED);

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
        ESP_LOGE(TAG, "ST7735 init failed — display will stay blank");
        return;
    }

    render_state(TFT_STATE_IDLE_MIO);
    s_idle_active = true;

    xTaskCreate(idle_eyes_task, "idle_eyes", 2560, NULL, 5, NULL);
}

void tft_display_on_bt_state(bool connected)
{
    s_bt_connected = connected;
    request_state(connected ? TFT_STATE_BT_CONNECTED : TFT_STATE_BT_DISCONNECTED);
}

void tft_display_on_ie_event(core_event_t evt)
{
    if (!s_bt_connected) {
        return;
    }
    switch (evt) {
        // Dropped text rendering for listening updates; handles operations purely behind the scenes
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
    if (!s_bt_connected) {
        return;
    }
    switch (cmd) {
        case CORE_CMD_KANSEI: request_state(TFT_STATE_CMD_KANSEI); break;
        case CORE_CMD_KIROKU: request_state(TFT_STATE_CMD_KIROKU); break;
        case CORE_CMD_IBASHO: request_state(TFT_STATE_CMD_IBASHO); break;
    }
}