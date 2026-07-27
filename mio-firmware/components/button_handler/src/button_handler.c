#include "button_handler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "job_dispatcher.h"   // job_dispatcher_on_command(), job_dispatcher_is_busy()
#include "tft_display.h"

static const char *TAG = "BTN";

#define CLICK_WINDOW_MS          400
#define LONGPRESS_MIN_MS         2500
#define LONGPRESS_MAX_MS         3500
#define DEBOUNCE_MS              40

// Updated confirmation windows:
// Gives 3 seconds overall to respond, and 2.5 seconds to register a 2nd click ("NO")
#define CONFIRM_TIMEOUT_MS       6000
#define CONFIRM_CLICK_WINDOW_MS  2000

static TaskHandle_t s_btn_task = NULL;

static void IRAM_ATTR btn_isr(void *arg) {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_btn_task, &woken);
    if (woken) portYIELD_FROM_ISR();
}

// Blocks for one debounced press+release. timeout_ms=0 blocks forever.
// Returns duration in ms, -1 = noise, -2 = timeout with no press.
static int wait_for_press(uint32_t timeout_ms) {
    uint32_t got = ulTaskNotifyTake(pdTRUE,
                        timeout_ms ? pdMS_TO_TICKS(timeout_ms) : portMAX_DELAY);
    if (got == 0) return -2;

    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
    if (gpio_get_level(BUTTON_GPIO_PIN) != 0) return -1;

    int64_t t_down = esp_timer_get_time();
    while (gpio_get_level(BUTTON_GPIO_PIN) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    int64_t t_up = esp_timer_get_time();
    return (int)((t_up - t_down) / 1000);
}

// Shows Y/N confirm prompt, waits for the reply, and dispatches on YES.
// Re-checks job_dispatcher_is_busy() right before dispatch too, in case a
// voice command started mid-confirmation window -- avoids silently
// swallowing a confirmed "yes" if job_dispatcher_on_command() would have
// dropped it anyway.
static void enter_confirm(core_command_t cmd) {
    tft_display_on_button_confirm(cmd);

    int first = wait_for_press(CONFIRM_TIMEOUT_MS);
    if (first < 0) {  // -2 timeout, or -1 noise -- either way, no clean "yes"
        ESP_LOGI(TAG, "confirm: no response within 3s, cancelling");
        tft_display_on_button_cancelled(cmd, true);
        return;
    }

    // 1 click so far = tentative YES, 2nd click within 2.5s window = NO
    int second = wait_for_press(CONFIRM_CLICK_WINDOW_MS);
    if (second >= 0) {
        ESP_LOGI(TAG, "confirm: user said NO");
        tft_display_on_button_cancelled(cmd, false);
        return;
    }

    if (job_dispatcher_is_busy()) {
        ESP_LOGW(TAG, "confirm: user said YES but system became busy meanwhile");
        tft_display_on_button_busy();
        return;
    }

    ESP_LOGI(TAG, "confirm: user said YES, dispatching cmd=0x%02X", (int)cmd);
    job_dispatcher_on_command(cmd);   // same entry point voice commands use
}

static void button_task(void *arg) {
    while (1) {
        int dur = wait_for_press(0);
        if (dur < 0) continue;

        core_command_t candidate;
        bool matched = false;

        if (dur >= LONGPRESS_MIN_MS && dur <= LONGPRESS_MAX_MS) {
            candidate = CORE_CMD_IBASHO;
            matched = true;
        } else if (dur > LONGPRESS_MAX_MS) {
            ESP_LOGW(TAG, "hold too long, ignored");
        } else {
            int click_count = 1;
            while (1) {
                int d = wait_for_press(CLICK_WINDOW_MS);
                if (d == -2) break;
                if (d < 0) continue;
                click_count++;
            }
            if (click_count == 2) { candidate = CORE_CMD_KANSEI; matched = true; }
            else if (click_count == 3) { candidate = CORE_CMD_KIROKU; matched = true; }
            else ESP_LOGW(TAG, "%d click(s) not mapped, ignored", click_count);
        }

        if (!matched) continue;

        // Don't even show a confirm prompt if the system is already busy --
        // avoids the dead-end of the user confirming "yes" to a prompt
        // that would silently get dropped by job_dispatcher.
        if (job_dispatcher_is_busy()) {
            ESP_LOGI(TAG, "button trigger ignored, system busy");
            tft_display_on_button_busy();
            continue;
        }

        enter_confirm(candidate);
    }
}

void button_handler_init(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&cfg);

    xTaskCreate(button_task, "button_task", 4096, NULL, 10, &s_btn_task);

    gpio_install_isr_service(0);  // skip if gpio_install_isr_service() is called elsewhere already
    gpio_isr_handler_add(BUTTON_GPIO_PIN, btn_isr, NULL);

    ESP_LOGI("BTN", "Button handler initialized on GPIO%d", BUTTON_GPIO_PIN);
}