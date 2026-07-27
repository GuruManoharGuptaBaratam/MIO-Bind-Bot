#include "audio_feedback.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "audio_fb";

#define ADPCM_READ_CHUNK_BYTES  128   
#define PCM_OUT_CHUNK_BYTES     (ADPCM_READ_CHUNK_BYTES * 4) 
#define FEEDBACK_RB_BYTES       8192  
#define DEBOUNCE_MS             400

#define FEEDBACK_GAIN_NUM       2     
#define FEEDBACK_GAIN_DEN       1     

static inline int16_t apply_gain(int16_t s) {
    int32_t g = ((int32_t)s * FEEDBACK_GAIN_NUM) / FEEDBACK_GAIN_DEN;
    if (g > 32767) g = 32767;
    if (g < -32768) g = -32768;
    return (int16_t)g;
}

typedef enum { PRIO_CRITICAL = 0, PRIO_STATE = 1, PRIO_INFO = 2 } priority_t;

typedef struct {
    const uint8_t   *fl_start;  
    const uint8_t   *fl_end;
    priority_t       prio;
} sound_def_t;

#define DECLARE_CLIP(sym) \
    extern const uint8_t _binary_##sym##_adpcm_start[]; \
    extern const uint8_t _binary_##sym##_adpcm_end[];

#define CLIP(sym) _binary_##sym##_adpcm_start, _binary_##sym##_adpcm_end

DECLARE_CLIP(sdmiss)    DECLARE_CLIP(sdbad)     DECLARE_CLIP(wififail)
DECLARE_CLIP(sdok)      DECLARE_CLIP(wwok)      DECLARE_CLIP(wwretry)
DECLARE_CLIP(cnt3)      DECLARE_CLIP(cnt2)      DECLARE_CLIP(cnt1)    
DECLARE_CLIP(kansei)    DECLARE_CLIP(kiroku)    DECLARE_CLIP(ibasho)    
DECLARE_CLIP(cmdunk)    DECLARE_CLIP(cmdtout)   DECLARE_CLIP(idle)      
DECLARE_CLIP(wificonn)  DECLARE_CLIP(wifidisc)  DECLARE_CLIP(scenepr)   
DECLARE_CLIP(scenedn)   DECLARE_CLIP(recstart)  DECLARE_CLIP(recsave)
DECLARE_CLIP(confkan)   DECLARE_CLIP(confkir)   DECLARE_CLIP(confiba)
DECLARE_CLIP(cnlkan)    DECLARE_CLIP(cnlkir)    DECLARE_CLIP(cnliba)
DECLARE_CLIP(cnltout)   DECLARE_CLIP(btnbusy)

/* Declarations for generated ADPCM files */
DECLARE_CLIP(procbg)    DECLARE_CLIP(camerr)

static const sound_def_t k_sounds[SND_COUNT] = {
    [SND_SD_MISSING]         = { CLIP(sdmiss),   PRIO_CRITICAL },
    [SND_SD_BAD_CONFIG]      = { CLIP(sdbad),    PRIO_CRITICAL },
    [SND_WIFI_FAILED]        = { CLIP(wififail), PRIO_CRITICAL },

    [SND_SD_OK]               = { CLIP(sdok),     PRIO_STATE },
    [SND_WAKEWORD_OK]        = { CLIP(wwok),     PRIO_STATE },
    [SND_WAKEWORD_RETRY]     = { CLIP(wwretry),  PRIO_STATE },
    [SND_COUNTDOWN_3]         = { CLIP(cnt3),     PRIO_STATE },
    [SND_COUNTDOWN_2]         = { CLIP(cnt2),     PRIO_STATE },
    [SND_COUNTDOWN_1]         = { CLIP(cnt1),     PRIO_STATE },
    [SND_CMD_KANSEI]         = { CLIP(kansei),   PRIO_STATE },
    [SND_CMD_KIROKU]         = { CLIP(kiroku),   PRIO_STATE },
    [SND_CMD_IBASHO]         = { CLIP(ibasho),   PRIO_STATE },
    [SND_CMD_UNKNOWN]        = { CLIP(cmdunk),   PRIO_STATE },
    [SND_COMMAND_TIMEOUT]    = { CLIP(cmdtout),  PRIO_STATE },
    [SND_BACK_TO_IDLE]       = { CLIP(idle),     PRIO_STATE },

    [SND_WIFI_CONNECTED]     = { CLIP(wificonn), PRIO_INFO },
    [SND_WIFI_DISCONNECTED]  = { CLIP(wifidisc), PRIO_INFO },
    [SND_SCENE_PROCESSING]   = { CLIP(scenepr),  PRIO_INFO },
    [SND_SCENE_DONE]         = { CLIP(scenedn),  PRIO_INFO },
    [SND_RECORDING_STARTED]  = { CLIP(recstart), PRIO_INFO },
    [SND_RECORDING_SAVED]    = { CLIP(recsave),  PRIO_INFO },
    [SND_CONFIRM_KANSEI]     = { CLIP(confkan),  PRIO_STATE },
    [SND_CONFIRM_KIROKU]     = { CLIP(confkir),  PRIO_STATE },
    [SND_CONFIRM_IBASHO]     = { CLIP(confiba),  PRIO_STATE },
    [SND_CANCEL_KANSEI]      = { CLIP(cnlkan),   PRIO_STATE },
    [SND_CANCEL_KIROKU]      = { CLIP(cnlkir),   PRIO_STATE },
    [SND_CANCEL_IBASHO]      = { CLIP(cnliba),   PRIO_STATE },
    [SND_CANCEL_TIMEOUT]     = { CLIP(cnltout),  PRIO_STATE },
    [SND_BUTTON_BUSY]        = { CLIP(btnbusy),  PRIO_STATE },
    [SND_CAM_JOB_ERROR]      = { CLIP(camerr),   PRIO_STATE },
};

static const int8_t k_index_table[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8
};
static const int16_t k_step_table[89] = {
    7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,66,73,
    80,88,97,107,118,130,143,157,173,190,209,230,253,279,307,337,371,408,
    449,494,544,598,658,724,796,876,963,1060,1166,1282,1411,1552,1707,1878,
    2066,2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,5894,6484,7132,
    7845,8630,9493,10442,11487,12635,13899,15289,16818,18500,20350,22385,
    24623,27086,29794,32767
};

static inline int16_t adpcm_decode_nibble(uint8_t nibble, int32_t *predictor, int32_t *step_idx) {
    int32_t step = k_step_table[*step_idx];
    int32_t diff = step >> 3;
    if (nibble & 4) diff += step;
    if (nibble & 2) diff += step >> 1;
    if (nibble & 1) diff += step >> 2;
    if (nibble & 8) *predictor -= diff; else *predictor += diff;
    if (*predictor > 32767) *predictor = 32767;
    if (*predictor < -32768) *predictor = -32768;
    *step_idx += k_index_table[nibble];
    if (*step_idx < 0) *step_idx = 0;
    if (*step_idx > 88) *step_idx = 88;
    return (int16_t)*predictor;
}

static RingbufHandle_t s_rb = NULL;
static QueueHandle_t   s_trigger_q = NULL;
static volatile sound_id_t s_active_id = SND_COUNT;
static volatile bool   s_abort_current = false;
static int64_t         s_last_played_us[SND_COUNT] = {0};
static volatile sound_id_t s_pending_deferred = SND_COUNT;
static volatile int64_t s_trigger_ts_us = 0;
static volatile bool    s_wideband_active = true; 

/* Ambient Loop Control Variables */
static volatile bool       s_is_looping_active = false;
static volatile sound_id_t s_current_loop_id = SND_COUNT;

static void drain_ringbuf(void) {
    size_t sz;
    void *item;
    while ((item = xRingbufferReceiveUpTo(s_rb, &sz, 0, FEEDBACK_RB_BYTES))) {
        vRingbufferReturnItem(s_rb, item);
    }
}

static void play_clip(sound_id_t id) {
    const sound_def_t *def = &k_sounds[id];
    const uint8_t *p = def->fl_start;
    const uint8_t *end = def->fl_end;

    ptrdiff_t clip_bytes = end - p;
    ESP_LOGI(TAG, "play_clip: id=%d flash_bytes=%td", (int)id, clip_bytes);

    if (clip_bytes < 4) {
        ESP_LOGE(TAG, "play_clip: id=%d has invalid/empty flash region (%td bytes)", (int)id, clip_bytes);
        s_active_id = SND_COUNT;
        return;
    }

    int32_t predictor = (int16_t)(p[0] | (p[1] << 8));
    int32_t step_idx = (int8_t)p[2];
    p += 4;

    s_active_id = id;
    s_abort_current = false;

    uint8_t pcm_buf[PCM_OUT_CHUNK_BYTES];
    size_t total_pushed = 0;

    while (!s_abort_current && p < end) {
        size_t remaining = (size_t)(end - p);
        size_t n = remaining < ADPCM_READ_CHUNK_BYTES ? remaining : ADPCM_READ_CHUNK_BYTES;

        size_t out_idx = 0;
        for (size_t i = 0; i < n; i++) {
            uint8_t byte = p[i];
            
            int16_t s0 = adpcm_decode_nibble(byte & 0x0F, &predictor, &step_idx);
            int16_t s1 = adpcm_decode_nibble((byte >> 4) & 0x0F, &predictor, &step_idx);

            if (s_wideband_active) {
                s0 = apply_gain(s0);
                s1 = apply_gain(s1);

                pcm_buf[out_idx++] = (uint8_t)(s0 & 0xFF);
                pcm_buf[out_idx++] = (uint8_t)((s0 >> 8) & 0xFF);
                pcm_buf[out_idx++] = (uint8_t)(s1 & 0xFF);
                pcm_buf[out_idx++] = (uint8_t)((s1 >> 8) & 0xFF);
            } else {
                int16_t s_ds = apply_gain((int16_t)(((int32_t)s0 + (int32_t)s1) / 2));

                pcm_buf[out_idx++] = (uint8_t)(s_ds & 0xFF);
                pcm_buf[out_idx++] = (uint8_t)((s_ds >> 8) & 0xFF);
            }
        }

        BaseType_t sent = xRingbufferSend(s_rb, pcm_buf, out_idx, pdMS_TO_TICKS(200));
        if (sent != pdTRUE) {
            ESP_LOGW(TAG, "play_clip: id=%d xRingbufferSend timed out", (int)id);
        } else {
            total_pushed += out_idx;
        }
        p += n;
    }

    ESP_LOGI(TAG, "play_clip: id=%d done, pushed %u PCM bytes", (int)id, (unsigned)total_pushed);
    s_active_id = SND_COUNT;

    /* Seamless re-triggering if background looping is active and wasn't aborted */
    if (s_is_looping_active && id == s_current_loop_id && !s_abort_current) {
        xQueueSend(s_trigger_q, (void*)&s_current_loop_id, 0);
    }
}

static void audio_feedback_task(void *arg) {
    sound_id_t id;
    for (;;) {
        if (xQueueReceive(s_trigger_q, &id, portMAX_DELAY) == pdTRUE) {
            play_clip(id);
        }
    }
}

bool audio_feedback_init(void) {
    s_rb = xRingbufferCreate(FEEDBACK_RB_BYTES, RINGBUF_TYPE_BYTEBUF);
    if (!s_rb) {
        ESP_LOGE(TAG, "ring buffer alloc failed");
        return false;
    }
    s_trigger_q = xQueueCreate(8, sizeof(sound_id_t));
    if (!s_trigger_q) {
        ESP_LOGE(TAG, "trigger queue alloc failed");
        return false;
    }

    xTaskCreatePinnedToCore(audio_feedback_task, "audio_fb", 4096, NULL, 18, NULL, 1);
    return true;
}

void audio_feedback_play(sound_id_t id) {
    if (id >= SND_COUNT || !s_trigger_q) {
        ESP_LOGW(TAG, "audio_feedback_play: id=%d rejected (bad id or not init'd)", (int)id);
        return;
    }

    int64_t now = esp_timer_get_time();
    if (now - s_last_played_us[id] < DEBOUNCE_MS * 1000) {
        ESP_LOGI(TAG, "audio_feedback_play: id=%d debounced", (int)id);
        return;
    }
    s_last_played_us[id] = now;

    priority_t new_prio = k_sounds[id].prio;

    if (s_active_id != SND_COUNT) {
        priority_t active_prio = k_sounds[s_active_id].prio;
        if (new_prio == PRIO_CRITICAL && active_prio != PRIO_CRITICAL) {
            ESP_LOGI(TAG, "audio_feedback_play: id=%d preempting active id=%d", (int)id, (int)s_active_id);
            s_abort_current = true;
            drain_ringbuf();
        } else if (new_prio == PRIO_INFO && s_active_id != s_current_loop_id) {
            ESP_LOGI(TAG, "audio_feedback_play: id=%d dropped (INFO, busy with id=%d)", (int)id, (int)s_active_id);
            return;
        } else if (s_active_id == s_current_loop_id) {
            /* Stop active looping when higher priority speech arrives */
            s_abort_current = true;
            drain_ringbuf();
        }
    }

    ESP_LOGI(TAG, "audio_feedback_play: id=%d queued", (int)id);
    s_trigger_ts_us = now;
    xQueueSend(s_trigger_q, &id, 0);
}

void audio_feedback_set_wideband(bool wideband) {
    if (s_wideband_active != wideband) {
        ESP_LOGI(TAG, "audio_feedback_set_wideband: now %s", 
                 wideband ? "WIDEBAND (mSBC, 16kHz)" : "NARROWBAND (CVSD, 8kHz -- downsampling on the fly)");
    }
    s_wideband_active = wideband;
}

void audio_feedback_defer_until_connected(sound_id_t id) {
    if (id >= SND_COUNT) return;
    ESP_LOGI(TAG, "audio_feedback_defer_until_connected: id=%d (waiting for SCO)", (int)id);
    s_pending_deferred = id;
}

void audio_feedback_flush_pending(void) {
    if (s_pending_deferred == SND_COUNT) return;
    sound_id_t id = s_pending_deferred;
    s_pending_deferred = SND_COUNT;
    ESP_LOGI(TAG, "audio_feedback_flush_pending: playing deferred id=%d now that SCO is up", (int)id);
    audio_feedback_play(id);
}

size_t audio_feedback_pull_frame(uint8_t *buf, size_t max_len) {
    if (!s_rb) return 0;
    static bool s_was_flowing = false;
    size_t sz;
    void *item = xRingbufferReceiveUpTo(s_rb, &sz, 0, max_len);
    if (!item) {
        if (s_active_id != SND_COUNT && !s_is_looping_active) {
            ESP_LOGW(TAG, "pull_frame: MID_CLIP_UNDERRUN id=%d -- producer falling behind consumer", (int)s_active_id);
        } else if (s_was_flowing) {
            ESP_LOGI(TAG, "pull_frame: feedback audio drained (BT stack now getting sidetone again)");
        }
        s_was_flowing = false;
        return 0;
    }
    if (!s_was_flowing) {
        int64_t latency_ms = (esp_timer_get_time() - s_trigger_ts_us) / 1000;
        ESP_LOGI(TAG, "pull_frame: feedback audio now flowing to BT stack "
                       "(max_len=%u, got=%u, TRIGGER_TO_AUDIBLE_MS=%lld)",
                 (unsigned)max_len, (unsigned)sz, (long long)latency_ms);
        s_was_flowing = true;
    }
    memcpy(buf, item, sz);
    vRingbufferReturnItem(s_rb, item);
    return sz;
}