#include "audio_feedback.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

static const char *TAG = "audio_fb";

#define ADPCM_READ_CHUNK_BYTES  128   
#define PCM_OUT_CHUNK_BYTES     (ADPCM_READ_CHUNK_BYTES * 4) 
#define FEEDBACK_RB_BYTES       32768 // Increased ring buffer to withstand TFT traffic  
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
static volatile bool    s_wideband_active = false; 

static volatile bool       s_is_looping_active = false;
static volatile sound_id_t s_current_loop_id = SND_COUNT;

#define SND_ID_STREAMING ((sound_id_t)0xFE)

static void drain_ringbuf(void) {
    size_t sz;
    void *item;
    while ((item = xRingbufferReceiveUpTo(s_rb, &sz, 0, FEEDBACK_RB_BYTES))) {
        vRingbufferReturnItem(s_rb, item);
    }
}

bool audio_feedback_push_direct_pcm(const uint8_t *pcm_data, size_t len) {
    if (!s_rb || !pcm_data || len == 0) return false;

    const int16_t *src = (const int16_t *)pcm_data;
    size_t total_samples = len / 2; // 16-bit PCM = 2 bytes per sample

    uint8_t out_buf[256];
    size_t out_idx = 0;

    if (s_wideband_active) {
        // mSBC session: source is 16kHz, link consumes 16kHz. Pass through 1:1.
        size_t needed = total_samples * 2;
        size_t free_bytes = xRingbufferGetCurFreeSize(s_rb);
        if (free_bytes < needed * 2) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        for (size_t i = 0; i < total_samples; i++) {
            int16_t s = apply_gain(src[i]);
            out_buf[out_idx++] = (uint8_t)(s & 0xFF);
            out_buf[out_idx++] = (uint8_t)((s >> 8) & 0xFF);
            if (out_idx >= sizeof(out_buf)) {
                xRingbufferSend(s_rb, out_buf, out_idx, pdMS_TO_TICKS(10));
                out_idx = 0;
            }
        }
    } else {
        // CVSD session: source is 16kHz, link consumes 8kHz.
        // Decimate 2:1 by averaging pairs — same technique play_clip() already
        // uses for embedded ADPCM clips, so streamed kansei audio matches the
        // pitch/speed of every other sound on the device.
        size_t pair_count = total_samples / 2;
        size_t needed = pair_count * 2;
        size_t free_bytes = xRingbufferGetCurFreeSize(s_rb);
        if (free_bytes < needed * 2) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        for (size_t i = 0; i + 1 < total_samples; i += 2) {
            int16_t s_ds = apply_gain((int16_t)(((int32_t)src[i] + (int32_t)src[i + 1]) / 2));
            out_buf[out_idx++] = (uint8_t)(s_ds & 0xFF);
            out_buf[out_idx++] = (uint8_t)((s_ds >> 8) & 0xFF);
            if (out_idx >= sizeof(out_buf)) {
                xRingbufferSend(s_rb, out_buf, out_idx, pdMS_TO_TICKS(10));
                out_idx = 0;
            }
        }
        // total_samples is guaranteed even: UART_AUDIO_CHUNK_SIZE = 320 bytes
        // = 160 samples per call, always even, so no leftover odd sample.
    }

    if (out_idx > 0) {
        xRingbufferSend(s_rb, out_buf, out_idx, pdMS_TO_TICKS(10));
    }

    return true;
}

void audio_feedback_direct_stream_start(void) {
    s_active_id = SND_ID_STREAMING;
    s_abort_current = false;
    ESP_LOGI(TAG, "Direct audio playback started");
}

void audio_feedback_direct_stream_end(void) {
    // FIX FOR CRASH/STATIC SOUND AT END OF DESC:
    // Append 640 bytes (20ms) of PCM zeros to let the Bluetooth SCO decoder/earbuds
    // exit gracefully without buffer underrun clicks or static pops.
    uint8_t zero_padding[640] = {0};
    xRingbufferSend(s_rb, zero_padding, sizeof(zero_padding), pdMS_TO_TICKS(100));

    s_active_id = SND_COUNT;
    ESP_LOGI(TAG, "Direct audio playback completed cleanly");
}

static void play_clip(sound_id_t id) {
    if (id == SND_ID_STREAMING) return;

    const sound_def_t *def = &k_sounds[id];
    const uint8_t *p = def->fl_start;
    const uint8_t *end = def->fl_end;

    ptrdiff_t clip_bytes = end - p;
    ESP_LOGI(TAG, "play_clip: id=%d flash_bytes=%td", (int)id, clip_bytes);

    if (clip_bytes < 4) {
        ESP_LOGE(TAG, "play_clip: id=%d has invalid/empty flash region", (int)id);
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
        if (sent == pdTRUE) {
            total_pushed += out_idx;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        p += n;
    }

    // Append small silence on internal clip end as well
    uint8_t zero_padding[320] = {0};
    xRingbufferSend(s_rb, zero_padding, sizeof(zero_padding), pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "play_clip: id=%d done, pushed %u PCM bytes", (int)id, (unsigned)total_pushed);
    s_active_id = SND_COUNT;

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

static inline priority_t active_priority(sound_id_t id) {
    return (id == SND_ID_STREAMING) ? PRIO_STATE : k_sounds[id].prio;
}

void audio_feedback_play(sound_id_t id) {
    if (id >= SND_COUNT || !s_trigger_q) return;

    int64_t now = esp_timer_get_time();
    if (now - s_last_played_us[id] < DEBOUNCE_MS * 1000) return;
    s_last_played_us[id] = now;

    priority_t new_prio = k_sounds[id].prio;

    if (s_active_id != SND_COUNT) {
        priority_t active_prio = active_priority(s_active_id);
        if (new_prio == PRIO_CRITICAL && active_prio != PRIO_CRITICAL) {
            s_abort_current = true;
            drain_ringbuf();
        } else if (new_prio == PRIO_INFO && s_active_id != s_current_loop_id) {
            return;
        } else if (s_active_id == s_current_loop_id) {
            s_abort_current = true;
            drain_ringbuf();
        }
    }

    s_trigger_ts_us = now;
    xQueueSend(s_trigger_q, &id, 0);
}

void audio_feedback_set_wideband(bool wideband) {
    s_wideband_active = wideband;
}

void audio_feedback_defer_until_connected(sound_id_t id) {
    if (id >= SND_COUNT) return;
    s_pending_deferred = id;
}

void audio_feedback_flush_pending(void) {
    if (s_pending_deferred == SND_COUNT) return;
    sound_id_t id = s_pending_deferred;
    s_pending_deferred = SND_COUNT;
    audio_feedback_play(id);
}

size_t audio_feedback_pull_frame(uint8_t *buf, size_t max_len) {
    if (!s_rb) return 0;
    size_t sz;
    void *item = xRingbufferReceiveUpTo(s_rb, &sz, 0, max_len);
    if (!item) return 0;

    memcpy(buf, item, sz);
    vRingbufferReturnItem(s_rb, item);
    return sz;
}