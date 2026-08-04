#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SND_SD_MISSING = 0,
    SND_SD_BAD_CONFIG,
    SND_WIFI_FAILED,
    SND_SD_OK,
    SND_WAKEWORD_OK,
    SND_WAKEWORD_RETRY,
    SND_COUNTDOWN_3,
    SND_COUNTDOWN_2,
    SND_COUNTDOWN_1,
    SND_CMD_KANSEI,
    SND_CMD_KIROKU,
    SND_CMD_IBASHO,
    SND_CMD_UNKNOWN,
    SND_COMMAND_TIMEOUT,
    SND_BACK_TO_IDLE,
    SND_WIFI_CONNECTED,
    SND_WIFI_DISCONNECTED,
    SND_SCENE_PROCESSING,
    SND_SCENE_DONE,
    SND_RECORDING_STARTED,
    SND_RECORDING_SAVED,
    SND_CONFIRM_KANSEI,
    SND_CONFIRM_KIROKU,
    SND_CONFIRM_IBASHO,
    SND_CANCEL_KANSEI,
    SND_CANCEL_KIROKU,
    SND_CANCEL_IBASHO,
    SND_CANCEL_TIMEOUT,
    SND_BUTTON_BUSY,

    SND_CAM_JOB_ERROR,   

    SND_COUNT
} sound_id_t;

bool audio_feedback_init(void);
void audio_feedback_play(sound_id_t id);
void audio_feedback_set_wideband(bool wideband);
void audio_feedback_defer_until_connected(sound_id_t id);
void audio_feedback_flush_pending(void);
size_t audio_feedback_pull_frame(uint8_t *buf, size_t max_len);

/* --- DIRECT LIVE PASS-THROUGH STREAMING FUNCTIONS --- */

/**
 * @brief Marks state as streaming and resets abort flags.
 */
void audio_feedback_direct_stream_start(void);

/**
 * @brief Pushes incoming PCM chunk straight into the Bluetooth RingBuffer.
 */
bool audio_feedback_push_direct_pcm(const uint8_t *pcm_data, size_t len);

/**
 * @brief Appends zero-padding to fade out audio and prevents earbud pops/crashes at stream end.
 */
void audio_feedback_direct_stream_end(void);

#ifdef __cplusplus
}
#endif