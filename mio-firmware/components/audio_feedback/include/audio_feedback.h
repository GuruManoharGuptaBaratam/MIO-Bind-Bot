#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------
// audio_feedback: plays short pre-rendered clips over the same SCO path
// used for sidetone/TTS, to confirm MIO's internal state to the user.
//
// FORMAT: raw 16-bit PCM, mono, 16kHz (matches outgoing_data_callback's
// existing PCM handling in hfp_manager.c -- NOT mSBC-encoded).
//
// Design rules:
//   - All clips are pre-rendered offline (see generate_sounds.py).
//   - Only one clip plays at a time. No mixing.
//   - CRITICAL preempts anything playing. STATE/INFO queue with
//     dedup/drop rules (see audio_feedback.c).
//   - outgoing_data_callback() must call audio_feedback_pull_frame()
//     FIRST on every call; only fall back to sidetone if it returns 0.
// ---------------------------------------------------------------------

typedef enum {
    // --- CRITICAL (preempts) ---
    SND_SD_MISSING = 0,
    SND_SD_BAD_CONFIG,
    SND_WIFI_FAILED,

    // --- STATE (queued, deduped) ---
    SND_SD_OK,
    SND_WAKEWORD_OK,
    SND_WAKEWORD_RETRY,
    SND_COUNTDOWN_3,             // one sound per real tick -- never a
    SND_COUNTDOWN_2,             // pre-mixed combined clip, which can only
    SND_COUNTDOWN_1,             // drift from the actual event timing
    SND_CMD_KANSEI,
    SND_CMD_KIROKU,
    SND_CMD_IBASHO,
    SND_CMD_UNKNOWN,
    SND_COMMAND_TIMEOUT,
    SND_BACK_TO_IDLE,

    // --- INFO (queued, drop-if-busy) ---
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

    SND_COUNT
} sound_id_t;

// Call once at boot, after SD card is mounted (main.c step 3, before hfp_init).
bool audio_feedback_init(void);

// Non-blocking: enqueue a feedback sound. Priority/dedup handled internally.
void audio_feedback_play(sound_id_t id);

// For triggers that fire before SCO is connected (e.g. the SD status
// check in main.c, which runs seconds before BT/SCO comes up). Calling
// audio_feedback_play() directly at that point wastes the clip: nothing
// drains the ring buffer until SCO connects, so most of the decoded
// audio gets discarded by send-timeouts, and what little survives plays
// stale and truncated the instant SCO finally connects. Use this instead
// -- it just remembers the request. Call audio_feedback_flush_pending()
// once SCO is confirmed connected to actually play it.
void audio_feedback_defer_until_connected(sound_id_t id);
void audio_feedback_flush_pending(void);

// Call from hfp_manager.c's SCO-connected handling: true for
// ESP_HF_AUDIO_STATE_CONNECTED_MSBC (wideband, 16kHz -- matches how every
// clip is authored), false for ESP_HF_AUDIO_STATE_CONNECTED (CVSD,
// narrowband, 8kHz). Clips are always generated at 16kHz; when narrowband
// negotiates instead, audio_feedback downsamples on the fly rather than
// silently playing 16kHz-authored audio through an 8kHz pipeline (which
// halves both pitch and speed -- a slow, deep, "old person" voice).
void audio_feedback_set_wideband(bool wideband);

// Call from outgoing_data_callback() before touching the sidetone/normal path.
// Fills `buf` with up to `max_len` bytes of raw PCM from the currently
// playing clip. Returns actual bytes written (0 if nothing is playing ->
// caller should fall back to its normal source).
size_t audio_feedback_pull_frame(uint8_t *buf, size_t max_len);

#ifdef __cplusplus
}
#endif