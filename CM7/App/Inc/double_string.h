/**
 ******************************************************************************
 * @file    double_string.h
 * @brief   Beat-based dual-string song scheduler with per-string convergence
 *
 * Architecture
 * ------------
 * Commands BOTH motors every hop.  While one string is playing (solenoid
 * fired + resting), the other string's motor is already moving to its next
 * target.  This maximizes PI convergence time per string.
 *
 * Song is defined as beat positions (not milliseconds).  At any BPM:
 *   fire_time = beat_position * (60000 / BPM)
 *   note_end  = fire_time + burst + rest
 *   convergence = fire_time - previous_same_string_note_end
 *
 * Rest time and burst stay constant; only convergence scales with BPM.
 *
 * Usage
 * -----
 *   DblString_Init(60);               // compute timing at 60 BPM
 *   DblString_Start(HAL_GetTick());   // begin playback
 *   // every hop:
 *   DblString_Tick(HAL_GetTick());
 *   freq = DblString_GetTargetFreq(str);
 *   if (DblString_ShouldFire(str)) { fire solenoid }
 ******************************************************************************
 */

#ifndef DOUBLE_STRING_H
#define DOUBLE_STRING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Constants
 * ========================================================================= */

/** Number of notes in the Smoke on the Water riff */
#define DBL_NUM_NOTES       12u

/** Number of string tracks (str0 = E2, str1 = G2) */
#define DBL_NUM_TRACKS      2u

/** Default BPM */
#define DBL_DEFAULT_BPM     47u

/** Minimum / maximum BPM range */
#define DBL_BPM_MIN         20u
#define DBL_BPM_MAX         200u

/** Pre-roll duration (ms) — time for motors to reach first targets on loop restart */
#define DBL_PREROLL_MS      1500u

/* =========================================================================
 * Data types
 * ========================================================================= */

/** One note in the beat-based song definition */
typedef struct {
    uint8_t  string_id;       /**< 0 or 1 */
    float    freq_hz;         /**< Target frequency */
    float    beat_pos;        /**< Beat position (fractional beats allowed) */
    uint16_t burst_ms;        /**< Solenoid burst duration (constant) */
    uint16_t rest_ms;         /**< Rest/breathing time after burst (constant) */
} DblNote_t;

/** Playback state machine */
typedef enum {
    DBL_STATE_IDLE,
    DBL_STATE_PREROLL,
    DBL_STATE_PLAYING,
    DBL_STATE_FINISHED
} DblState_t;

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief Compute timing tables from BPM and reset all state.
 * @param bpm  Playback tempo (clamped to DBL_BPM_MIN..DBL_BPM_MAX)
 */
void DblString_Init(uint32_t bpm);

/**
 * @brief Queue a BPM change, applied at next loop boundary.
 * @param bpm  New tempo (clamped to range)
 */
void DblString_SetBpm(uint32_t bpm);

/** @brief Get current playback BPM. */
uint32_t DblString_GetBpm(void);

/**
 * @brief Begin playback.
 * @param now_ms  Current HAL_GetTick()
 */
void DblString_Start(uint32_t now_ms);

/** @brief Stop playback immediately. */
void DblString_Stop(void);

/**
 * @brief Advance the scheduler.  Call every hop (or more often).
 * @param now_ms  Current HAL_GetTick()
 */
void DblString_Tick(uint32_t now_ms);

/** @brief 1 if playing or in pre-roll, 0 if idle/finished. */
uint8_t DblString_IsActive(void);

/**
 * @brief Get the motor target frequency for a string track.
 * @param track  0 or 1
 * @return Target freq in Hz (0 if track invalid or no target yet)
 */
float DblString_GetTargetFreq(uint8_t track);

/**
 * @brief Check and consume the solenoid fire flag for a track.
 * @param track  0 or 1
 * @return 1 if solenoid should fire NOW (edge-triggered, auto-cleared)
 */
uint8_t DblString_ShouldFire(uint8_t track);

/**
 * @brief Get burst duration for the current note on a track.
 * @param track  0 or 1
 * @return Burst in ms (0 if no active note)
 */
uint16_t DblString_GetBurstMs(uint8_t track);

/** @brief Current global note index (0..DBL_NUM_NOTES-1). */
uint32_t DblString_GetNoteIndex(void);

/** @brief Total number of notes. */
uint32_t DblString_GetNoteCount(void);

/** @brief Get the current playback state. */
DblState_t DblString_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* DOUBLE_STRING_H */
