/**
 ******************************************************************************
 * @file    song_scheduler.h
 * @brief   Song schedule – note sequencer for the autonomous guitar
 *
 * Provides a simple time-indexed note schedule: a hardcoded array of
 * (string_id, target_freq_hz, duration_ms) entries played back in order.
 * The scheduler advances automatically when SongSched_Tick() is called
 * with the current HAL_GetTick() time.
 *
 * To change the song, edit the `song_notes[]` array in song_scheduler.c.
 *
 * Usage
 * -----
 *  SongSched_Init();          // call once in setup
 *  SongSched_Tick(now_ms);    // call every hop (or more often – idempotent)
 *  str  = SongSched_GetStringId();
 *  freq = SongSched_GetTargetFreq();
 *  if (SongSched_IsActive()) { ... }
 ******************************************************************************
 */

#ifndef SONG_SCHEDULER_H
#define SONG_SCHEDULER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Data types
 * ========================================================================= */

/**
 * @brief One note in the song schedule.
 */
typedef struct {
    uint8_t  string_id;        /**< String to tune: 0=E2, 1=A2, 2=D3,
                                     3=G3, 4=B3, 5=E4                 */
    float    target_freq_hz;   /**< Target pitch in Hz                */
    uint32_t duration_ms;      /**< How long to hold this note (ms)   */
} SongNote_t;

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief Initialise the scheduler (reset to first note).
 * Call once before the main loop.
 */
void SongSched_Init(void);

/**
 * @brief Advance the scheduler to the current time.
 *
 * Must be called at least once per control hop with the current
 * HAL_GetTick() value.  Idempotent if called more than once per hop.
 *
 * @param now_ms  Current system time in milliseconds (HAL_GetTick()).
 */
void SongSched_Tick(uint32_t now_ms);

/**
 * @brief Return the string index for the currently active note.
 * Valid only when SongSched_IsActive() returns non-zero.
 */
uint8_t SongSched_GetStringId(void);

/**
 * @brief Return the target frequency for the currently active note (Hz).
 * Valid only when SongSched_IsActive() returns non-zero.
 */
float SongSched_GetTargetFreq(void);

/**
 * @brief Return 1 if the song is playing and a note is active, 0 otherwise.
 *
 * Returns 0 after the last note has expired (song finished) or before
 * SongSched_Init() has been called.
 */
uint8_t SongSched_IsActive(void);

/**
 * @brief Restart the song from the first note.
 * Can be called at any time to loop the song.
 */
void SongSched_Restart(void);

/**
 * @brief Return total number of notes in the current song.
 */
uint32_t SongSched_GetNoteCount(void);

/**
 * @brief Return the index of the currently active note.
 */
uint32_t SongSched_GetNoteIndex(void);

#ifdef __cplusplus
}
#endif

#endif /* SONG_SCHEDULER_H */
