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
 * @brief One note (or two-string chord) in the song schedule.
 *
 * For a single note set num_strings = 1 and leave string_id2 / target_freq_hz2 at 0.
 * For a two-string chord set num_strings = 2 and fill all fields.
 */
typedef struct {
    uint8_t  string_id;        /**< Primary string: 0=E2,1=A2,2=D3,3=G3,4=B3,5=E4 */
    float    target_freq_hz;   /**< Primary pitch in Hz                              */
    uint32_t duration_ms;      /**< How long to hold this note/chord (ms)            */
    uint8_t  num_strings;      /**< 1 = single note, 2 = chord                       */
    uint8_t  string_id2;       /**< Second string (valid when num_strings == 2)       */
    float    target_freq_hz2;  /**< Second pitch in Hz (valid when num_strings == 2)  */
    uint16_t burst_ms;         /**< Solenoid burst duration (ms), 0 = use default     */
} SongNote_t;

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief Initialise the scheduler (reset to first note of the active song).
 * Call once before the main loop.
 */
void SongSched_Init(void);

/**
 * @brief Select a song and initialise the scheduler.
 * @param song_id  0 = Smoke on the Water (chords), 1 = Smoke on the Water (A2 single string)
 */
void SongSched_InitSong(uint8_t song_id);

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
 * @brief Return the number of simultaneous strings in the current note (1 or 2).
 */
uint8_t SongSched_GetStringCount(void);

/**
 * @brief Return the second string index (valid only when SongSched_GetStringCount() == 2).
 */
uint8_t SongSched_GetStringId2(void);

/**
 * @brief Return the second target frequency in Hz (valid only when SongSched_GetStringCount() == 2).
 */
float SongSched_GetTargetFreq2(void);

/**
 * @brief Return 1 if the song is playing and a note is active, 0 otherwise.
 *
 * Returns 0 after the last note has expired (song finished) or before
 * SongSched_Init() has been called.
 */
uint8_t SongSched_IsActive(void);

/**
 * @brief Stop the song immediately.
 * Clears the active flag so SongSched_IsActive() returns 0.
 */
void SongSched_Stop(void);

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

/**
 * @brief Return the number of milliseconds elapsed since the current note started.
 * @param now_ms Current system time in milliseconds (HAL_GetTick()).
 */
uint32_t SongSched_GetElapsedMs(uint32_t now_ms);

/**
 * @brief Hold or release the current note.
 *
 * When hold=1 the scheduler will not advance to the next note even if the
 * note's duration has expired.  Used by the lock-wait mode in app_cm7.c to
 * guarantee every note is in tune before advancing.
 *
 * @param hold  1 = freeze on current note, 0 = normal time-based advance
 */
void SongSched_HoldNote(uint8_t hold);

/**
 * @brief Reset the current note's start time to now_ms.
 *
 * Called by the tuning controller after the pipeline (cold-start deferral +
 * solenoid fire) is ready, so that the note's full duration counts from the
 * moment the string is actually actuated — not from the first scheduler tick.
 *
 * @param now_ms  Current system time in milliseconds (HAL_GetTick()).
 */
void SongSched_ResetNoteTimer(uint32_t now_ms);

/**
 * @brief Return a read-only pointer to the note at the given index.
 * Returns NULL if idx >= note count.
 */
const SongNote_t *SongSched_GetNoteAt(uint32_t idx);

/**
 * @brief Return the BPM-scaled solenoid actuate delay (ms).
 *
 * At the reference BPM this equals SONG_ACTUATE_BASE_MS (1200 ms).
 * Higher BPM → shorter delay (less PI convergence time).
 * Lower BPM  → longer delay (more PI convergence time).
 */
uint32_t SongSched_GetActuateDelayMs(void);

#ifdef __cplusplus
}
#endif

#endif /* SONG_SCHEDULER_H */
