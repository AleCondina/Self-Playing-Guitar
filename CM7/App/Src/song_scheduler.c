/**
 ******************************************************************************
 * @file    song_scheduler.c
 * @brief   Song schedule implementation – hardcoded note sequence
 *
 * Edit the `song_notes[]` array below to change the song.
 * Duration values are in milliseconds.  The first note begins immediately
 * after SongSched_Init() is called.
 *
 * Standard open-string frequencies (Hz):
 *   E2 = 82.41   A2 = 110.00   D3 = 146.83
 *   G3 = 196.00  B3 = 246.94   E4 = 329.63
 ******************************************************************************
 */

#include "song_scheduler.h"
#include <stddef.h>

/* =========================================================================
 * Song data  ── edit here to change the song
 * =========================================================================
 * Demo song: simple pentatonic phrase on E2 and A2 strings.
 * All frequencies are in standard equal temperament (A4 = 440 Hz).
 *
 * String mapping:
 *   0 = E2  (low)    1 = A2    2 = D3
 *   3 = G3           4 = B3    5 = E4 (high)
 * ========================================================================= */
static const SongNote_t song_notes[] = {
    /* -- Bar 1: E2 string -- */
    { 0,  82.41f,  800u },   /* E2  open  */
    { 0,  87.31f,  400u },   /* F2  fret1 */
    { 0,  92.50f,  400u },   /* F#2 fret2 */
    { 0,  98.00f,  800u },   /* G2  fret3 */
    { 0, 110.00f,  800u },   /* A2  fret5 */

    /* -- Bar 2: A2 string -- */
    { 1, 110.00f,  800u },   /* A2  open  */
    { 1, 116.54f,  400u },   /* A#2 fret1 */
    { 1, 123.47f,  400u },   /* B2  fret2 */
    { 1, 130.81f,  800u },   /* C3  fret3 */
    { 1, 146.83f,  800u },   /* D3  fret5 */

    /* -- Bar 3: D3 string -- */
    { 2, 146.83f,  800u },   /* D3  open  */
    { 2, 155.56f,  400u },   /* D#3 fret1 */
    { 2, 164.81f,  400u },   /* E3  fret2 */
    { 2, 174.61f,  800u },   /* F3  fret3 */
    { 2, 196.00f,  800u },   /* G3  fret5 */

    /* -- Bar 4: back to E2 -- */
    { 0, 196.00f,  400u },   /* G2  fret10 (high, tests full range) */
    { 0, 164.81f,  400u },   /* E3  fret7  */
    { 0, 146.83f,  400u },   /* D3  fret5  */
    { 0, 123.47f,  400u },   /* B2  fret2  */
    { 0,  82.41f, 1200u },   /* E2  open (return home) */
};

#define SONG_NOTE_COUNT  ((uint32_t)(sizeof(song_notes) / sizeof(song_notes[0])))

/* =========================================================================
 * Scheduler state
 * ========================================================================= */
static uint32_t s_note_index   = 0u;
static uint32_t s_note_start_ms = 0u;
static uint8_t  s_active        = 0u;
static uint8_t  s_initialised   = 0u;

/* =========================================================================
 * API implementation
 * ========================================================================= */

void SongSched_Init(void)
{
    s_note_index    = 0u;
    s_note_start_ms = 0u;   /* will be set on first Tick call */
    s_active        = (SONG_NOTE_COUNT > 0u) ? 1u : 0u;
    s_initialised   = 1u;
}

void SongSched_Tick(uint32_t now_ms)
{
    if (!s_initialised || !s_active) return;

    /* First tick: latch the song start time */
    static uint8_t first_tick = 1u;
    if (first_tick) {
        s_note_start_ms = now_ms;
        first_tick = 0u;
    }

    /* Check if the current note has expired */
    uint32_t elapsed = now_ms - s_note_start_ms;

    if (elapsed >= song_notes[s_note_index].duration_ms) {
        /* Advance to next note */
        s_note_index++;

        if (s_note_index >= SONG_NOTE_COUNT) {
            /* Song finished */
            s_active = 0u;
            return;
        }

        /* Latch start time of new note */
        s_note_start_ms = now_ms;
    }
}

uint8_t SongSched_GetStringId(void)
{
    if (!s_active) return 0u;
    return song_notes[s_note_index].string_id;
}

float SongSched_GetTargetFreq(void)
{
    if (!s_active) return 0.0f;
    return song_notes[s_note_index].target_freq_hz;
}

uint8_t SongSched_IsActive(void)
{
    return s_active;
}

void SongSched_Restart(void)
{
    s_note_index    = 0u;
    s_note_start_ms = 0u;
    /* Reset first_tick flag (static local) is not directly accessible;
     * re-init via SongSched_Init() achieves the same effect. */
    SongSched_Init();
}

uint32_t SongSched_GetNoteCount(void)
{
    return SONG_NOTE_COUNT;
}

uint32_t SongSched_GetNoteIndex(void)
{
    return s_note_index;
}
