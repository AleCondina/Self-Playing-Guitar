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
 *
 * Note fields: { string_id, freq_hz, duration_ms, num_strings, string_id2, freq_hz2 }
 * Currently using single notes on A2 (string 1) at 80 BPM.
 ******************************************************************************
 */

#include "song_scheduler.h"
#include <stddef.h>

/* =========================================================================
 * Song data  ── edit here to change the song
 * =========================================================================
 * Smoke on the Water – Deep Purple (main riff, intro)
 * Tempo: 80 BPM  |  Time: 4/4  |  A2 string only (single-note)
 *
 * String mapping:
 *   0 = E2  (low)    1 = A2    2 = D3
 *   3 = G3           4 = B3    5 = E4 (high)
 *
 * Fret frequencies – A2 string (110.00 Hz open):
 *   fret 0 = 110.00 Hz   fret 3 = 130.81 Hz
 *   fret 5 = 146.83 Hz   fret 6 = 155.56 Hz
 *
 * At 80 BPM:  quarter = 750 ms  |  eighth = 375 ms
 *             half = 1500 ms    |  dotted quarter = 1125 ms
 * ========================================================================= */

/* Convenience macro for a single-note entry on A2 (string index 1) */
#define NOTE(freq_hz, dur_ms) \
    { 1u, (freq_hz), (dur_ms), 1u, 0u, 0.0f }

static const SongNote_t song_notes[] = {

    /* ── Phrase 1: G – Bb – C  (one bar, 4 beats) ─────────────────── */
    NOTE(110.00f,  750u),   /* A(0)  – open, quarter       */
    NOTE(130.81f,  750u),   /* A(3)  – Bb,   quarter       */
    NOTE(146.83f, 1500u),   /* A(5)  – C,    half (held)   */

    /* ── Phrase 2: G – Bb – Db – C  (one bar, 4 beats) ───────────── */
    NOTE(110.00f,  750u),   /* A(0)  – open, quarter       */
    NOTE(130.81f,  750u),   /* A(3)  – Bb,   quarter       */
    NOTE(155.56f,  375u),   /* A(6)  – Db,   eighth        */
    NOTE(146.83f, 1125u),   /* A(5)  – C,    dotted quarter */

    /* ── Phrase 3: G – Bb – C – Bb – G  (one bar, 4 beats) ──────── */
    NOTE(110.00f,  750u),   /* A(0)  – open, quarter       */
    NOTE(130.81f,  750u),   /* A(3)  – Bb,   quarter       */
    NOTE(146.83f,  375u),   /* A(5)  – C,    eighth        */
    NOTE(130.81f,  375u),   /* A(3)  – Bb,   eighth        */
    NOTE(110.00f,  750u),   /* A(0)  – open, quarter       */
};

#define SONG_NOTE_COUNT  ((uint32_t)(sizeof(song_notes) / sizeof(song_notes[0])))

/* =========================================================================
 * Scheduler state
 * ========================================================================= */
static uint32_t s_note_index   = 0u;
static uint32_t s_note_start_ms = 0u;
static uint8_t  s_active        = 0u;
static uint8_t  s_initialised   = 0u;
static uint8_t  s_first_tick    = 1u;

/* =========================================================================
 * API implementation
 * ========================================================================= */

void SongSched_Init(void)
{
    s_note_index    = 0u;
    s_note_start_ms = 0u;   /* will be set on first Tick call */
    s_active        = (SONG_NOTE_COUNT > 0u) ? 1u : 0u;
    s_initialised   = 1u;
    s_first_tick    = 1u;
}

void SongSched_Tick(uint32_t now_ms)
{
    if (!s_initialised || !s_active) return;

    /* First tick: latch the song start time */
    if (s_first_tick) {
        s_note_start_ms = now_ms;
        s_first_tick = 0u;
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

uint8_t SongSched_GetStringCount(void)
{
    if (!s_active) return 0u;
    return song_notes[s_note_index].num_strings;
}

uint8_t SongSched_GetStringId2(void)
{
    if (!s_active) return 0u;
    return song_notes[s_note_index].string_id2;
}

float SongSched_GetTargetFreq2(void)
{
    if (!s_active) return 0.0f;
    return song_notes[s_note_index].target_freq_hz2;
}

uint8_t SongSched_IsActive(void)
{
    return s_active;
}

void SongSched_Restart(void)
{
    SongSched_Init();
}

void SongSched_Stop(void)
{
    s_active      = 0u;
    s_first_tick  = 1u;
    s_note_index  = 0u;
    s_note_start_ms = 0u;
    s_initialised = 1u;
}

uint32_t SongSched_GetNoteCount(void)
{
    return SONG_NOTE_COUNT;
}

uint32_t SongSched_GetNoteIndex(void)
{
    return s_note_index;
}
