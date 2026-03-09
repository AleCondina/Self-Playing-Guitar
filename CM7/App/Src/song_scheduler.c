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
 * Chord fields: { string_id, freq_hz, duration_ms, num_strings, string_id2, freq_hz2 }
 * Set num_strings = 1 for a single note, 2 for a two-string chord.
 ******************************************************************************
 */

#include "song_scheduler.h"
#include <stddef.h>

/* =========================================================================
 * Song data  ── edit here to change the song
 * =========================================================================
 * Smoke on the Water – Deep Purple (main riff, intro)
 * Tempo: 112 BPM  |  Time: 4/4
 *
 * String mapping:
 *   0 = E2  (low)    1 = A2    2 = D3
 *   3 = G3           4 = B3    5 = E4 (high)
 *
 * All chords are played on G (idx 3) + D (idx 2) strings simultaneously.
 *
 * Fret frequencies – G3 string (196.00 Hz open):
 *   fret 0 = 196.00 Hz   fret 3 = 233.08 Hz
 *   fret 5 = 261.63 Hz   fret 6 = 277.18 Hz
 *
 * Fret frequencies – D3 string (146.83 Hz open):
 *   fret 0 = 146.83 Hz   fret 3 = 174.61 Hz
 *   fret 5 = 196.00 Hz   fret 6 = 207.65 Hz
 *
 * At 112 BPM:  quarter = 536 ms  |  eighth = 268 ms
 * ========================================================================= */

/* Convenience macro for a two-string chord entry */
#define CHORD(g_hz, d_hz, dur_ms) \
    { 3u, (g_hz), (dur_ms), 2u, 2u, (d_hz) }

static const SongNote_t song_notes[] = {

    /* ── Phrase 1: G5 – Bb5 – C5  (one bar, 4 beats) ─────────────────── */
    CHORD(196.00f, 146.83f,  536u),   /* G(0)D(0)  – open power chord   */
    CHORD(233.08f, 174.61f,  536u),   /* G(3)D(3)  – Bb5 power chord    */
    CHORD(261.63f, 196.00f, 1072u),   /* G(5)D(5)  – C5  power chord (held + rest) */

    /* ── Phrase 2: G5 – Bb5 – Db5 – C5  (one bar, 4 beats) ───────────── */
    CHORD(196.00f, 146.83f,  536u),   /* G(0)D(0)  – open power chord   */
    CHORD(233.08f, 174.61f,  536u),   /* G(3)D(3)  – Bb5 power chord    */
    CHORD(277.18f, 207.65f,  268u),   /* G(6)D(6)  – Db5 power chord    */
    CHORD(261.63f, 196.00f,  804u),   /* G(5)D(5)  – C5  (dotted quarter + rest) */

    /* ── Phrase 3: G5 – Bb5 – C5 – Bb5 – G5  (one bar, 4 beats) ──────── */
    CHORD(196.00f, 146.83f,  536u),   /* G(0)D(0)  – open power chord   */
    CHORD(233.08f, 174.61f,  536u),   /* G(3)D(3)  – Bb5 power chord    */
    CHORD(261.63f, 196.00f,  268u),   /* G(5)D(5)  – C5  power chord    */
    CHORD(233.08f, 174.61f,  268u),   /* G(3)D(3)  – Bb5 power chord    */
    CHORD(196.00f, 146.83f,  536u),   /* G(0)D(0)  – return to open (held + rest) */
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
