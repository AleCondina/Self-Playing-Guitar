/**
 ******************************************************************************
 * @file    song_scheduler.c
 * @brief   Song schedule implementation – hardcoded note sequence
 *
 * Edit the `song_notes[]` array below to change the song.
 * Duration values are in milliseconds at the reference tempo (SONG_BPM_REF).
 * Set SONG_BPM to a different value to scale the entire song faster or slower.
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
#include "application_config.h"
#include <stddef.h>

/* =========================================================================
 * BPM configuration
 * =========================================================================
 * All note durations in song_notes[] are written at SONG_BPM_REF (112 BPM).
 * Change SONG_BPM to play slower (e.g. 80) or faster (e.g. 120).
 * The scheduler scales every duration proportionally on SongSched_Init().
 *
 * Tip: if the guitar cannot achieve PI lock within a quarter note at 112 BPM,
 * drop SONG_BPM to 80 for more reliable note accuracy.  Enable
 * SONG_LOCK_WAIT_MODE in app_cm7.c for guaranteed accuracy at any BPM.
 * ========================================================================= */
#define SONG_BPM_REF        60u   /**< Reference BPM of the durations below     */
#define SONG_BPM            40u   /**< Playback BPM — conservative for PI tuning */
#define SONG_LOOP_ENABLED    1u   /**< 1 = restart song automatically at end     */

/* =========================================================================
 * Song data  ── edit here to change the song
 * =========================================================================
 * Smoke on the Water – Deep Purple (main riff)
 *
 * String mapping:
 *   2 = D3   3 = G3
 * Both strings play the same fret simultaneously on every chord.
 *
 * Fret frequencies – G3 string (196.00 Hz open):
 *   fret 0 = 196.00 Hz   fret 3 = 233.08 Hz
 *   fret 5 = 261.63 Hz   fret 6 = 277.18 Hz
 *
 * Fret frequencies – D3 string (146.83 Hz open):
 *   fret 0 = 146.83 Hz   fret 3 = 174.61 Hz
 *   fret 5 = 196.00 Hz   fret 6 = 207.65 Hz
 *
 * Note timing:
 *   Song0: scheduler hold times are encoded directly in song0_notes[]
 *   Song1: see per-note comments in song1_notes[] below
 *
 * Each note now follows the live tuning-controller pipeline:
 * solenoid burst at note onset, one-hop resonance wait, feedforward move,
 * then PI only after the post-burst FFT window is clean and the motor stops.
 * ========================================================================= */

/* Convenience macros for note entries */
#define CHORD(g_hz, d_hz, dur_ms, burst) \
    { 3u, (g_hz), (dur_ms), 2u, 2u, (d_hz), (burst) }
#define NOTE(str, hz, dur_ms, burst) \
    { (str), (hz), (dur_ms), 1u, 0u, 0.0f, (burst) }

/* =========================================================================
 * Song 0: Smoke on the Water – two-string chords (G3 + D3)
 * ========================================================================= */
static const SongNote_t song0_notes[] = {

    /* ── Timing note ─────────────────────────────────────────────────── *
     *  Durations below are scheduler hold times only. Runtime note flow  *
     *  is: burst -> 1 hop breathe -> FF -> PI after clean FFT + motor    *
     *  done. Longer bursts consume more of the available PI window.      *
     * ────────────────────────────────────────────────────────────────── */

    /* ── Phrase 1: fret 0 – fret 3 – fret 5 ──────────────────────────── */
    CHORD(G3_HZ,   D3_HZ,   1363u, 300u),  /* #0  fret 0: cold start burst     */
    CHORD(As3_HZ,  F3_HZ,   1925u, 250u),  /* #1  fret 3: Bb3 + F3  – whole    */
    CHORD(C4_HZ,   G3_HZ,   1925u, 250u),  /* #2  fret 5: C4 + G3   – whole    */

    /* ── Phrase 2: fret 0 – fret 3 (whole), fret 6 – fret 5 (half) ─── */
    CHORD(G3_HZ,   D3_HZ,   1925u, 250u),  /* #3  fret 0: G3 + D3   – whole    */
    CHORD(As3_HZ,  F3_HZ,   1925u, 250u),  /* #4  fret 3: Bb3 + F3  – whole    */
    CHORD(Cs4_HZ,  Gs3_HZ,  1613u, 125u),  /* #5  fret 6: Db4 + Ab3 – half     */
    CHORD(C4_HZ,   G3_HZ,   1613u, 125u),  /* #6  fret 5: C4  + G3  – half     */

    /* ── Phrase 3: same as phrase 1 ─────────────────────────────────── */
    CHORD(G3_HZ,   D3_HZ,   1925u, 250u),  /* #7  fret 0: G3 + D3   – whole    */
    CHORD(As3_HZ,  F3_HZ,   1925u, 250u),  /* #8  fret 3: Bb3 + F3  – whole    */
    CHORD(C4_HZ,   G3_HZ,   1925u, 250u),  /* #9  fret 5: C4 + G3   – whole    */

    /* ── Phrase 4: fret 3 then fret 0 (ending) ───────────────────────── */
    CHORD(As3_HZ,  F3_HZ,   1925u, 250u),  /* #10 fret 3: Bb3 + F3  – whole    */
    CHORD(G3_HZ,   D3_HZ,   1925u, 250u),  /* #11 fret 0: G3 + D3   – ending   */
};

/* =========================================================================
 * Song 1: Smoke on the Water – single E2 string (string_id = 0)
 * =========================================================================
 * E2 fret frequencies:
 *   fret 0 = 82.41 Hz (E2)     fret 3 = 98.00 Hz (G2)
 *   fret 5 = 110.00 Hz (A2)    fret 6 = 116.54 Hz (As2)
 * ========================================================================= */
static const SongNote_t song1_notes[] = {

    /* ── Timing note ─────────────────────────────────────────────────── *
     *  Simple linear FF + PI mode. Solenoid fires short burst (72ms)    *
     *  on each note to keep string vibrating. Cold start uses 125ms.    *
     * ────────────────────────────────────────────────────────────────── */

    /* ── Measure 1: fret 0 – fret 3 – fret 5 ──────────────────────────── */
    NOTE(0u, E2_HZ,   1000u,  125u),  /* #0  fret 0: cold start        */
    NOTE(0u, G2_HZ,   1000u,  125u),   /* #1  fret 3: G2                */
    NOTE(0u, A2_HZ,   1500u,  125u),   /* #2  fret 5: A2                */
    NOTE(0u, E2_HZ,   1000u,  125u),   /* #3  fret 0: E2                */

    /* ── Measure 2: fret 0 – fret 3, fret 6 – fret 5 ──────────────────── */
    NOTE(0u, G2_HZ,   1000u,  125u),   /* #4  fret 3: G2                */
    NOTE(0u, As2_HZ,  500u,   125u),   /* #5  fret 6: As2               */
    NOTE(0u, A2_HZ,   2000u,  125u),   /* #6  fret 5: A2                */

    /* ── Measure 3: same as measure 1 ─────────────────────────────────── */
    NOTE(0u, E2_HZ,   1000u,  125u),   /* #7  fret 0: E2                */
    NOTE(0u, G2_HZ,   1000u,  125u),   /* #8  fret 3: G2                */
    NOTE(0u, A2_HZ,   1500u,  125u),   /* #9  fret 5: A2                */
    NOTE(0u, G2_HZ,   1000u,  125u),   /* #10 fret 3: G2                */

    /* ── Phrase 4: fret 3 then fret 0 (ending) ───────────────────────── */
    NOTE(0u, E2_HZ,   4000u,  125u),   /* #11 fret 0: E2                */

};

/* =========================================================================
 * Song 2: Test — single E2 hold (10 s)
 * ========================================================================= */
static const SongNote_t song2_notes[] = {
    NOTE(0u, E2_HZ, 10000u, 125u),
};

/* =========================================================================
 * Song 3: Test — E2 → G2 slow transition (5 s each)
 * ========================================================================= */
static const SongNote_t song3_notes[] = {
    NOTE(0u, E2_HZ, 5000u, 125u),
    NOTE(0u, G2_HZ, 5000u, 125u),
};

/* =========================================================================
 * Song 4: Test — E2 → G2 fast transition (1 s each)
 * ========================================================================= */
static const SongNote_t song4_notes[] = {
    NOTE(0u, E2_HZ, 1000u, 125u),
    NOTE(0u, G2_HZ, 1000u, 125u),
};

/* =========================================================================
 * Song table — add new songs here
 * ========================================================================= */
typedef struct {
    const SongNote_t *notes;
    uint32_t          count;
} SongDef_t;

#define SONG_ENTRY(arr)  { (arr), (uint32_t)(sizeof(arr) / sizeof((arr)[0])) }

static const SongDef_t s_songs[] = {
    SONG_ENTRY(song0_notes),  /* play 0: Smoke chords (G3+D3)   */
    SONG_ENTRY(song1_notes),  /* play 1: Smoke single E2        */
    SONG_ENTRY(song2_notes),  /* play 2: Test — E2 10s hold     */
    SONG_ENTRY(song3_notes),  /* play 3: Test — E2→G2 5s each   */
    SONG_ENTRY(song4_notes),  /* play 4: Test — E2→G2 1s each   */
};

#define NUM_SONGS  ((uint32_t)(sizeof(s_songs) / sizeof(s_songs[0])))

/* Active song pointer + count (set by SongSched_InitSong) */
static const SongNote_t *s_song_ptr   = song0_notes;
static uint32_t          s_song_count = (uint32_t)(sizeof(song0_notes) / sizeof(song0_notes[0]));

/* =========================================================================
 * Scaled duration table  (computed from SONG_BPM at init)
 * Max 16 notes per song — increase if needed.
 * ========================================================================= */
#define SONG_MAX_NOTES  16u
static uint32_t s_scaled_durations[SONG_MAX_NOTES];

/* =========================================================================
 * Scheduler state
 * ========================================================================= */
static uint32_t s_note_index    = 0u;
static uint32_t s_note_start_ms = 0u;
static uint8_t  s_active        = 0u;
static uint8_t  s_initialised   = 0u;
static uint8_t  s_first_tick    = 1u;
static uint8_t  s_hold_note     = 0u;

/* =========================================================================
 * API implementation
 * ========================================================================= */

void SongSched_Init(void)
{
    /* Pre-compute BPM-scaled durations for the active song */
    for (uint32_t i = 0u; i < s_song_count && i < SONG_MAX_NOTES; i++) {
        s_scaled_durations[i] =
            (s_song_ptr[i].duration_ms * SONG_BPM_REF) / SONG_BPM;
    }

    s_note_index    = 0u;
    s_note_start_ms = 0u;
    s_active        = (s_song_count > 0u) ? 1u : 0u;
    s_initialised   = 1u;
    s_first_tick    = 1u;
    s_hold_note     = 0u;
}

void SongSched_InitSong(uint8_t song_id)
{
    if (song_id < NUM_SONGS) {
        s_song_ptr   = s_songs[song_id].notes;
        s_song_count = s_songs[song_id].count;
    }
    SongSched_Init();
}

void SongSched_Tick(uint32_t now_ms)
{
    if (!s_initialised || !s_active) return;

    if (s_first_tick) {
        s_note_start_ms = now_ms;
        s_first_tick = 0u;
    }

    if (s_hold_note) return;

    uint32_t elapsed = now_ms - s_note_start_ms;

    if (elapsed >= s_scaled_durations[s_note_index]) {
        s_note_start_ms += s_scaled_durations[s_note_index];
        s_note_index++;

        if (s_note_index >= s_song_count) {
#if SONG_LOOP_ENABLED
            SongSched_Init();
#else
            s_active = 0u;
#endif
            return;
        }
    }
}

uint8_t SongSched_GetStringId(void)
{
    if (!s_active) return 0u;
    return s_song_ptr[s_note_index].string_id;
}

float SongSched_GetTargetFreq(void)
{
    if (!s_active) return 0.0f;
    return s_song_ptr[s_note_index].target_freq_hz;
}

uint8_t SongSched_GetStringCount(void)
{
    if (!s_active) return 0u;
    return s_song_ptr[s_note_index].num_strings;
}

uint8_t SongSched_GetStringId2(void)
{
    if (!s_active) return 0u;
    return s_song_ptr[s_note_index].string_id2;
}

float SongSched_GetTargetFreq2(void)
{
    if (!s_active) return 0.0f;
    return s_song_ptr[s_note_index].target_freq_hz2;
}

uint8_t SongSched_IsActive(void)
{
    return s_active;
}

void SongSched_Stop(void)
{
    s_active = 0u;
}

void SongSched_Restart(void)
{
    SongSched_Init();
}

uint32_t SongSched_GetNoteCount(void)
{
    return s_song_count;
}

uint32_t SongSched_GetNoteIndex(void)
{
    return s_note_index;
}

uint32_t SongSched_GetElapsedMs(uint32_t now_ms)
{
    if (!s_initialised || !s_active || s_first_tick) return 0u;
    return now_ms - s_note_start_ms;
}

void SongSched_HoldNote(uint8_t hold)
{
    s_hold_note = hold;
}

void SongSched_ResetNoteTimer(uint32_t now_ms)
{
    if (s_initialised && s_active) {
        s_note_start_ms = now_ms;
    }
}

const SongNote_t *SongSched_GetNoteAt(uint32_t idx)
{
    if (idx >= s_song_count) return NULL;
    return &s_song_ptr[idx];
}
