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
#define SONG_BPM            60u   /**< Playback BPM — 1:1 with reference tempo   */
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
 * Note timing (durations encode the full note lifecycle):
 *   Note 0 (cold start):  1000ms burst + 750ms settle     = 1750ms
 *   Whole notes:          1500ms PI    + 500ms burst + 500ms rest = 2500ms
 *   Half  notes (5, 6):   1500ms PI    + 250ms burst + 250ms rest = 2000ms
 *
 * The solenoid fires once per note at SONG_ACTUATE_DELAY_MS (1500 ms).
 * String carries energy from the previous burst — no re-excitation on
 * note change.  Motor moves via LUT immediately, PI refines, then
 * solenoid fires at the converged pitch.
 * ========================================================================= */

/* Convenience macros for note entries */
#define CHORD(g_hz, d_hz, dur_ms) \
    { 3u, (g_hz), (dur_ms), 2u, 2u, (d_hz) }
#define NOTE(str, hz, dur_ms) \
    { (str), (hz), (dur_ms), 1u, 0u, 0.0f }

/* =========================================================================
 * Song 0: Smoke on the Water – two-string chords (G3 + D3)
 * ========================================================================= */
static const SongNote_t song0_notes[] = {

    /* ── Timing budget per note ──────────────────────────────────────── *
     *  Note 0 (cold start): 1000ms burst + 750ms FFT settle = 1750ms   *
     *  Whole notes:  1500ms PI + 500ms burst + 500ms rest   = 2500ms   *
     *  Half  notes:  1500ms PI + 250ms burst + 250ms rest   = 2000ms   *
     * ────────────────────────────────────────────────────────────────── */

    /* ── Phrase 1: fret 0 – fret 3 – fret 5 ──────────────────────────── */
    CHORD(G3_HZ,   D3_HZ,   1750u),  /* #0  fret 0: cold start burst     */
    CHORD(As3_HZ,  F3_HZ,   2500u),  /* #1  fret 3: Bb3 + F3  – whole    */
    CHORD(C4_HZ,   G3_HZ,   2500u),  /* #2  fret 5: C4 + G3   – whole    */

    /* ── Phrase 2: fret 0 – fret 3 (whole), fret 6 – fret 5 (half) ─── */
    CHORD(G3_HZ,   D3_HZ,   2500u),  /* #3  fret 0: G3 + D3   – whole    */
    CHORD(As3_HZ,  F3_HZ,   2500u),  /* #4  fret 3: Bb3 + F3  – whole    */
    CHORD(Cs4_HZ,  Gs3_HZ,  2000u),  /* #5  fret 6: Db4 + Ab3 – half     */
    CHORD(C4_HZ,   G3_HZ,   2000u),  /* #6  fret 5: C4  + G3  – half     */

    /* ── Phrase 3: same as phrase 1 ─────────────────────────────────── */
    CHORD(G3_HZ,   D3_HZ,   2500u),  /* #7  fret 0: G3 + D3   – whole    */
    CHORD(As3_HZ,  F3_HZ,   2500u),  /* #8  fret 3: Bb3 + F3  – whole    */
    CHORD(C4_HZ,   G3_HZ,   2500u),  /* #9  fret 5: C4 + G3   – whole    */

    /* ── Phrase 4: fret 3 then fret 0 (ending) ───────────────────────── */
    CHORD(As3_HZ,  F3_HZ,   2500u),  /* #10 fret 3: Bb3 + F3  – whole    */
    CHORD(G3_HZ,   D3_HZ,   2500u),  /* #11 fret 0: G3 + D3   – ending   */
};

/* =========================================================================
 * Song 1: Smoke on the Water – single A2 string (string_id = 1)
 * =========================================================================
 * A2 fret frequencies:
 *   fret 0 = 110.00 Hz (A2)    fret 3 = 130.81 Hz (C3)
 *   fret 5 = 146.83 Hz (D3)    fret 6 = 155.56 Hz (Ds3)
 * ========================================================================= */
static const SongNote_t song1_notes[] = {

    /* ── Phrase 1: fret 0 – fret 3 – fret 5 ──────────────────────────── */
    NOTE(1u, A2_HZ,   1750u),   /* #0  fret 0: cold start burst         */
    NOTE(1u, C3_HZ,   2500u),   /* #1  fret 3: C3   – whole             */
    NOTE(1u, D3_HZ,   2500u),   /* #2  fret 5: D3   – whole             */

    /* ── Phrase 2: fret 0 – fret 3 (whole), fret 6 – fret 5 (half) ─── */
    NOTE(1u, A2_HZ,   2500u),   /* #3  fret 0: A2   – whole             */
    NOTE(1u, C3_HZ,   2500u),   /* #4  fret 3: C3   – whole             */
    NOTE(1u, Ds3_HZ,  2000u),   /* #5  fret 6: Ds3  – half              */
    NOTE(1u, D3_HZ,   2000u),   /* #6  fret 5: D3   – half              */

    /* ── Phrase 3: same as phrase 1 ─────────────────────────────────── */
    NOTE(1u, A2_HZ,   2500u),   /* #7  fret 0: A2   – whole             */
    NOTE(1u, C3_HZ,   2500u),   /* #8  fret 3: C3   – whole             */
    NOTE(1u, D3_HZ,   2500u),   /* #9  fret 5: D3   – whole             */

    /* ── Phrase 4: fret 3 then fret 0 (ending) ───────────────────────── */
    NOTE(1u, C3_HZ,   2500u),   /* #10 fret 3: C3   – whole             */
    NOTE(1u, A2_HZ,   2500u),   /* #11 fret 0: A2   – ending            */
};

/* =========================================================================
 * Song table — add new songs here
 * ========================================================================= */
typedef struct {
    const SongNote_t *notes;
    uint32_t          count;
} SongDef_t;

static const SongDef_t s_songs[] = {
    { song0_notes, (uint32_t)(sizeof(song0_notes) / sizeof(song0_notes[0])) },
    { song1_notes, (uint32_t)(sizeof(song1_notes) / sizeof(song1_notes[0])) },
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

const SongNote_t *SongSched_GetNoteAt(uint32_t idx)
{
    if (idx >= s_song_count) return NULL;
    return &s_song_ptr[idx];
}

void SongSched_Advance(void)
{
    if (!s_initialised || !s_active) return;
    s_note_index++;
    if (s_note_index >= s_song_count) {
#if SONG_LOOP_ENABLED
        SongSched_Init();
#else
        s_active = 0u;
#endif
    }
}

const SongNote_t *SongSched_PeekNextNote(void)
{
    if (!s_active) return NULL;
    uint32_t next = s_note_index + 1u;
    if (next >= s_song_count) {
#if SONG_LOOP_ENABLED
        return &s_song_ptr[0];
#else
        return NULL;
#endif
    }
    return &s_song_ptr[next];
}

uint32_t SongSched_GetScaledDurationMs(uint32_t idx)
{
    if (idx >= s_song_count || idx >= SONG_MAX_NOTES) return 0u;
    return s_scaled_durations[idx];
}
