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
 * Note durations in song_notes[] are written at SONG_BPM_REF (~47 BPM).
 * Change SONG_BPM to play slower or faster.  The scheduler scales the
 * PI-convergence and rest portions proportionally while keeping the
 * solenoid burst duration (burst_ms) constant.
 *
 * Scaling formula per note:
 *   scaled = ((duration_ms - burst_ms) * SONG_BPM_REF / SONG_BPM) + burst_ms
 *
 * The solenoid actuate delay also scales:
 *   scaled_actuate = SONG_ACTUATE_BASE_MS * SONG_BPM_REF / SONG_BPM
 *
 * Example:  REF=47 BPM=60 → note 1635ms becomes 1298ms, actuate 940ms
 *           REF=47 BPM=35 → note 1635ms becomes 2169ms, actuate 1612ms
 * ========================================================================= */
#define SONG_BPM_REF        47u   /**< Effective BPM the durations are written at */
#define SONG_BPM            47u   /**< Playback BPM — change this to speed/slow   */
#define SONG_LOOP_ENABLED    1u   /**< 1 = restart song automatically at end      */

/** Base solenoid actuate delay at SONG_BPM_REF (ms).
 *  Scaled at init so higher BPM = less PI convergence time. */
#define SONG_ACTUATE_BASE_MS 1200u

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
 *   Song0: unchanged from original (1063/1625/1313 ms, fires at 600 ms)
 *   Song1: see per-note comments in song1_notes[] below
 *
 * The solenoid fires once per note at the BPM-scaled actuate delay.
 * String carries energy from the previous burst — no re-excitation on
 * note change.  Motor moves via LUT immediately, PI refines, then
 * solenoid fires at the converged pitch.
 * ========================================================================= */

/* Convenience macros for note entries */
#define CHORD(g_hz, d_hz, dur_ms, burst) \
    { 3u, (g_hz), (dur_ms), 2u, 2u, (d_hz), (burst) }
#define CHORD2(s1, hz1, s2, hz2, dur_ms, burst) \
    { (s1), (hz1), (dur_ms), 2u, (s2), (hz2), (burst) }
#define NOTE(str, hz, dur_ms, burst) \
    { (str), (hz), (dur_ms), 1u, 0u, 0.0f, (burst) }

/* =========================================================================
 * Song 0: Smoke on the Water – alternating E2 + G2 strings (key of E)
 * =========================================================================
 * Requires TUNING_DROP_G2 (string 1 tuned from A2 down to G2).
 *
 * String mapping (Drop G2 tuning):
 *   0 = E2 (82.41 Hz open)    1 = G2 (98.00 Hz open)
 *
 * Riff transposed to E root — same intervals as Song1 (A-root):
 *   root = E2 (fret 0)   m3 = G2 (fret 3 on E2 / fret 0 on G2)
 *   P4   = A2 (fret 5)   tritone = Bb2/As2 (fret 6 on E2 / fret 3 on G2)
 *
 * Notes alternate between str0 (E2) and str1 (G2).
 * str0 (E2) plays: fret 0 (E2), fret 5 (A2), fret 6 (As2/Bb2, tritone).
 * str1 (G2) plays: fret 0 open (G2, the m3) and fret 2 (A2, the P4) in phrase 2 only.
 * Timing matches Song1: 1635ms standard, 1435ms short, 1885ms held.
 * ========================================================================= */
static const SongNote_t song0_notes[] = {

    /* ── Timing: 1200ms PI + 80ms burst + rest ───────────────────────── *
     *  Standard notes: rest=355ms → 1635ms                               *
     *  Short note (#5): rest=155ms → 1435ms                              *
     *  Held notes (#2,6,9,11): rest=605ms → 1885ms                       *
     *  Effective tempo: ~47 BPM (16 beats across 20.4s)                   *
     * ────────────────────────────────────────────────────────────────── */

    /* ── Phrase 1: root – m3 – P4 ────────────────────────────────────── */
    NOTE(0u, E2_HZ,   1635u,  80u),   /* #0  str0 E2 fret 0 (root)      */
    NOTE(1u, G2_HZ,   1635u,  80u),   /* #1  str1 G2 fret 0 open (m3)   */
    NOTE(0u, A2_HZ,   1885u,  80u),   /* #2  str0 E2 fret 5 (P4, held)  */

    /* ── Phrase 2: root – m3 – tritone – P4 ──────────────────────────── */
    NOTE(0u, E2_HZ,   1635u,  80u),   /* #3  str0 E2 fret 0 (root)      */
    NOTE(1u, G2_HZ,   1635u,  80u),   /* #4  str1 G2 fret 0 open (m3)   */
    NOTE(0u, As2_HZ,  1435u,  80u),   /* #5  str0 E2 fret 6 (tritone, short) */
    NOTE(1u, A2_HZ,   1885u,  80u),   /* #6  str1 G2 fret 2 (P4, held)      */

    /* ── Phrase 3: same as phrase 1 ──────────────────────────────────── */
    NOTE(0u, E2_HZ,   1635u,  80u),   /* #7  str0 E2 fret 0 (root)      */
    NOTE(1u, G2_HZ,   1635u,  80u),   /* #8  str1 G2 fret 0 (m3)        */
    NOTE(0u, A2_HZ,   1885u,  80u),   /* #9  str0 E2 fret 5 (P4, held)  */

    /* ── Phrase 4: m3 – root (ending) ────────────────────────────────── */
    NOTE(1u, G2_HZ,   1635u,  80u),   /* #10 str1 G2 fret 0 (m3)        */
    NOTE(0u, E2_HZ,   1885u,  80u),   /* #11 str0 E2 fret 0 (root, end) */
};

/* =========================================================================
 * Song 1: Smoke on the Water – single A2 string (string_id = 1)
 * =========================================================================
 * A2 fret frequencies:
 *   fret 0 = 110.00 Hz (A2)    fret 3 = 130.81 Hz (C3)
 *   fret 5 = 146.83 Hz (D3)    fret 6 = 155.56 Hz (Ds3)
 * ========================================================================= */
static const SongNote_t song1_notes[] = {

    /* ── Timing budget per note ──────────────────────────────────────── *
     *  1200ms PI + 100ms burst + rest                                        *
     *  Notes 0,1,3,4,7,8,10: rest=400ms → total=1700ms                      *
     *  Note 5:               rest=200ms → total=1500ms                      *
     *  Notes 2,6,9,11:       rest=650ms → total=1950ms                      *
     * ────────────────────────────────────────────────────────────────── */

    /* ── Phrase 1: fret 0 – fret 3 – fret 5 ──────────────────────────── */
    NOTE(1u, A2_HZ,   1700u, 100u),  /* #0  fret 0: cold start, rest=400   */
    NOTE(1u, C3_HZ,   1700u, 100u),  /* #1  fret 3: C3,  rest=400          */
    NOTE(1u, D3_HZ,   1950u, 100u),  /* #2  fret 5: D3,  rest=650          */

    /* ── Phrase 2: fret 0 – fret 3, fret 6 – fret 5 ──────────────────── */
    NOTE(1u, A2_HZ,   1700u, 100u),  /* #3  fret 0: A2,  rest=400          */
    NOTE(1u, C3_HZ,   1700u, 100u),  /* #4  fret 3: C3,  rest=400          */
    NOTE(1u, Ds3_HZ,  1500u, 100u),  /* #5  fret 6: Ds3, rest=200          */
    NOTE(1u, D3_HZ,   1950u, 100u),  /* #6  fret 5: D3,  rest=650          */

    /* ── Phrase 3: same as phrase 1 ─────────────────────────────────── */
    NOTE(1u, A2_HZ,   1700u, 100u),  /* #7  fret 0: A2,  rest=400          */
    NOTE(1u, C3_HZ,   1700u, 100u),  /* #8  fret 3: C3,  rest=400          */
    NOTE(1u, D3_HZ,   1950u, 100u),  /* #9  fret 5: D3,  rest=650          */

    /* ── Phrase 4: fret 3 then fret 0 (ending) ───────────────────────── */
    NOTE(1u, C3_HZ,   1700u, 100u),  /* #10 fret 3: C3,  rest=400          */
    NOTE(1u, A2_HZ,   1950u, 100u),  /* #11 fret 0: A2,  rest=650          */
};

/* =========================================================================
 * Song 2: Drop D fingerpicking pattern (played 4x via loop)
 * =========================================================================
 * Drop D tuning: str0=D2, str1=A2, str2=D3, str4=B3
 *
 * Tab:
 *   e|--------------------------------|
 *   B|----------------------------0---|
 *   G|--------------------------------|
 *   D|-----0------0------0------0---0-|  (4x)
 *   A|----0------3------0------3--3---|
 *   D|--0------3------0------3--------|
 *
 * Fret frequencies:
 *   D low: open=73.42 Hz (D2), fret 3=87.31 Hz (F2)
 *   A:     open=110.00 Hz (A2), fret 3=130.81 Hz (C3)
 *   D mid: open=146.83 Hz (D3)
 *   B:     open=246.94 Hz (B3)
 *
 * NOTE: String 4 has no commissioned solenoid timer in the current hardware
 *       map, so its motor can position but that note will not pluck.
 * ========================================================================= */
static const SongNote_t song2_notes[] = {

    /* ── Timing: 1200ms PI + 35ms burst + 400ms rest = 1635ms ──────── *
     *  Final note gets extra rest: 1200 + 35 + 650 = 1885ms            *
     * ────────────────────────────────────────────────────────────────── */

    /* ── Group 1: D-open arpeggio (D low → A → D mid) ────────────── */
    NOTE(0u, D2_HZ,   1635u,  35u),  /* #0  D low open              */
    NOTE(1u, A2_HZ,   1635u,  35u),  /* #1  A open                  */
    NOTE(2u, D3_HZ,   1635u,  35u),  /* #2  D mid open              */

    /* ── Group 2: F-chord arpeggio (D low fret3 → A fret3 → D mid) ── */
    NOTE(0u, F2_HZ,   1635u,  35u),  /* #3  D low fret 3            */
    NOTE(1u, C3_HZ,   1635u,  35u),  /* #4  A fret 3                */
    NOTE(2u, D3_HZ,   1635u,  35u),  /* #5  D mid open              */

    /* ── Group 3: D-open arpeggio (repeat of group 1) ────────────── */
    NOTE(0u, D2_HZ,   1635u,  35u),  /* #6  D low open              */
    NOTE(1u, A2_HZ,   1635u,  35u),  /* #7  A open                  */
    NOTE(2u, D3_HZ,   1635u,  35u),  /* #8  D mid open              */

    /* ── Group 4: F-chord arpeggio (repeat of group 2) ───────────── */
    NOTE(0u, F2_HZ,   1635u,  35u),  /* #9  D low fret 3            */
    NOTE(1u, C3_HZ,   1635u,  35u),  /* #10 A fret 3                */
    NOTE(2u, D3_HZ,   1635u,  35u),  /* #11 D mid open              */

    /* ── Ending: A fret3 + B open chord, then D mid ──────────────── */
    CHORD2(1u, C3_HZ, 4u, B3_HZ, 1635u, 35u),  /* #12 A fret3 + B open chord */
    NOTE(2u, D3_HZ,   1885u,  35u),  /* #13 D mid open (final, extra rest) */
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
    { song2_notes, (uint32_t)(sizeof(song2_notes) / sizeof(song2_notes[0])) },
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
static uint32_t s_scaled_actuate_delay_ms = SONG_ACTUATE_BASE_MS;

/* =========================================================================
 * API implementation
 * ========================================================================= */

void SongSched_Init(void)
{
    /* Pre-compute BPM-scaled durations for the active song.
     * Burst stays constant; only PI convergence + rest scale with BPM. */
    for (uint32_t i = 0u; i < s_song_count && i < SONG_MAX_NOTES; i++) {
        uint16_t burst    = s_song_ptr[i].burst_ms;
        uint32_t non_burst = s_song_ptr[i].duration_ms - burst;
        s_scaled_durations[i] = (non_burst * SONG_BPM_REF) / SONG_BPM + burst;
    }

    /* Scale the solenoid actuate delay (PI convergence budget) */
    s_scaled_actuate_delay_ms = (SONG_ACTUATE_BASE_MS * SONG_BPM_REF) / SONG_BPM;

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

uint32_t SongSched_GetActuateDelayMs(void)
{
    return s_scaled_actuate_delay_ms;
}
