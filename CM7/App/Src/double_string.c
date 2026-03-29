/**
 ******************************************************************************
 * @file    double_string.c
 * @brief   Beat-based dual-string song scheduler — Smoke on the Water riff
 *
 * Commands BOTH motors every hop.  While one string is playing (solenoid
 * fired + resting), the other string's motor moves to its next target,
 * maximizing PI convergence time.
 *
 * See double_string.h for the full architecture description.
 ******************************************************************************
 */

#include "double_string.h"
#include "application_config.h"
#include <stdio.h>
#include <string.h>

/* =========================================================================
 * Song data — Smoke on the Water, E root, Drop G2 tuning
 * =========================================================================
 * String mapping (Drop G2):
 *   str0 = E2 (82.41 Hz open)    str1 = G2 (98.00 Hz open)
 *
 * Riff intervals:
 *   root = E2    m3 = G2    P4 = A2    tritone = As2/Bb2
 *
 * Rest times are CONSTANT (do not scale with BPM):
 *   Quarter note: rest = 355 ms
 *   Eighth note:  rest = 155 ms
 *   Half/held:    rest = 605 ms
 * ========================================================================= */

static const DblNote_t s_song[DBL_NUM_NOTES] = {
    /* Phrase 1: root - m3 - P4 */
    { 0u, E2_HZ,   0.0f,  80u, 355u },  /* #0  str0 E2  quarter       */
    { 1u, G2_HZ,   1.0f,  80u, 355u },  /* #1  str1 G2  quarter       */
    { 0u, A2_HZ,   2.0f,  80u, 605u },  /* #2  str0 A2  half (held)   */

    /* Phrase 2: root - m3 - tritone - P4 */
    { 0u, E2_HZ,   4.0f,  80u, 355u },  /* #3  str0 E2  quarter       */
    { 1u, G2_HZ,   5.0f,  80u, 355u },  /* #4  str1 G2  quarter       */
    { 0u, As2_HZ,  6.0f,  80u, 155u },  /* #5  str0 As2 eighth (short)*/
    { 1u, A2_HZ,   6.5f,  80u, 605u },  /* #6  str1 A2  dotted qtr    */

    /* Phrase 3: root - m3 - P4 */
    { 0u, E2_HZ,   8.0f,  80u, 355u },  /* #7  str0 E2  quarter       */
    { 1u, G2_HZ,   9.0f,  80u, 355u },  /* #8  str1 G2  quarter       */
    { 0u, A2_HZ,  10.0f,  80u, 605u },  /* #9  str0 A2  half (held)   */

    /* Phrase 4: m3 - root (ending) */
    { 1u, G2_HZ,  12.0f,  80u, 355u },  /* #10 str1 G2  quarter       */
    { 0u, E2_HZ,  13.0f,  80u, 605u },  /* #11 str0 E2  dotted half   */
};

/* =========================================================================
 * Runtime timing tables — computed from BPM
 * ========================================================================= */

/** Absolute times (ms) computed at init */
static uint32_t s_fire_time_ms[DBL_NUM_NOTES];     /**< When solenoid fires    */
static uint32_t s_note_end_ms[DBL_NUM_NOTES];       /**< fire + burst + rest    */
static uint32_t s_motor_start_ms[DBL_NUM_NOTES];    /**< When motor can start   */
static uint32_t s_convergence_ms[DBL_NUM_NOTES];     /**< fire - motor_start     */

/** Total song duration (ms) */
static uint32_t s_song_duration_ms = 0u;

/* =========================================================================
 * Per-track state
 * ========================================================================= */

typedef struct {
    float    target_freq_hz;    /**< Current motor target             */
    uint8_t  next_note_idx;     /**< Index of next note for this track */
    uint8_t  fire_pending;      /**< Edge-triggered solenoid flag      */
    uint16_t current_burst_ms;  /**< Burst duration for current note   */
    uint8_t  has_target;        /**< 1 = target_freq_hz is valid       */
} TrackState_t;

static TrackState_t s_track[DBL_NUM_TRACKS];

/* =========================================================================
 * Scheduler state
 * ========================================================================= */

static DblState_t s_state     = DBL_STATE_IDLE;
static uint32_t   s_bpm       = DBL_DEFAULT_BPM;
static uint32_t   s_pending_bpm = 0u;  /**< Queued BPM change (0 = none) */
static uint32_t   s_start_ms  = 0u;    /**< Absolute time playback started */
static uint32_t   s_preroll_end_ms = 0u;
static uint32_t   s_global_note_idx = 0u; /**< Next global note to process */
static uint8_t    s_loop_enabled = 1u;

/* =========================================================================
 * Private helpers
 * ========================================================================= */

/**
 * @brief Build timing tables from current BPM.
 *
 * For each note: fire_time = beat_pos * (60000 / BPM)
 * motor_start = previous same-string note_end (or 0 for first note on track)
 * convergence = fire_time - motor_start
 */
static void ComputeTimingTables(void)
{
    float ms_per_beat = 60000.0f / (float)s_bpm;

    /* Track the last note_end for each string to compute motor_start */
    int32_t last_end[DBL_NUM_TRACKS] = { -1, -1 };

    uint32_t min_conv = 0xFFFFFFFFu;

    for (uint32_t i = 0u; i < DBL_NUM_NOTES; i++) {
        uint8_t trk = s_song[i].string_id;

        s_fire_time_ms[i] = (uint32_t)(s_song[i].beat_pos * ms_per_beat);
        s_note_end_ms[i]  = s_fire_time_ms[i] + s_song[i].burst_ms + s_song[i].rest_ms;

        if (last_end[trk] >= 0) {
            s_motor_start_ms[i] = (uint32_t)last_end[trk];
            if (s_fire_time_ms[i] > s_motor_start_ms[i]) {
                s_convergence_ms[i] = s_fire_time_ms[i] - s_motor_start_ms[i];
            } else {
                s_convergence_ms[i] = 0u;
            }
        } else {
            /* First note on this track — motor starts at t=0 (pre-roll) */
            s_motor_start_ms[i] = 0u;
            s_convergence_ms[i] = s_fire_time_ms[i];
        }

        if (s_convergence_ms[i] < min_conv && i > 0u) {
            min_conv = s_convergence_ms[i];
        }

        last_end[trk] = (int32_t)s_note_end_ms[i];
    }

    /* Song duration = last note_end */
    s_song_duration_ms = s_note_end_ms[DBL_NUM_NOTES - 1u];

    /* Warn if minimum convergence is low */
    if (min_conv < 800u) {
        printf("[DBL] WARNING: min convergence = %lu ms at %lu BPM\r\n",
               (unsigned long)min_conv, (unsigned long)s_bpm);
    }
}

/** Reset per-track state for start of song */
static void ResetTracks(void)
{
    for (uint8_t t = 0u; t < DBL_NUM_TRACKS; t++) {
        s_track[t].target_freq_hz  = 0.0f;
        s_track[t].next_note_idx   = 0u;
        s_track[t].fire_pending    = 0u;
        s_track[t].current_burst_ms = 80u;
        s_track[t].has_target      = 0u;
    }

    /* Build per-track "next note" indices by scanning the song in order
     * and assigning each note to its track's linked list.
     * next_note_idx for each track = index of the FIRST note for that track. */
    for (uint8_t t = 0u; t < DBL_NUM_TRACKS; t++) {
        s_track[t].next_note_idx = DBL_NUM_NOTES; /* sentinel: no notes */
        for (uint32_t i = 0u; i < DBL_NUM_NOTES; i++) {
            if (s_song[i].string_id == t) {
                s_track[t].next_note_idx = (uint8_t)i;
                break;
            }
        }
    }
    s_global_note_idx = 0u;
}

/** Find the next note for a given track after index `after`. */
static uint8_t FindNextNote(uint8_t track, uint8_t after)
{
    for (uint32_t i = (uint32_t)after + 1u; i < DBL_NUM_NOTES; i++) {
        if (s_song[i].string_id == track) {
            return (uint8_t)i;
        }
    }
    return DBL_NUM_NOTES; /* no more notes */
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void DblString_Init(uint32_t bpm)
{
    if (bpm < DBL_BPM_MIN) bpm = DBL_BPM_MIN;
    if (bpm > DBL_BPM_MAX) bpm = DBL_BPM_MAX;
    s_bpm = bpm;
    s_pending_bpm = 0u;

    ComputeTimingTables();
    ResetTracks();

    s_state = DBL_STATE_IDLE;
    s_start_ms = 0u;

    /* Log timing table */
    printf("[DBL] Init at %lu BPM (%.0f ms/beat), song=%lu ms\r\n",
           (unsigned long)s_bpm,
           (double)(60000.0f / (float)s_bpm),
           (unsigned long)s_song_duration_ms);
    printf("[DBL] Timing table:\r\n");
    for (uint32_t i = 0u; i < DBL_NUM_NOTES; i++) {
        printf("[DBL]   #%lu str%u %.1fHz  fire=%lu  end=%lu  motor=%lu  conv=%lu ms\r\n",
               (unsigned long)i,
               (unsigned)s_song[i].string_id,
               (double)s_song[i].freq_hz,
               (unsigned long)s_fire_time_ms[i],
               (unsigned long)s_note_end_ms[i],
               (unsigned long)s_motor_start_ms[i],
               (unsigned long)s_convergence_ms[i]);
    }
}

void DblString_SetBpm(uint32_t bpm)
{
    if (bpm < DBL_BPM_MIN) bpm = DBL_BPM_MIN;
    if (bpm > DBL_BPM_MAX) bpm = DBL_BPM_MAX;
    s_pending_bpm = bpm;
    printf("[DBL] BPM change to %lu queued (applied at loop boundary)\r\n",
           (unsigned long)bpm);
}

uint32_t DblString_GetBpm(void)
{
    return s_bpm;
}

void DblString_Start(uint32_t now_ms)
{
    ResetTracks();

    /* Set initial targets for both tracks (first note of each track) */
    for (uint8_t t = 0u; t < DBL_NUM_TRACKS; t++) {
        uint8_t idx = s_track[t].next_note_idx;
        if (idx < DBL_NUM_NOTES) {
            s_track[t].target_freq_hz  = s_song[idx].freq_hz;
            s_track[t].current_burst_ms = s_song[idx].burst_ms;
            s_track[t].has_target      = 1u;
        }
    }

    s_start_ms = now_ms;
    s_preroll_end_ms = now_ms + DBL_PREROLL_MS;
    s_state = DBL_STATE_PREROLL;

    printf("[DBL] Playback started (pre-roll %u ms)\r\n", (unsigned)DBL_PREROLL_MS);
}

void DblString_Stop(void)
{
    s_state = DBL_STATE_IDLE;
    for (uint8_t t = 0u; t < DBL_NUM_TRACKS; t++) {
        s_track[t].fire_pending = 0u;
        s_track[t].has_target   = 0u;
    }
    printf("[DBL] Playback stopped\r\n");
}

void DblString_Tick(uint32_t now_ms)
{
    if (s_state == DBL_STATE_IDLE || s_state == DBL_STATE_FINISHED) {
        return;
    }

    /* --- Pre-roll: wait for motors to settle --- */
    if (s_state == DBL_STATE_PREROLL) {
        if (now_ms >= s_preroll_end_ms) {
            s_state = DBL_STATE_PLAYING;
            s_start_ms = now_ms;  /* Reset start time so song_time=0 at play start */
            printf("[DBL] Pre-roll complete, playing\r\n");
        }
        return;
    }

    /* --- Playing state --- */
    uint32_t song_time = now_ms - s_start_ms;

    /* Process each track independently */
    for (uint8_t t = 0u; t < DBL_NUM_TRACKS; t++) {
        uint8_t idx = s_track[t].next_note_idx;
        if (idx >= DBL_NUM_NOTES) continue; /* track finished */

        /* 1. Motor start: update target freq when it's time to start moving */
        if (song_time >= s_motor_start_ms[idx] && !s_track[t].has_target) {
            s_track[t].target_freq_hz  = s_song[idx].freq_hz;
            s_track[t].current_burst_ms = s_song[idx].burst_ms;
            s_track[t].has_target      = 1u;
        }

        /* 2. Fire time: set solenoid flag */
        if (song_time >= s_fire_time_ms[idx] && !s_track[t].fire_pending
            && s_track[t].has_target) {
            s_track[t].fire_pending = 1u;

            /* Update global note index for status display */
            if (idx >= s_global_note_idx) {
                s_global_note_idx = idx;
            }
        }

        /* 3. Note end: advance to next note on this track */
        if (song_time >= s_note_end_ms[idx]) {
            uint8_t next = FindNextNote(t, idx);
            s_track[t].next_note_idx = next;
            s_track[t].has_target = 0u; /* will be set when motor_start_ms[next] arrives */

            /* If there IS a next note and motor_start has already passed, set target now */
            if (next < DBL_NUM_NOTES && song_time >= s_motor_start_ms[next]) {
                s_track[t].target_freq_hz  = s_song[next].freq_hz;
                s_track[t].current_burst_ms = s_song[next].burst_ms;
                s_track[t].has_target      = 1u;
            }
        }
    }

    /* --- End of song check --- */
    if (song_time >= s_song_duration_ms) {
        if (s_loop_enabled) {
            /* Apply pending BPM change at loop boundary */
            if (s_pending_bpm > 0u) {
                printf("[DBL] Applying BPM change: %lu -> %lu\r\n",
                       (unsigned long)s_bpm, (unsigned long)s_pending_bpm);
                s_bpm = s_pending_bpm;
                s_pending_bpm = 0u;
                ComputeTimingTables();
            }
            /* Restart */
            ResetTracks();
            s_start_ms = now_ms;

            /* Set initial targets for both tracks immediately */
            for (uint8_t t = 0u; t < DBL_NUM_TRACKS; t++) {
                uint8_t idx = s_track[t].next_note_idx;
                if (idx < DBL_NUM_NOTES) {
                    s_track[t].target_freq_hz   = s_song[idx].freq_hz;
                    s_track[t].current_burst_ms = s_song[idx].burst_ms;
                    s_track[t].has_target       = 1u;
                }
            }
            printf("[DBL] Loop restart at %lu BPM\r\n", (unsigned long)s_bpm);
        } else {
            s_state = DBL_STATE_FINISHED;
            printf("[DBL] Song finished\r\n");
        }
    }
}

uint8_t DblString_IsActive(void)
{
    return (s_state == DBL_STATE_PREROLL || s_state == DBL_STATE_PLAYING) ? 1u : 0u;
}

float DblString_GetTargetFreq(uint8_t track)
{
    if (track >= DBL_NUM_TRACKS) return 0.0f;
    if (!s_track[track].has_target) return 0.0f;
    return s_track[track].target_freq_hz;
}

uint8_t DblString_ShouldFire(uint8_t track)
{
    if (track >= DBL_NUM_TRACKS) return 0u;
    if (s_track[track].fire_pending) {
        s_track[track].fire_pending = 0u;
        return 1u;
    }
    return 0u;
}

uint16_t DblString_GetBurstMs(uint8_t track)
{
    if (track >= DBL_NUM_TRACKS) return 0u;
    return s_track[track].current_burst_ms;
}

uint32_t DblString_GetNoteIndex(void)
{
    return s_global_note_idx;
}

uint32_t DblString_GetNoteCount(void)
{
    return DBL_NUM_NOTES;
}

DblState_t DblString_GetState(void)
{
    return s_state;
}
