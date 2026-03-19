/**
 ******************************************************************************
 * @file    song_player.c
 * @brief   Event-driven song player — state machine implementation
 *
 * See song_player.h for the architecture overview.
 *
 * Note lifecycle state machine:
 *
 *   PREPARE ──> CONVERGE ──> FIRE ──> SUSTAIN ──> [next note]
 *      |            |          |          |
 *      |            |          |          +-> Look-ahead: pre-position motors
 *      |            |          |              for next note during sustain
 *      |            |          |
 *      |            |          +-> Solenoid fires, FFT blanked
 *      |            |
 *      |            +-> PI refines (FFT clean, no solenoid)
 *      |
 *      +-> Motor moves to LUT position; cold string gets excitation burst
 ******************************************************************************
 */

#include "song_player.h"
#include "song_scheduler.h"
#include "tuning_controller.h"
#include "inter_core_cm7.h"
#include "shared_memory.h"
#include "application_config.h"
#include "stm32h7xx_hal.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* =========================================================================
 * Compile-time constants (tuned for 100 BPM)
 * ========================================================================= */

#define SP_MAX_PREPARE_HOPS    5u     /**< Max hops waiting for motor arrival  */
#define SP_MAX_CONVERGE_HOPS   3u     /**< Max hops waiting for PI lock        */
#define SP_EXCITATION_BURST_MS 200u   /**< Cold-start excitation burst (ms)    */
#define SP_FFT_FLUSH_MS        340u   /**< 2 FFT hops to flush burst from win  */
#define SP_ACCEL_MARGIN_MS     50u    /**< Motor accel/decel overhead estimate  */
#define SP_MIN_RING_MS         340u   /**< Min ring before look-ahead (2 hops) */
#define SP_HOP_PERIOD_MS       170u   /**< ~1024/6000 * 1000                   */

/* =========================================================================
 * Private state
 * ========================================================================= */

static PlayerPhase_t s_phase   = PLAYER_IDLE;
static uint8_t       s_playing = 0u;

/* Current note info (cached from scheduler) */
static uint8_t  s_str1, s_str2;
static float    s_freq1, s_freq2;
static uint8_t  s_is_chord;
static uint8_t  s_motor_enable_mask;

/* Timing */
static uint16_t s_prepare_hop_count;
static uint16_t s_converge_hop_count;
static uint32_t s_ring_start_ms;
static uint32_t s_ring_duration_ms;

/* FFT blanking */
static uint32_t s_blanking_end_ms;
static uint8_t  s_blanking_active;

/* Cold start */
static uint8_t  s_need_excitation;
static uint8_t  s_excitation_fired;

/* Look-ahead */
static uint8_t  s_lookahead_started;

/* Chord convergence tracking */
static uint8_t  s_chord_partial_lock;
static uint8_t  s_chord_wait_hops;

/* Diagnostics */
static uint32_t s_timeout_count;
static uint32_t s_locked_count;

/* =========================================================================
 * Private helpers
 * ========================================================================= */

/**
 * @brief Cache current note info from the scheduler into local state.
 */
static void CacheCurrentNote(void)
{
    s_str1  = SongSched_GetStringId();
    s_freq1 = SongSched_GetTargetFreq();
    s_is_chord = (SongSched_GetStringCount() >= 2u);
    if (s_is_chord) {
        s_str2  = SongSched_GetStringId2();
        s_freq2 = SongSched_GetTargetFreq2();
    } else {
        s_str2  = 0u;
        s_freq2 = 0.0f;
    }
}

/**
 * @brief Send motor goto commands for active strings based on TuningCtrl state.
 */
static void SendMotorCommands(void)
{
    int32_t positions[NUM_MOTORS];
    memset(positions, 0, sizeof(positions));

    uint8_t move_mask = (1u << s_str1);
    positions[s_str1] = TuningCtrl_GetCmdSteps(s_str1);

    if (s_is_chord) {
        move_mask       |= (1u << s_str2);
        positions[s_str2] = TuningCtrl_GetCmdSteps(s_str2);
    }

    InterCore_SendMotorGotoBatch(move_mask, positions);
}

/**
 * @brief Enable only the motors needed for the current note.
 */
static void UpdateMotorEnables(void)
{
    uint8_t needed = (1u << s_str1);
    if (s_is_chord) needed |= (1u << s_str2);
    if (needed != s_motor_enable_mask) {
        InterCore_SendMotorEnableMask(needed);
        s_motor_enable_mask = needed;
    }
}

/**
 * @brief Start FFT blanking for a given duration.
 */
static void StartBlanking(uint32_t duration_ms)
{
    s_blanking_end_ms = HAL_GetTick() + duration_ms;
    s_blanking_active = 1u;
    TuningCtrl_SetBlanking(1);
}

/**
 * @brief Clear FFT blanking.
 */
static void ClearBlanking(void)
{
    if (s_blanking_active) {
        s_blanking_active = 0u;
        TuningCtrl_SetBlanking(0);
    }
}

/**
 * @brief Auto-clear blanking if the blanking window has expired.
 */
static void CheckBlankingExpiry(void)
{
    if (s_blanking_active && HAL_GetTick() >= s_blanking_end_ms) {
        ClearBlanking();
    }
}

/* =========================================================================
 * Phase handlers
 * ========================================================================= */

static void Hop_Prepare(void)
{
    /* First hop: enable motors, set FF position, send motor commands */
    if (s_prepare_hop_count == 0u) {
        UpdateMotorEnables();

        /* Run TuningCtrl to trigger note-change detection and set FF position */
        TuningCtrl_RunString(s_str1, s_freq1);
        if (s_is_chord) TuningCtrl_RunString(s_str2, s_freq2);
        SendMotorCommands();

        /* Determine if cold-start excitation is needed */
        const TuningStringState_t *st1 = TuningCtrl_GetStringState(s_str1);
        s_need_excitation = !st1->string_warm;
        if (s_is_chord) {
            const TuningStringState_t *st2 = TuningCtrl_GetStringState(s_str2);
            if (!st2->string_warm) s_need_excitation = 1u;
        }
        s_excitation_fired = 0u;
    }

    s_prepare_hop_count++;

    /* Check if all active motors have arrived */
    uint8_t arrived = !g_shared_memory.motor_is_moving[s_str1];
    if (s_is_chord) arrived &= !g_shared_memory.motor_is_moving[s_str2];

    /* Cold string: fire excitation burst once motor arrives */
    if (arrived && s_need_excitation && !s_excitation_fired) {
        const TuningStringState_t *st1 = TuningCtrl_GetStringState(s_str1);
        float ef1 = st1->expected_freq_hz;
        float f = (ef1 >= 40.0f && ef1 <= 800.0f) ? ef1 : 0.0f;

        if (s_is_chord) {
            const TuningStringState_t *st2 = TuningCtrl_GetStringState(s_str2);
            float ef2 = st2->expected_freq_hz;
            float f2 = (ef2 >= 40.0f && ef2 <= 800.0f) ? ef2 : 0.0f;

            uint8_t burst_mask = 0u;
            float burst_freqs[6] = {0.0f};
            if (f  > 1e-6f) { burst_mask |= (1u << s_str1); burst_freqs[s_str1] = f;  }
            if (f2 > 1e-6f) { burst_mask |= (1u << s_str2); burst_freqs[s_str2] = f2; }
            if (burst_mask) {
                InterCore_TriggerSolenoidHzBatch(burst_mask, burst_freqs,
                    SOLENOID_DEFAULT_DUTY_CYCLE, SOLENOID_MODE_TIMED,
                    (uint16_t)SP_EXCITATION_BURST_MS);
            }
        } else {
            if (f > 1e-6f) {
                InterCore_TriggerSolenoidHz(s_str1, f, SOLENOID_DEFAULT_DUTY_CYCLE,
                    SOLENOID_MODE_TIMED, (uint16_t)SP_EXCITATION_BURST_MS, 0u);
            }
        }

        StartBlanking(SP_EXCITATION_BURST_MS + SP_FFT_FLUSH_MS);
        s_excitation_fired = 1u;

        printf("[SP] Excitation burst: %u ms at %.1f Hz\r\n",
               (unsigned)SP_EXCITATION_BURST_MS, (double)f);
    }

    /* Wait for blanking to end after excitation */
    if (s_excitation_fired) {
        if (!s_blanking_active || HAL_GetTick() >= s_blanking_end_ms) {
            ClearBlanking();
            s_phase = PLAYER_CONVERGE;
            s_converge_hop_count = 0u;
            s_chord_partial_lock = 0u;
            s_chord_wait_hops    = 0u;
            return;
        }
        /* Still blanked — keep waiting */
        return;
    }

    /* Warm string: transition immediately when motor arrives */
    if (arrived && !s_need_excitation) {
        s_phase = PLAYER_CONVERGE;
        s_converge_hop_count = 0u;
        s_chord_partial_lock = 0u;
        s_chord_wait_hops    = 0u;
        return;
    }

    /* Timeout: force transition */
    if (s_prepare_hop_count >= SP_MAX_PREPARE_HOPS) {
        ClearBlanking();
        printf("[SP] PREPARE timeout (%u hops)\r\n", (unsigned)s_prepare_hop_count);
        s_phase = PLAYER_CONVERGE;
        s_converge_hop_count = 0u;
        s_chord_partial_lock = 0u;
        s_chord_wait_hops    = 0u;
    }
}

static void Hop_Converge(void)
{
    /* FFT is clean here — no solenoid active, blanking is off.
     * Run PI for each active string. */
    TuningCtrl_RunString(s_str1, s_freq1);
    if (s_is_chord) TuningCtrl_RunString(s_str2, s_freq2);
    SendMotorCommands();

    s_converge_hop_count++;

    /* Check lock states */
    uint8_t lock1 = TuningCtrl_GetLockState(s_str1);
    uint8_t lock2 = s_is_chord ? TuningCtrl_GetLockState(s_str2) : 1u;

    /* All locked -> fire */
    if (lock1 && lock2) {
        s_phase = PLAYER_FIRE;
        return;
    }

    /* Chord partial timeout: one locked, other hasn't */
    if (s_is_chord) {
        if ((lock1 || lock2) && !s_chord_partial_lock) {
            s_chord_partial_lock = 1u;
            s_chord_wait_hops    = 0u;
        }
        if (s_chord_partial_lock) {
            s_chord_wait_hops++;
            if (s_chord_wait_hops >= SONG_CHORD_PLUCK_TIMEOUT_HOPS) {
                printf("[SP] Chord partial lock (lock=%u/%u)\r\n",
                       (unsigned)lock1, (unsigned)lock2);
                s_phase = PLAYER_FIRE;
                return;
            }
        }
    }

    /* Full timeout */
    if (s_converge_hop_count >= SP_MAX_CONVERGE_HOPS) {
        printf("[SP] CONVERGE timeout (%u hops, lock=%u/%u)\r\n",
               (unsigned)s_converge_hop_count,
               (unsigned)lock1, (unsigned)lock2);
        s_phase = PLAYER_FIRE;
    }
}

static void Hop_Fire(void)
{
    /* Determine burst duration based on note length */
    uint32_t note_idx = SongSched_GetNoteIndex();
    uint32_t note_dur = SongSched_GetScaledDurationMs(note_idx);
    uint16_t burst_ms = (note_dur <= 1200u) ? 250u : 500u;

    /* Get solenoid frequency: use measured if locked, else target */
    const TuningStringState_t *st1 = TuningCtrl_GetStringState(s_str1);
    float f1 = st1->lock_state ? st1->last_valid_freq_hz : s_freq1;
    if (f1 < 40.0f || f1 > 800.0f) f1 = 0.0f;

    if (s_is_chord) {
        const TuningStringState_t *st2 = TuningCtrl_GetStringState(s_str2);
        float f2 = st2->lock_state ? st2->last_valid_freq_hz : s_freq2;
        if (f2 < 40.0f || f2 > 800.0f) f2 = 0.0f;

        uint8_t sol_mask  = 0u;
        float   freqs[6]  = {0.0f};
        if (f1 > 1e-6f) { sol_mask |= (1u << s_str1); freqs[s_str1] = f1; }
        if (f2 > 1e-6f) { sol_mask |= (1u << s_str2); freqs[s_str2] = f2; }
        if (sol_mask) {
            InterCore_TriggerSolenoidHzBatch(sol_mask, freqs,
                SOLENOID_DEFAULT_DUTY_CYCLE, SOLENOID_MODE_TIMED, burst_ms);
        }

        /* Diagnostics */
        if (st1->lock_state && st2->lock_state) s_locked_count++;
        else s_timeout_count++;

        printf("[SP] FIRE chord str%u+%u @%.1f/%.1f Hz burst=%ums (lock=%u/%u)\r\n",
               (unsigned)s_str1, (unsigned)s_str2,
               (double)f1, (double)f2, (unsigned)burst_ms,
               (unsigned)st1->lock_state, (unsigned)st2->lock_state);
    } else {
        if (f1 > 1e-6f) {
            InterCore_TriggerSolenoidHz(s_str1, f1, SOLENOID_DEFAULT_DUTY_CYCLE,
                SOLENOID_MODE_TIMED, burst_ms, 0u);
        }

        if (st1->lock_state) s_locked_count++;
        else s_timeout_count++;

        printf("[SP] FIRE str%u @%.1f Hz burst=%ums (lock=%u)\r\n",
               (unsigned)s_str1, (double)f1, (unsigned)burst_ms,
               (unsigned)st1->lock_state);
    }

    /* Set FFT blanking for burst + flush */
    StartBlanking((uint32_t)burst_ms + SP_FFT_FLUSH_MS);

    /* Record sustain start */
    s_ring_start_ms    = HAL_GetTick();
    s_ring_duration_ms = note_dur;
    s_lookahead_started = 0u;

    /* Transition to SUSTAIN immediately */
    s_phase = PLAYER_SUSTAIN;
}

static void Hop_Sustain(void)
{
    uint32_t now          = HAL_GetTick();
    uint32_t ring_elapsed = now - s_ring_start_ms;

    /* Auto-clear blanking when window expires */
    CheckBlankingExpiry();

    /* PI maintenance — only when FFT is clean and look-ahead hasn't started */
    if (!s_lookahead_started) {
        if (!s_blanking_active) {
            /* Clean FFT — run PI to maintain pitch */
            TuningCtrl_RunString(s_str1, s_freq1);
            if (s_is_chord) TuningCtrl_RunString(s_str2, s_freq2);
        }
        /* Always send motor commands to hold position */
        SendMotorCommands();
    }

    /* ------------------------------------------------------------------ */
    /* Look-ahead: pre-position motors for next note during sustain        */
    /* ------------------------------------------------------------------ */
    if (!s_lookahead_started) {
        const SongNote_t *next = SongSched_PeekNextNote();
        if (next != NULL) {
            /* Calculate motor travel time for the next note */
            float next_ff1 = TuningCtrl_GetFFSteps(next->string_id,
                                                     next->target_freq_hz);
            int32_t cur1   = TuningCtrl_GetCmdSteps(next->string_id);
            float step_dist = fabsf(next_ff1 - (float)cur1);
            float max_travel_ms = (step_dist / TUNING_MAX_STEP_RATE_SPS)
                                  * 1000.0f + (float)SP_ACCEL_MARGIN_MS;

            /* For chords, use the slower string's travel time */
            if (next->num_strings >= 2u) {
                float next_ff2 = TuningCtrl_GetFFSteps(next->string_id2,
                                                        next->target_freq_hz2);
                int32_t cur2   = TuningCtrl_GetCmdSteps(next->string_id2);
                float dist2    = fabsf(next_ff2 - (float)cur2);
                float travel2  = (dist2 / TUNING_MAX_STEP_RATE_SPS)
                                 * 1000.0f + (float)SP_ACCEL_MARGIN_MS;
                if (travel2 > max_travel_ms) max_travel_ms = travel2;
            }

            /* Total prepare window = travel + converge budget + safety */
            float prepare_window_ms = max_travel_ms
                + (float)SP_MAX_CONVERGE_HOPS * (float)SP_HOP_PERIOD_MS
                + (float)SP_HOP_PERIOD_MS;

            /* Never start look-ahead before minimum ring time */
            float earliest_ms = (float)SP_MIN_RING_MS;
            float start_ms    = (float)s_ring_duration_ms - prepare_window_ms;
            if (start_ms < earliest_ms) start_ms = earliest_ms;

            if ((float)ring_elapsed >= start_ms) {
                /* Send motors to next note's FF positions */
                int32_t positions[NUM_MOTORS];
                memset(positions, 0, sizeof(positions));

                uint8_t move_mask = (1u << next->string_id);
                int32_t pos1 = (int32_t)roundf(next_ff1);
                if (pos1 < TUNING_STEPS_MIN_DEFAULT) pos1 = TUNING_STEPS_MIN_DEFAULT;
                if (pos1 > TUNING_STEPS_MAX_DEFAULT) pos1 = TUNING_STEPS_MAX_DEFAULT;
                positions[next->string_id] = pos1;

                if (next->num_strings >= 2u) {
                    float nff2 = TuningCtrl_GetFFSteps(next->string_id2,
                                                        next->target_freq_hz2);
                    int32_t pos2 = (int32_t)roundf(nff2);
                    if (pos2 < TUNING_STEPS_MIN_DEFAULT) pos2 = TUNING_STEPS_MIN_DEFAULT;
                    if (pos2 > TUNING_STEPS_MAX_DEFAULT) pos2 = TUNING_STEPS_MAX_DEFAULT;
                    move_mask |= (1u << next->string_id2);
                    positions[next->string_id2] = pos2;
                }

                /* Enable motors for next note if strings change */
                uint8_t next_mask = (1u << next->string_id);
                if (next->num_strings >= 2u)
                    next_mask |= (1u << next->string_id2);
                uint8_t combined = s_motor_enable_mask | next_mask;
                if (combined != s_motor_enable_mask) {
                    InterCore_SendMotorEnableMask(combined);
                    s_motor_enable_mask = combined;
                }

                InterCore_SendMotorGotoBatch(move_mask, positions);
                s_lookahead_started = 1u;

                printf("[SP] LOOKAHEAD note %lu: motor dist=%.0f steps\r\n",
                       (unsigned long)SongSched_GetNoteIndex(),
                       (double)step_dist);
            }
        }
    }

    /* ------------------------------------------------------------------ */
    /* Check if ring time has expired                                       */
    /* ------------------------------------------------------------------ */
    if (ring_elapsed >= s_ring_duration_ms) {
        SongSched_Advance();

        if (!SongSched_IsActive()) {
            printf("[SP] Song complete (locked=%lu, timeout=%lu)\r\n",
                   (unsigned long)s_locked_count,
                   (unsigned long)s_timeout_count);
            SongPlayer_Stop();
            return;
        }

        /* Cache the new note */
        CacheCurrentNote();
        ClearBlanking();

        if (s_lookahead_started) {
            /* Motor already at target — trigger note-change detection and
             * go straight to CONVERGE (skip PREPARE). */
            TuningCtrl_RunString(s_str1, s_freq1);
            if (s_is_chord) TuningCtrl_RunString(s_str2, s_freq2);
            SendMotorCommands();
            UpdateMotorEnables();

            s_phase = PLAYER_CONVERGE;
            s_converge_hop_count = 0u;
            s_chord_partial_lock = 0u;
            s_chord_wait_hops    = 0u;
        } else {
            /* No look-ahead — go through full PREPARE */
            s_phase = PLAYER_PREPARE;
            s_prepare_hop_count = 0u;
            s_excitation_fired  = 0u;
            s_need_excitation   = 0u;

            /* Check cold-start need */
            const TuningStringState_t *st1 = TuningCtrl_GetStringState(s_str1);
            s_need_excitation = !st1->string_warm;
            if (s_is_chord) {
                const TuningStringState_t *st2 = TuningCtrl_GetStringState(s_str2);
                if (!st2->string_warm) s_need_excitation = 1u;
            }
        }
    }
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void SongPlayer_Init(void)
{
    s_phase             = PLAYER_IDLE;
    s_playing           = 0u;
    s_motor_enable_mask = 0u;
    s_blanking_active   = 0u;
    s_timeout_count     = 0u;
    s_locked_count      = 0u;
}

void SongPlayer_Start(uint8_t song_id)
{
    /* Reset tuning controller and initialise scheduler */
    TuningCtrl_ResetAll();
    SongSched_InitSong(song_id);

    if (!SongSched_IsActive()) {
        printf("[SP] No notes in song %u\r\n", (unsigned)song_id);
        return;
    }

    /* Reset diagnostics */
    s_timeout_count = 0u;
    s_locked_count  = 0u;

    /* Cache first note */
    CacheCurrentNote();

    /* Enter PREPARE for the first note */
    s_phase             = PLAYER_PREPARE;
    s_prepare_hop_count = 0u;
    s_excitation_fired  = 0u;
    s_blanking_active   = 0u;
    s_lookahead_started = 0u;

    /* First note is always cold */
    s_need_excitation = 1u;

    s_playing = 1u;
    s_motor_enable_mask = 0u;  /* UpdateMotorEnables will set on first hop */

    printf("[SP] Start song %u (%lu notes)\r\n",
           (unsigned)song_id,
           (unsigned long)SongSched_GetNoteCount());
}

void SongPlayer_Stop(void)
{
    s_playing = 0u;
    s_phase   = PLAYER_IDLE;
    ClearBlanking();

    /* Disable all motors */
    if (s_motor_enable_mask != 0u) {
        InterCore_SendMotorEnableMask(0u);
        s_motor_enable_mask = 0u;
    }

    printf("[SP] Stopped\r\n");
}

void SongPlayer_Hop(void)
{
    if (!s_playing) return;

    /* Microstep safeguard — verify configured motors are in 1/16 mode */
    for (uint8_t m = 0u; m < 4u; m++) {
        if (g_shared_memory.motor_microstep_status[m]
            != TUNING_REQUIRED_MICROSTEP_MODE)
        {
            printf("[SP] *** HALT: Motor %u microstep=%u, expected %u ***\r\n",
                   (unsigned)m,
                   (unsigned)g_shared_memory.motor_microstep_status[m],
                   (unsigned)TUNING_REQUIRED_MICROSTEP_MODE);
            SongPlayer_Stop();
            return;
        }
    }

    /* Song ended externally */
    if (!SongSched_IsActive()) {
        SongPlayer_Stop();
        return;
    }

    switch (s_phase) {
        case PLAYER_PREPARE:  Hop_Prepare();  break;
        case PLAYER_CONVERGE: Hop_Converge(); break;
        case PLAYER_FIRE:     Hop_Fire();     break;
        case PLAYER_SUSTAIN:  Hop_Sustain();  break;
        default:              break;
    }
}

PlayerPhase_t SongPlayer_GetPhase(void)
{
    return s_phase;
}

uint8_t SongPlayer_IsPlaying(void)
{
    return s_playing;
}

uint32_t SongPlayer_GetTimeoutCount(void)
{
    return s_timeout_count;
}

uint32_t SongPlayer_GetLockedCount(void)
{
    return s_locked_count;
}
