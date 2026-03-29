/**
 ******************************************************************************
 * @file    calibration_controller.c
 * @brief   Guitar string fret calibration with PI lock-in and SRAM persistence
 *
 * Calibration sequence (per string)
 * -----------------------------------
 *
 *  STR_INIT
 *    Motor homes to position 0 and the user is prompted to pluck the string.
 *
 *  WAIT_PLUCK
 *    FFT-based pluck detection — confirms the string is live at the expected
 *    open-string frequency.  Times out after CAL_PLUCK_TIMEOUT hops (~4 s).
 *
 *  SOL_BURST  (primary)
 *    Fires the solenoid for CAL_SOL_DURATION_MS (1 s) at the PREVIOUS fret's
 *    resonant frequency.  This provides a long, high-energy ring that outlasts
 *    the motor transit and PI settling time.
 *    SIMULTANEOUSLY starts the motor moving at full speed toward the NEXT
 *    fret's LUT position so it travels during the burst and wait period.
 *    FFT sensing is inhibited while the solenoid is active.
 *    After CAL_BURST_WAIT_HOPS (8 hops) the FFT window is fully clear.
 *
 *  BURST_REENABLE
 *    Restores fft_sensing_enabled = 1.  PI state is reset here.
 *
 *  PI_LOCK
 *    The PI controller fine-tunes the motor position using FFT feedback while
 *    the string is still ringing.  The motor should already be near the LUT
 *    target; the PI corrects the remaining error.
 *
 *    Lock-band leeway: the PI only accumulates outside CAL_LOCK_BAND_HZ
 *    (0.6 Hz), and motor commands are suppressed below TUNING_MIN_MOVE_STEPS
 *    (2 microsteps) to prevent hunting — both constants from tuning_controller.h.
 *
 *    "Last valid frequency" tracking:
 *    Every hop the last accepted FFT reading is saved as s_last_valid_freq.
 *    If the string decays (CAL_DEAD_HOPS consecutive hops with no signal),
 *    the controller performs an EARLY EXIT:
 *      1. Fire solenoid for CAL_REEXCITE_DURATION_MS (300 ms) at s_last_valid_freq.
 *         This re-excites the string exactly at its current resonance rather
 *         than at the target fret frequency, maximising energy transfer.
 *      2. Re-issue the motor goto so it continues the remaining travel.
 *      3. After CAL_REEXCITE_WAIT_HOPS FFT clears, PI_LOCK resumes.
 *    Up to CAL_MAX_RETRIES early exits are attempted before recording the
 *    best available measurement and moving on.
 *
 *  RECORD
 *    Saves (actual_motor_steps, measured_freq) into s_buf.
 *    On success: use current FFT reading.
 *    On partial success: fall back to s_last_valid_freq with current position.
 *    On failure: fall back to (expected_steps, expected_freq) as estimate.
 *    Advances s_fret and loops back to SOL_BURST for the next fret.
 *
 *  BUILD
 *    Derives a 7-point LUT (Cal_BuildTable) and a quadratic least-squares
 *    polynomial steps = a + b·f + c·f² (Cal_FitPolynomial) from the 7
 *    measured points (frets 1–7) plus the fret-0 anchor (0 steps, open freq).
 *    Saves both to backup SRAM (Cal_SaveToSRAM).
 *    Returns the motor to position 0.
 *
 * Motor travel strategy
 * ---------------------
 *  The motor starts moving toward the next LUT position at the beginning of
 *  SOL_BURST (during the burst and wait).  At 3000 steps/sec a 1600-step
 *  fret transition takes ≈ 533 ms; the wait lasts 8 × 170 ms = 1360 ms,
 *  so the motor arrives before the FFT re-opens.  The motor's own
 *  braking-distance logic (proportional deceleration in motor_control.c)
 *  handles the final approach, so no explicit slow-phase is needed.
 *
 * FFT gating
 * ----------
 *  fft_sensing_enabled (extern in fft_processing.h) is cleared to 0 before
 *  each solenoid burst.  While 0, FFT_ProcessBlock() returns early after
 *  zeroing active_string_count so contaminated samples are never surfaced.
 *
 * Persistent storage — backup SRAM (0x38800000, 4 KB, VBAT-backed)
 * -----------------------------------------------------------------
 *  Call Cal_LoadFromSRAM() at boot to restore a previous calibration.
 ******************************************************************************
 */

#include "calibration_controller.h"
#include "tuning_controller.h"      /* LUT arrays + TUNING_* constants    */
#include "fft_processing.h"         /* active_frequencies[], fft_sensing_enabled */
#include "inter_core_cm7.h"
#include "shared_memory.h"
#include "application_config.h"
#include "stm32h7xx_hal.h"          /* pulls in hal_rcc.h + hal_pwr_ex.h  */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* =========================================================================
 * Calibration parameters
 * ========================================================================= */

/** Number of half-semitone measurement targets per string.
 *  Uniform sweep from −1 to +7 semitones in 0.5-semitone steps.
 *  idx 0 = −1.0 st, idx 2 = 0 st (open), idx 16 = +7.0 st. */
#define CAL_NUM_TARGETS             17u

/** Hops to wait after FFT re-enable before recording measurement.
 *  Lets the string vibration stabilise after motor arrival. */
#define CAL_SETTLE_HOPS             4u

/** Motor max speed (steps/sec) during the inter-fret transit.
 *  At 3000 steps/sec a 1600-step move takes ≈ 533 ms — well within the
 *  1360 ms burst+wait window before the PI starts. */
#define CAL_MOTOR_SPEED_FAST        3000

/** Hops to wait after homing before listening for the initial pluck. */
#define CAL_HOME_SETTLE_HOPS        6u

/** Lock-band deadband (Hz).  PI only accumulates outside this band. */
#define CAL_LOCK_BAND_HZ            0.6f

/** Cents-based lock accuracy for OPEN_PI_LOCK and HOME_PI_LOCK.
 *  5 cents = 0.05 semitones.  More musically meaningful than a fixed Hz band
 *  since the same Hz error is a larger pitch deviation at low frequencies. */
#define CAL_LOCK_CENTS              5.0f

/** Motor position (steps) within which we consider the string "at home". */
#define CAL_HOME_POS_THRESHOLD      50

/** Max hops to wait for motor to reach home before proceeding anyway.
 *  Worst case: D3 at 9000 steps / 3000 steps/sec = 3.0 s → 18 hops.
 *  Add 12 hops for acceleration/braking margin → 30 hops total. */
#define CAL_HOME_TIMEOUT_HOPS       30u

/** Max hops to wait for the user to pluck + collection window.
 *  ~4 s for user to pluck (24 hops) + 12 hops collection = 36 hops total. */
#define CAL_PLUCK_TIMEOUT           36u

/** Hops to collect after first pluck detection before committing (~2 s). */
#define CAL_PLUCK_CONFIRM_HOPS      12u

/** Minimum readings in collection buffer before an early-exit on decay is allowed. */
#define CAL_PLUCK_CONFIRM_MIN       3u

/** Consecutive silent hops during collection that trigger early commit. */
#define CAL_PLUCK_DEAD_HOPS         3u

/** Hz radius for vote clustering — readings within this band count as the same frequency. */
#define CAL_PLUCK_VOTE_BAND_HZ      5.0f

/** Primary solenoid burst duration (ms): 1 s gives long ring energy. */
#define CAL_SOL_DURATION_MS         1000u

/** Re-excite burst duration (ms): shorter, just enough to restart ring. */
#define CAL_REEXCITE_DURATION_MS    300u

/** Solenoid duty cycle (%). */
#define CAL_SOLENOID_DUTY           50u

/** Hops to wait after primary burst before re-enabling FFT.
 *  Minimum = ceil((CAL_SOL_DURATION_MS + FFT_window_ms) / hop_ms)
 *           = ceil((1000 + 341) / 170) = 8. */
#define CAL_BURST_WAIT_HOPS         8u

/** Hops to wait after re-excite burst before re-enabling FFT.
 *  = ceil((300 + 341) / 170) = 4. */
#define CAL_REEXCITE_WAIT_HOPS      4u

/** PI proportional gain (microsteps / Hz). */
#define CAL_PI_KP                   3.0f

/** PI integral gain (microsteps / Hz / s). */
#define CAL_PI_KI                   1.5f

/** Maximum PI trim authority (±microsteps). */
#define CAL_PI_MAX_TRIM             600.0f

/** Max PI_LOCK hops before triggering re-excite or giving up. */
#define CAL_PI_LOCK_TIMEOUT         25u

/** Consecutive hops with no FFT signal before the early-exit re-excite fires. */
#define CAL_DEAD_HOPS               3u

/** Consecutive locked hops required to confirm PI convergence. */
#define CAL_LOCK_CONSEC_HOPS        2u

/** Max re-excite retries per fret before accepting a best-estimate recording. */
#define CAL_MAX_RETRIES             3u

/** Max re-excite retries during HOME_PI_LOCK. */
#define CAL_HOME_MAX_RETRIES        5u

/** Tight lock-band for HOME_PI_LOCK (Hz). Tighter than TUNING_LOCK_BAND_IN_HZ
 *  (1.0 Hz) so the string must settle to within 0.6 Hz before home is declared. */
#define CAL_HOME_LOCK_BAND_HZ       0.6f

/** Continuity tracking window (Hz). Once the first valid reading is captured,
 *  subsequent hops search within ±CAL_TRACK_BAND_HZ of s_last_valid_freq.
 *  Chosen > max motor-induced Hz-per-hop (~3.5 Hz at 3000 steps/sec) but
 *  well below adjacent-string minimum separation (~15 Hz at high frets). */
#define CAL_TRACK_BAND_HZ           5.0f

/** Re-excite burst duration (ms) used during HOME_PI_LOCK.
 *  Longer than CAL_REEXCITE_DURATION_MS to give the string a strong ring
 *  with enough decay time for the PI to converge at the open frequency.
 *  Wait hops = ceil((700 + 341) / 170) = 7 → reuse CAL_REEXCITE_WAIT_HOPS
 *  is slightly short; HOME burst re-enables after CAL_BURST_WAIT_HOPS (8). */
#define CAL_HOME_REEXCITE_MS        700u

/** Motor-position error (steps) that triggers a stall warning. */
#define CAL_STALL_WARN_STEPS        250

/** Sample buffer capacity.  17 measurement points + headroom. */
#define CAL_MAX_SAMPLES             20u

/** Minimum valid samples required to build the LUT. */
#define CAL_MIN_SAMPLES             ((uint16_t)TUNING_FF_NUM_POINTS)

/* Backup SRAM base address and magic word (STM32H7, VBAT-backed, 4 KB). */

#define CAL_PERSIST_MAGIC           0xCA115701UL


//Moving to Flash
#define CAL_FLASH_ADDR   ((uint32_t)0x080E0000)
#define CAL_FLASH_SECTOR FLASH_SECTOR_7

/* Open-string frequencies (Hz). Order: E2 A2 D3 G3 B3 E4. */
static const float s_open_freq[NUM_STRINGS] = {
    E2_HZ, A2_HZ, D3_HZ, G3_HZ, B3_HZ, E4_HZ
};

/**
 * Hard upper limit on motor position during calibration, per string.
 * Derived from LUT RUN 1 fret-6 measured steps + 1500-step safety buffer.
 * Unconfigured motors (B3, E4) use a conservative placeholder.
 */
static const int32_t s_cal_steps_max[NUM_STRINGS] = {
    5000,  /* E2 — LUT max 3385 + 1500 */
    7500,  /* A2 — LUT max 5955 + 1500 */
    9000,  /* D3 — LUT max 7499 + 1500 */
    8200,  /* G3 — LUT max 6646 + 1500 */
    2200,  /* B3 — placeholder (motor unconfigured) */
    2200,  /* E4 — placeholder (motor unconfigured) */
};

/**
 * Compute the target frequency for a half-semitone measurement index.
 * idx 0 = −1.0 st, idx 1 = −0.5 st, idx 2 = 0 st (open), …, idx 16 = +7.0 st.
 * Uses equal-temperament: f = f_open × 2^(semitones / 12).
 */
static inline float Cal_TargetFreq(uint8_t str, uint8_t idx) {
    float semitones = ((float)idx - 2.0f) * 0.5f;
    return s_open_freq[str] * powf(2.0f, semitones / 12.0f);
}

/* Forward declaration: calibration invalidates tune offsets for reworked strings. */
static uint8_t Tune_ClearOffsetsMask(uint8_t string_mask);

/* =========================================================================
 * Persistent storage layout
 * ========================================================================= */

typedef struct {
    uint32_t magic;
    float    ff_freq [NUM_STRINGS][TUNING_FF_NUM_POINTS];
    float    ff_steps[NUM_STRINGS][TUNING_FF_NUM_POINTS];
    float    poly_a  [NUM_STRINGS];
    float    poly_b  [NUM_STRINGS];
    float    poly_c  [NUM_STRINGS];
    /* Semitone-domain coefficients (added for v2 controller) */
    float    semi_ff  [NUM_STRINGS][3];   /**< tuning_ff_semi_coeff[str][a,b,c] */
    float    semi_plant[NUM_STRINGS][3];  /**< tuning_plant_coeff[str][a,b,c]   */
    uint32_t checksum;
} __attribute__((packed)) CalPersist_t;

/* =========================================================================
 * Public polynomial coefficients
 * ========================================================================= */
float cal_poly_coeff[NUM_STRINGS][3] = { {0} };

/* =========================================================================
 * Private types
 * ========================================================================= */

typedef enum {
    CAL_PHASE_IDLE          = 0,
    CAL_PHASE_STR_INIT,       /**< Enable motor, prompt user to pluck        */
    CAL_PHASE_WAIT_PLUCK,     /**< Wait for FFT pluck (any fret)             */
    CAL_PHASE_WAIT_HOME,      /**< Wait for motor to reach position 0        */
    CAL_PHASE_OPEN_SOL_BURST, /**< Fire solenoid at nominal open freq        */
    CAL_PHASE_OPEN_BURST_REENABLE, /**< Restore FFT for open lock            */
    CAL_PHASE_OPEN_PI_LOCK,   /**< PI targets open freq, finds true pos 0    */
    CAL_PHASE_OPEN_DONE,      /**< Record offset, proceed to measurement loop */
    CAL_PHASE_WAIT,           /**< Generic hop-count wait                    */
    CAL_PHASE_SOL_BURST,      /**< Fire solenoid + start motor transit       */
    CAL_PHASE_BURST_REENABLE, /**< Restore FFT, settle, then record           */
    CAL_PHASE_RECORD,         /**< Save (steps, freq) and advance target     */
    CAL_PHASE_BUILD,          /**< Build LUT + polynomial, save SRAM         */
    CAL_PHASE_HOME_SOL_BURST, /**< Fire solenoid + command motor toward 0     */
    CAL_PHASE_HOME_BURST_REENABLE, /**< Restore FFT, reset PI for home       */
    CAL_PHASE_HOME_PI_LOCK,   /**< PI targets open-string frequency           */
    CAL_PHASE_HOME_DONE,      /**< Advance to next string or finish           */
    CAL_PHASE_DONE,
    CAL_PHASE_ERROR
} CalPhase_t;

typedef struct { float steps; float freq; } CalPoint_t;

/* =========================================================================
 * Private state
 * ========================================================================= */

static CalPhase_t  s_phase             = CAL_PHASE_IDLE;
static CalStatus_t s_status            = CAL_STATUS_IDLE;
static uint8_t     s_str               = 0u;
static uint8_t     s_string_mask       = 0x0Fu;

/* Per-fret state
 * s_fret is the half-semitone measurement TARGET (1..CAL_NUM_TARGETS).
 * The solenoid fires at fret (s_fret-1) to excite the string,
 * and the motor moves to s_fret's LUT position. */
static uint8_t     s_fret              = 0u;
static int32_t     s_steps_expected    = 0;   /* LUT target steps for s_fret */

/* Burst control */
static float       s_burst_freq        = 0.0f; /* Hz for the upcoming burst   */
static uint8_t     s_is_reexciting     = 0u;   /* 0=primary burst, 1=re-excite */

/* Retry and pluck tracking */
static uint8_t     s_retry_cnt         = 0u;
static uint8_t     s_pluck_timeout_cnt = 0u;
static uint8_t     s_pluck_detected    = 0u;
static float       s_open_detected_freq   = 0.0f; /* Hz detected at pluck time            */

/* Pluck confidence collection (WAIT_PLUCK phase) */
static float       s_pluck_buf[CAL_PLUCK_CONFIRM_HOPS]; /* per-hop frequency readings */
static uint8_t     s_pluck_buf_n       = 0u;   /* readings stored                 */
static uint8_t     s_pluck_collecting  = 0u;   /* 1 while collecting post-detect  */
static uint8_t     s_pluck_collect_cnt = 0u;   /* hops since first detection      */
static uint8_t     s_pluck_dead_hops   = 0u;   /* consecutive silent hops in coll */
static uint8_t     s_home_timeout_cnt     = 0u;   /* Hops waited in WAIT_HOME             */
static int32_t     s_open_position_offset = 0;    /* Measured open-string motor offset    */
static int32_t     s_open_homing_target   = 0;    /* FF-estimated motor pos for open freq */

/* PI state (inline controller, reset per fret) */
static float       s_pi_integral       = 0.0f;
static int32_t     s_pi_cmd_steps      = 0;
static uint8_t     s_pi_hop_cnt        = 0u;
static uint8_t     s_pi_lock_consec    = 0u;

/* Last-valid-frequency tracking (for early-exit re-excite) */
static float       s_last_valid_freq   = 0.0f;
static uint8_t     s_pi_no_sig_cnt     = 0u;   /* consecutive no-signal hops  */

/* Dynamic trim cap state (OPEN_PI_LOCK only)
 * Tracks whether the PI is actually converging hop-over-hop.
 * If converging: trim allowed up to cal_steps_max (PI drives as far as needed).
 * If not converging for CAL_OPEN_STALL_HOPS consecutive hops: hard cap at
 * CAL_PI_MAX_TRIM to protect against stall / dead string / false FFT reading. */
static float       s_pi_last_abs_error   = 9999.0f; /* abs error from previous hop     */
static uint8_t     s_pi_stall_cnt        = 0u;       /* consecutive non-converging hops */
#define CAL_OPEN_STALL_HOPS  3u                      /* hops before hard cap re-engages */

/* Generic wait */
static uint16_t    s_wait_cnt          = 0u;
static CalPhase_t  s_after_wait        = CAL_PHASE_IDLE;

/* Configurable WAIT_HOME destination (mirrors s_after_wait pattern) */
static CalPhase_t  s_after_home        = CAL_PHASE_SOL_BURST;

/* Sample buffer */
static CalPoint_t  s_buf[CAL_MAX_SAMPLES];
static uint16_t    s_buf_n             = 0u;

/* =========================================================================
 * Private helpers
 * ========================================================================= */

static inline float CalClampf(float v, float lo, float hi){
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/** Compute pitch error in cents: 1200 × log2(meas / target). */
static inline float CalErrorCents(float meas_f, float target_f)
{
    if (meas_f <= 0.0f || target_f <= 0.0f) return 9999.0f;
    return 1200.0f * log2f(meas_f / target_f);
}

static void WaitThen(uint16_t hops, CalPhase_t next){
    s_wait_cnt   = hops;
    s_after_wait = next;
    s_phase      = CAL_PHASE_WAIT;
}

static void StopSolenoid(uint8_t str_id){
    g_shared_memory.solenoid_id          = str_id;
    g_shared_memory.motor_command_flags |= CMD_STOP_SOLENOID;
    InterCore_NotifyCM4();
}

/**
 * @brief Scan active_frequencies[] for the peak nearest 'target'.
 * @param target      Expected frequency (Hz)
 * @param search_band Accept if |measured − target| ≤ search_band
 * @param out_freq    Best match; 0.0f if nothing within band
 */
static void CalGetFreq(float target, float search_band, float *out_freq){
    *out_freq = 0.0f;
    float best_err = search_band + 1.0f;
    for (uint8_t i = 0u; i < active_string_count; i++) {
        float err = fabsf(active_frequencies[i] - target);
        if (err < best_err) {
            best_err  = err;
            *out_freq = active_frequencies[i];
        }
    }
}

/**
 * @brief Find the most consistently occurring frequency in a collection buffer.
 *
 * Each candidate reading votes for itself plus any other reading within
 * CAL_PLUCK_VOTE_BAND_HZ.  The centroid of the winning cluster is returned.
 * This lets the fundamental (which dominates the sustained ring) beat a
 * short-lived transient harmonic that may appear in the first hop or two.
 *
 * @param buf          Array of frequency readings (Hz)
 * @param n            Number of valid entries in buf
 * @param nominal_freq Fallback if buf is empty
 */
static float Cal_PluckConsensus(const float *buf, uint8_t n, float nominal_freq)
{
    if (n == 0u) return nominal_freq;
    if (n == 1u) return buf[0];

    uint8_t best_votes = 0u;
    float   best_sum   = buf[0];
    uint8_t best_cnt   = 1u;

    for (uint8_t i = 0u; i < n; i++) {
        uint8_t votes = 0u;
        float   sum   = 0.0f;
        for (uint8_t k = 0u; k < n; k++) {
            if (fabsf(buf[k] - buf[i]) <= CAL_PLUCK_VOTE_BAND_HZ) {
                votes++;
                sum += buf[k];
            }
        }
        if (votes > best_votes) {
            best_votes = votes;
            best_sum   = sum;
            best_cnt   = votes;
        }
    }
    return best_sum / (float)best_cnt;
}

static uint32_t CalChecksum(const CalPersist_t *p){
    const uint32_t *w         = (const uint32_t *)p;
    uint32_t        size_words = (sizeof(CalPersist_t) / sizeof(uint32_t)) - 2u;
    uint32_t        csum       = 0u;
    for (uint32_t i = 1u; i <= size_words; i++) csum ^= w[i];
    return csum;
}

static void Cal_EnableBkpSRAM(void){
    HAL_PWR_EnableBkUpAccess();          /* unlock backup domain write protection */
    __HAL_RCC_BKPRAM_CLK_ENABLE();      /* enable backup SRAM clock on AHB4     */

    if (HAL_PWREx_EnableBkUpReg() != HAL_OK) {
        printf("[CAL] ERROR: backup regulator timeout to write \r\n");
    }
}


/* =========================================================================
 * LUT and polynomial builders
 * ========================================================================= */

/**
 * @brief Fill the 7-point LUT breakpoints from the sample buffer.
 *
 * Breakpoint 0 is always the mechanical anchor (open_freq, 0 steps).
 * Remaining breakpoints are distributed evenly across the measured
 * frequency range using nearest-neighbour selection.
 */
static void Cal_BuildTable(uint8_t str){
    if (s_buf_n < CAL_MIN_SAMPLES) {
        printf("[CAL] str%u WARN: only %u samples — LUT unchanged\r\n",
               (unsigned)str, (unsigned)s_buf_n);
        return;
    }

    /* Find the open-string measurement (nearest to 0 steps).
     * In the unified sweep, index 2 (0 semitones) should be near pos 0.
     * Fallback to nominal open frequency if nothing is close. */
    float open_f = s_open_freq[str];   /* nominal fallback */
    float open_s = 0.0f;
    float best_abs_steps = 9999.0f;
    for (uint16_t k = 0u; k < s_buf_n; k++) {
        float abs_s = fabsf(s_buf[k].steps);
        if (abs_s < best_abs_steps) {
            best_abs_steps = abs_s;
            open_f = s_buf[k].freq;
            open_s = s_buf[k].steps;
        }
    }

    float max_f = s_buf[0].freq;
    for (uint16_t k = 1u; k < s_buf_n; k++) {
        if (s_buf[k].freq > max_f) max_f = s_buf[k].freq;
    }

    printf("[CAL] str%u: %u samples | open=%.2f Hz | top=%.2f Hz\r\n",
           (unsigned)str, (unsigned)s_buf_n,
           (double)open_f, (double)max_f);

    for (uint8_t bp = 0u; bp < TUNING_FF_NUM_POINTS; bp++) {

        if (bp == 0u) {
            /* Use measured open-string frequency (nearest to 0 steps) */
            tuning_ff_freq_bp [str][0] = open_f;
            tuning_ff_steps_bp[str][0] = open_s;
            printf("[CAL-LUT] %u, 0, %.2f, %.0f\r\n",
                   (unsigned)str, (double)open_f, (double)open_s);
            continue;
        }

        float target_f = open_f + (max_f - open_f)
                         * ((float)bp / (float)(TUNING_FF_NUM_POINTS - 1u));

        uint16_t best      = 0u;
        float    best_diff = fabsf(s_buf[0].freq - target_f);
        for (uint16_t k = 1u; k < s_buf_n; k++) {
            float d = fabsf(s_buf[k].freq - target_f);
            if (d < best_diff) { best_diff = d; best = k; }
        }

        tuning_ff_freq_bp [str][bp] = s_buf[best].freq;
        tuning_ff_steps_bp[str][bp] = s_buf[best].steps;

        printf("[CAL-LUT] %u, %u, %.2f, %.0f\r\n",
               (unsigned)str, (unsigned)bp,
               (double)s_buf[best].freq, (double)s_buf[best].steps);
    }
}

/**
 * @brief Fit quadratic least-squares polynomial: steps = a + b·f + c·f²
 *
 * Solves the 3×3 normal equations via partial-pivoting Gaussian elimination
 * in double precision.  Falls back to a linear estimate if fewer than 3
 * valid samples are present or the system is singular.
 */
static void Cal_FitPolynomial(uint8_t str){
    if (s_buf_n < 3u) {
        cal_poly_coeff[str][0] = 0.0f;
        cal_poly_coeff[str][1] = 10.0f;
        cal_poly_coeff[str][2] = 0.0f;
        printf("[CAL-POLY] str%u: too few samples, linear fallback\r\n",
               (unsigned)str);
        return;
    }

    double S0=0, S1=0, S2=0, S3=0, S4=0, T0=0, T1=0, T2=0;

    for (uint16_t i = 0u; i < s_buf_n; i++) {
        double x  = (double)s_buf[i].freq;
        double y  = (double)s_buf[i].steps;
        double x2 = x * x;
        S0 += 1.0;   S1 += x;    S2 += x2;
        S3 += x2*x;  S4 += x2*x2;
        T0 += y;     T1 += x*y;  T2 += x2*y;
    }

    double A[3][4] = {
        { S0, S1, S2, T0 },
        { S1, S2, S3, T1 },
        { S2, S3, S4, T2 }
    };

    for (int piv = 0; piv < 3; piv++) {
        int maxRow = piv;
        for (int r = piv + 1; r < 3; r++) {
            if (fabs(A[r][piv]) > fabs(A[maxRow][piv])) maxRow = r;
        }
        for (int c = 0; c < 4; c++) {
            double tmp = A[piv][c]; A[piv][c] = A[maxRow][c]; A[maxRow][c] = tmp;
        }
        if (fabs(A[piv][piv]) < 1e-12) {
            cal_poly_coeff[str][0] = 0.0f;
            cal_poly_coeff[str][1] = 10.0f;
            cal_poly_coeff[str][2] = 0.0f;
            printf("[CAL-POLY] str%u: singular matrix, linear fallback\r\n",
                   (unsigned)str);
            return;
        }
        for (int r = piv + 1; r < 3; r++) {
            double factor = A[r][piv] / A[piv][piv];
            for (int c = piv; c < 4; c++) A[r][c] -= factor * A[piv][c];
        }
    }

    double coeff[3];
    for (int i = 2; i >= 0; i--) {
        coeff[i] = A[i][3];
        for (int j = i + 1; j < 3; j++) coeff[i] -= A[i][j] * coeff[j];
        coeff[i] /= A[i][i];
    }

    cal_poly_coeff[str][0] = (float)coeff[0];
    cal_poly_coeff[str][1] = (float)coeff[1];
    cal_poly_coeff[str][2] = (float)coeff[2];

    printf("[CAL-POLY] str%u: a=%.3f b=%.4f c=%.6f\r\n",
           (unsigned)str,
           (double)coeff[0], (double)coeff[1], (double)coeff[2]);
}

/**
 * @brief Fit semitone-domain polynomials from s_buf[] data.
 *
 * Converts (steps, freq) → (steps, semi) using freq_to_semi, then fits:
 *   Feedforward: steps = a·semi² + b·semi + c   (tuning_ff_semi_coeff)
 *   Plant model: semi  = a·steps² + b·steps + c (tuning_plant_coeff)
 *
 * Uses the same Gaussian elimination approach as Cal_FitPolynomial.
 */
static void Cal_FitSemiPolynomial(uint8_t str)
{
    if (s_buf_n < 3u) {
        printf("[CAL-SEMI] str%u: too few samples for semitone fit\r\n",
               (unsigned)str);
        return;
    }

    float open_f = s_open_freq[str];

    /* --- Feedforward fit: steps = a·semi² + b·semi + c --- */
    {
        double S0=0, S1=0, S2=0, S3=0, S4=0, T0=0, T1=0, T2=0;

        for (uint16_t i = 0u; i < s_buf_n; i++) {
            double semi = 12.0 * log2((double)s_buf[i].freq / (double)open_f);
            double y    = (double)s_buf[i].steps;
            double x2   = semi * semi;
            S0 += 1.0;   S1 += semi;     S2 += x2;
            S3 += x2*semi; S4 += x2*x2;
            T0 += y;     T1 += semi*y;   T2 += x2*y;
        }

        double A[3][4] = {
            { S0, S1, S2, T0 },
            { S1, S2, S3, T1 },
            { S2, S3, S4, T2 }
        };

        uint8_t ok = 1u;
        for (int piv = 0; piv < 3; piv++) {
            int maxRow = piv;
            for (int r = piv + 1; r < 3; r++) {
                if (fabs(A[r][piv]) > fabs(A[maxRow][piv])) maxRow = r;
            }
            for (int c = 0; c < 4; c++) {
                double tmp = A[piv][c]; A[piv][c] = A[maxRow][c]; A[maxRow][c] = tmp;
            }
            if (fabs(A[piv][piv]) < 1e-12) { ok = 0u; break; }
            for (int r = piv + 1; r < 3; r++) {
                double factor = A[r][piv] / A[piv][piv];
                for (int c = piv; c < 4; c++) A[r][c] -= factor * A[piv][c];
            }
        }

        if (ok) {
            double coeff[3];
            for (int i = 2; i >= 0; i--) {
                coeff[i] = A[i][3];
                for (int j = i + 1; j < 3; j++) coeff[i] -= A[i][j] * coeff[j];
                coeff[i] /= A[i][i];
            }
            tuning_ff_semi_coeff[str][0] = (float)coeff[0];  /* a (semi²) */
            tuning_ff_semi_coeff[str][1] = (float)coeff[1];  /* b (semi)  */
            tuning_ff_semi_coeff[str][2] = (float)coeff[2];  /* c (const) */

            printf("[CAL-SEMI-FF] str%u: steps = %.4f·s² + %.3f·s + %.3f\r\n",
                   (unsigned)str, coeff[0], coeff[1], coeff[2]);
        } else {
            printf("[CAL-SEMI-FF] str%u: singular — keeping existing\r\n",
                   (unsigned)str);
        }
    }

    /* --- Plant model fit: semi = a·steps² + b·steps + c --- */
    {
        double S0=0, S1=0, S2=0, S3=0, S4=0, T0=0, T1=0, T2=0;

        for (uint16_t i = 0u; i < s_buf_n; i++) {
            double x    = (double)s_buf[i].steps;
            double semi = 12.0 * log2((double)s_buf[i].freq / (double)open_f);
            double x2   = x * x;
            S0 += 1.0;   S1 += x;      S2 += x2;
            S3 += x2*x;  S4 += x2*x2;
            T0 += semi;  T1 += x*semi;  T2 += x2*semi;
        }

        double A[3][4] = {
            { S0, S1, S2, T0 },
            { S1, S2, S3, T1 },
            { S2, S3, S4, T2 }
        };

        uint8_t ok = 1u;
        for (int piv = 0; piv < 3; piv++) {
            int maxRow = piv;
            for (int r = piv + 1; r < 3; r++) {
                if (fabs(A[r][piv]) > fabs(A[maxRow][piv])) maxRow = r;
            }
            for (int c = 0; c < 4; c++) {
                double tmp = A[piv][c]; A[piv][c] = A[maxRow][c]; A[maxRow][c] = tmp;
            }
            if (fabs(A[piv][piv]) < 1e-12) { ok = 0u; break; }
            for (int r = piv + 1; r < 3; r++) {
                double factor = A[r][piv] / A[piv][piv];
                for (int c = piv; c < 4; c++) A[r][c] -= factor * A[piv][c];
            }
        }

        if (ok) {
            double coeff[3];
            for (int i = 2; i >= 0; i--) {
                coeff[i] = A[i][3];
                for (int j = i + 1; j < 3; j++) coeff[i] -= A[i][j] * coeff[j];
                coeff[i] /= A[i][i];
            }
            tuning_plant_coeff[str][0] = (float)coeff[0];  /* a (steps²) */
            tuning_plant_coeff[str][1] = (float)coeff[1];  /* b (steps)  */
            tuning_plant_coeff[str][2] = (float)coeff[2];  /* c (const)  */

            printf("[CAL-SEMI-PLANT] str%u: semi = %.4e·s² + %.4e·s + %.4f\r\n",
                   (unsigned)str, coeff[0], coeff[1], coeff[2]);
        } else {
            printf("[CAL-SEMI-PLANT] str%u: singular — keeping existing\r\n",
                   (unsigned)str);
        }
    }
}

/* =========================================================================
 * Persistent storage
 * ========================================================================= */



static void Cal_SaveToFlash(void)
{

    if (sizeof(CalPersist_t) > 128 * 1024) {
        printf("[CAL] ERROR: struct too large for sector\n");
        return;
    }


    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase;
    uint32_t error;

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Sector       = CAL_FLASH_SECTOR;
    erase.NbSectors    = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASHEx_Erase(&erase, &error) != HAL_OK) {
        printf("[CAL] Flash erase failed\n");
        HAL_FLASH_Lock();
        return;
    }

    CalPersist_t temp __attribute__((aligned(32)));

    temp.magic = CAL_PERSIST_MAGIC;

    memcpy(temp.ff_freq,  tuning_ff_freq_bp,  sizeof(temp.ff_freq));
    memcpy(temp.ff_steps, tuning_ff_steps_bp, sizeof(temp.ff_steps));

    for (uint8_t i = 0; i < NUM_STRINGS; i++) {
        temp.poly_a[i] = cal_poly_coeff[i][0];
        temp.poly_b[i] = cal_poly_coeff[i][1];
        temp.poly_c[i] = cal_poly_coeff[i][2];
    }

    memcpy(temp.semi_ff,    tuning_ff_semi_coeff, sizeof(temp.semi_ff));
    memcpy(temp.semi_plant, tuning_plant_coeff,   sizeof(temp.semi_plant));

    temp.checksum = CalChecksum(&temp);


uint8_t *src = (uint8_t*)&temp;
uint32_t addr = CAL_FLASH_ADDR;

for (uint32_t i = 0; i < sizeof(CalPersist_t); i += 32) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                          addr,
                          (uint32_t)(src + i)) != HAL_OK) {
        printf("[CAL] Flash write failed at block %lu\n", i/32);
        break;
    }
    addr += 32;
}

SCB_CleanDCache();
SCB_InvalidateDCache();

    HAL_FLASH_Lock();

    printf("[CAL] Saved to FLASH @ 0x%08lX\n", (unsigned long)CAL_FLASH_ADDR);
}




uint8_t Cal_LoadFromFlash(void){
    

   const CalPersist_t *p = (const CalPersist_t *)CAL_FLASH_ADDR;

    if (p->magic != CAL_PERSIST_MAGIC) {
        printf("[CAL] No stored calibration (magic 0x%08lX)\r\n",
               (unsigned long)p->magic);
        return 0u;
    }

    uint32_t expected = CalChecksum(p);
    if (p->checksum != expected) {
        printf("[CAL] Checksum mismatch (got 0x%08lX expected 0x%08lX) — ignored\r\n",
               (unsigned long)p->checksum, (unsigned long)expected);

        /* Dump raw words to find which one is corrupted */
        const uint32_t *w = (const uint32_t *)p;
        uint32_t nwords = sizeof(CalPersist_t) / sizeof(uint32_t);
        for (uint32_t i = 0u; i < nwords; i++) {
            printf("  w[%lu]=0x%08lX\r\n", (unsigned long)i, (unsigned long)w[i]);
        }

        return 0u;
    }

    memcpy(tuning_ff_freq_bp,  p->ff_freq,  sizeof(tuning_ff_freq_bp));
    memcpy(tuning_ff_steps_bp, p->ff_steps, sizeof(tuning_ff_steps_bp));

    for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
        cal_poly_coeff[i][0] = p->poly_a[i];
        cal_poly_coeff[i][1] = p->poly_b[i];
        cal_poly_coeff[i][2] = p->poly_c[i];
    }

    /* Semitone-domain coefficients — check for non-zero to detect old SRAM
     * layouts that don't have these fields (they'd read as all-zero). */
    {
        uint8_t has_semi = 0u;
        for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
            if (p->semi_ff[i][0] != 0.0f || p->semi_ff[i][1] != 0.0f ||
                p->semi_ff[i][2] != 0.0f) {
                has_semi = 1u;
                break;
            }
        }
        if (has_semi) {
            memcpy(tuning_ff_semi_coeff, p->semi_ff,    sizeof(tuning_ff_semi_coeff));
            memcpy(tuning_plant_coeff,   p->semi_plant, sizeof(tuning_plant_coeff));
            printf("[CAL] Semitone-domain coefficients restored.\r\n");
        } else {
            printf("[CAL] No semitone coefficients in FLASH — using defaults.\r\n");
        }
    }

    printf("[CAL] Calibration restored from FLASH.\r\n");
    return 1u;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief Seed cal_poly_coeff from the hardcoded base LUT data.
 *
 * Call when Cal_LoadFromSRAM() returns 0 (no valid stored calibration).
 * Fits the quadratic polynomial steps = a + b·f + c·f² for every string
 * using the existing tuning_ff_freq_bp / tuning_ff_steps_bp breakpoints so
 * the tuning controller has a reasonable feedforward curve from first boot.
 */
void Cal_FitFromBaseLUT(void){
    for (uint8_t str = 0u; str < NUM_STRINGS; str++) {
        s_buf_n = TUNING_FF_NUM_POINTS;
        for (uint8_t i = 0u; i < TUNING_FF_NUM_POINTS; i++) {
            s_buf[i].freq  = tuning_ff_freq_bp[str][i];
            s_buf[i].steps = tuning_ff_steps_bp[str][i];
        }
        Cal_FitPolynomial(str);

        /* Seed semitone-domain coefficients from the base LUT too,
         * but skip string 0 (E2) which has MATLAB-derived values. */
        if (str != 0u) {
            Cal_FitSemiPolynomial(str);
        }
    }
    s_buf_n = 0u;
}

void Cal_Start(uint8_t string_mask){
    if ((string_mask & 0x3Fu) == 0u) {
        printf("[CAL] ERROR: empty string mask.\r\n");
        s_status = CAL_STATUS_ERROR;
        return;
    }

    s_string_mask = string_mask & 0x3Fu;
    s_str = 0u;
    while (s_str < NUM_STRINGS && !((s_string_mask >> s_str) & 1u)) s_str++;

    /* A fresh calibration changes the FF/LUT basis for these strings, so any
     * previously stored open-string tune offsets for the same strings are stale. */
    if (Tune_ClearOffsetsMask(s_string_mask)) {
        printf("[CAL] Cleared stale tune offsets for mask 0x%02X\r\n",
               (unsigned)s_string_mask);
    }

    s_buf_n         = 0u;
    s_fret          = 0u;
    s_retry_cnt     = 0u;
    s_is_reexciting = 0u;
    s_phase         = CAL_PHASE_STR_INIT;
    s_status        = CAL_STATUS_RUNNING;

    printf("[CAL] ================================================\r\n");
    printf("[CAL] Fret calibration started — mask 0x%02X\r\n",
           (unsigned)s_string_mask);
    printf("[CAL] Burst: %u ms primary / %u ms re-excite\r\n",
           (unsigned)CAL_SOL_DURATION_MS, (unsigned)CAL_REEXCITE_DURATION_MS);
    printf("[CAL] str, step, freq_hz\r\n");
    printf("[CAL] ================================================\r\n");
}

void Cal_StartStr(uint8_t str_id){
    if (str_id >= NUM_STRINGS) {
        printf("[CAL] ERROR: invalid string ID %u\r\n", (unsigned)str_id);
        return;
    }
    Cal_Start((uint8_t)(1u << str_id));
}

void Cal_StartAll(void)   { Cal_Start(0x3Fu); }

CalStatus_t Cal_GetStatus(void) { return s_status; }

/* =========================================================================
 * Cal_Hop — state machine (called once per FFT hop ≈ every 170 ms)
 * ========================================================================= */

void Cal_Hop(void){
    switch (s_phase) {

    /* ------------------------------------------------------------------
     * STR_INIT: enable motor and prompt the user to pluck immediately.
     * The motor has NOT been homed yet — homing happens AFTER the pluck
     * is detected so the user doesn't have to wait.
     * ------------------------------------------------------------------ */
    case CAL_PHASE_STR_INIT:
    {
        InterCore_SendMotorEnableMask(1u << s_str);
        InterCore_SendMotorMaxSpeed(s_str, CAL_MOTOR_SPEED_FAST);

        s_fret               = 0u;
        s_buf_n              = 0u;
        s_retry_cnt          = 0u;
        s_is_reexciting      = 0u;
        s_pluck_timeout_cnt  = 0u;
        s_pluck_detected     = 0u;
        s_pluck_collecting   = 0u;
        s_pluck_collect_cnt  = 0u;
        s_pluck_buf_n        = 0u;
        s_pluck_dead_hops    = 0u;
        s_home_timeout_cnt   = 0u;
        s_open_detected_freq = 0.0f;
        s_open_homing_target  = 0;
        s_last_valid_freq     = 0.0f;
        s_pi_last_abs_error   = 9999.0f;
        s_pi_stall_cnt        = 0u;
        s_pi_integral        = 0.0f;
        s_pi_cmd_steps       = 0;
        s_pi_hop_cnt         = 0u;
        s_pi_lock_consec     = 0u;
        s_pi_no_sig_cnt      = 0u;
        s_open_position_offset = 0;

        printf("[CAL] -- String %u (%.2f Hz) | -1 to +7 semitones, %u pts\r\n",
               (unsigned)s_str,
               (double)s_open_freq[s_str],
               (unsigned)CAL_NUM_TARGETS);
        printf("[CAL] >>> Pluck string %u now (any fret is fine)!\r\n",
               (unsigned)s_str);

        /* Go straight to pluck detection — motor will home after pluck */
        s_phase = CAL_PHASE_WAIT_PLUCK;
        break;
    }

    /* ------------------------------------------------------------------
     * WAIT: generic hop-count decrement
     * ------------------------------------------------------------------ */
    case CAL_PHASE_WAIT:
    {
        if (s_wait_cnt > 0u) {
            s_wait_cnt--;
        } else {
            s_phase = s_after_wait;
        }
        break;
    }

    /* ------------------------------------------------------------------
     * WAIT_PLUCK: confirm the string is live at ANY fret position.
     *
     * The user plucks the string wherever it is.  A wide frequency window
     * (70%–180% of nominal open) covers the full physical fret range plus
     * the 2nd harmonic, so the string can be at any fret and still trigger.
     * After detection (or timeout), the motor is commanded home and we
     * enter WAIT_HOME to let the motor settle before calibration begins.
     * ------------------------------------------------------------------ */
    case CAL_PHASE_WAIT_PLUCK:
    {
        /* Wide window: covers full fret range (open → fret 7) + 2nd harmonic.
         * For A2 (110 Hz): 77–198 Hz — accepts any fret the string could be at.
         * Magnitude threshold rejects broadband noise.
         * active_frequencies[] is magnitude-sorted descending, so the first
         * qualifying entry is always the strongest peak in the window.        */
        float lo = s_open_freq[s_str] * 0.70f;
        float hi = s_open_freq[s_str] * 1.80f;

        float this_freq = 0.0f;
        for (uint8_t j = 0u; j < active_string_count; j++) {
            if (active_frequencies[j] >= lo && active_frequencies[j] <= hi
                    && active_magnitudes[j] > 50.0f) {
                this_freq = active_frequencies[j];
                break;
            }
        }

        /* ---- Phase 1: waiting for first signal ---- */
        if (!s_pluck_collecting) {
            if (this_freq > 0.0f) {
                s_pluck_collecting  = 1u;
                s_pluck_collect_cnt = 0u;
                s_pluck_buf_n       = 0u;
                s_pluck_dead_hops   = 0u;
                printf("[CAL] str%u: pluck detected %.2f Hz — collecting (%u hops)...\r\n",
                       (unsigned)s_str, (double)this_freq,
                       (unsigned)CAL_PLUCK_CONFIRM_HOPS);
            }
        }

        /* ---- Phase 2: collecting readings to build confidence ---- */
        if (s_pluck_collecting) {
            if (this_freq > 0.0f) {
                if (s_pluck_buf_n < CAL_PLUCK_CONFIRM_HOPS) {
                    s_pluck_buf[s_pluck_buf_n++] = this_freq;
                }
                s_pluck_dead_hops = 0u;
            } else {
                s_pluck_dead_hops++;
            }

            s_pluck_collect_cnt++;

            /* Exit collection when: full window elapsed, buffer full, OR
             * string decayed with enough readings to form a consensus.    */
            uint8_t done =
                (s_pluck_collect_cnt >= CAL_PLUCK_CONFIRM_HOPS) ||
                (s_pluck_buf_n       >= CAL_PLUCK_CONFIRM_HOPS) ||
                (s_pluck_dead_hops   >= CAL_PLUCK_DEAD_HOPS &&
                 s_pluck_buf_n       >= CAL_PLUCK_CONFIRM_MIN);

            if (done) {
                s_open_detected_freq = Cal_PluckConsensus(s_pluck_buf, s_pluck_buf_n,
                                                           s_open_freq[s_str]);
                s_pluck_detected = 1u;
                printf("[CAL] str%u: consensus %.2f Hz (%u/%u readings)\r\n",
                       (unsigned)s_str, (double)s_open_detected_freq,
                       (unsigned)s_pluck_buf_n,
                       (unsigned)CAL_PLUCK_CONFIRM_HOPS);
            }
        }

        /* ---- Commit: compute FF homing target and advance ---- */
        if (s_pluck_detected) {
            /* Compute polynomial feedforward estimate for where the motor
             * needs to be to produce the nominal open-string frequency.
             * delta = poly(open_freq) - poly(detected_freq)
             * Since poly(open_freq) ≈ 0, delta ≈ -poly(detected_freq).
             * This allows tuning DOWN when the string is sharp at pos 0. */
            {
                float _a = cal_poly_coeff[s_str][0];
                float _b = cal_poly_coeff[s_str][1];
                float _c = cal_poly_coeff[s_str][2];
                float _detected_steps = _a + _b * s_open_detected_freq
                                        + _c * s_open_detected_freq * s_open_detected_freq;
                float _open_steps     = _a + _b * s_open_freq[s_str]
                                        + _c * s_open_freq[s_str]   * s_open_freq[s_str];
                float _delta = _open_steps - _detected_steps;
                int32_t cur_pos = g_shared_memory.current_positions[s_str];
                s_open_homing_target = (int32_t)CalClampf(
                    (float)cur_pos + _delta,
                    -((float)s_cal_steps_max[s_str]),
                     (float)s_cal_steps_max[s_str]);
            }
            printf("[CAL] str%u: pluck=%.2f Hz → FF homing target=%ld steps\r\n",
                   (unsigned)s_str, (double)s_open_detected_freq,
                   (long)s_open_homing_target);
            InterCore_SendMotorGoto(s_str, s_open_homing_target);
            s_home_timeout_cnt = 0u;
            s_after_home       = CAL_PHASE_OPEN_SOL_BURST;
            s_phase            = CAL_PHASE_WAIT_HOME;
            break;
        }

        /* ---- Timeout: use partial data if any, else proceed blind ---- */
        s_pluck_timeout_cnt++;
        if (s_pluck_timeout_cnt >= CAL_PLUCK_TIMEOUT) {
            if (s_pluck_buf_n > 0u) {
                s_open_detected_freq = Cal_PluckConsensus(s_pluck_buf, s_pluck_buf_n,
                                                           s_open_freq[s_str]);
                printf("[CAL] str%u: timeout — partial consensus %.2f Hz (%u readings)\r\n",
                       (unsigned)s_str, (double)s_open_detected_freq,
                       (unsigned)s_pluck_buf_n);
                float _a = cal_poly_coeff[s_str][0];
                float _b = cal_poly_coeff[s_str][1];
                float _c = cal_poly_coeff[s_str][2];
                float _detected_steps = _a + _b * s_open_detected_freq
                                        + _c * s_open_detected_freq * s_open_detected_freq;
                float _open_steps     = _a + _b * s_open_freq[s_str]
                                        + _c * s_open_freq[s_str]   * s_open_freq[s_str];
                float _delta = _open_steps - _detected_steps;
                int32_t cur_pos = g_shared_memory.current_positions[s_str];
                s_open_homing_target = (int32_t)CalClampf(
                    (float)cur_pos + _delta,
                    -((float)s_cal_steps_max[s_str]),
                     (float)s_cal_steps_max[s_str]);
            } else {
                printf("[CAL] str%u: no pluck — homing to 0 and proceeding anyway\r\n",
                       (unsigned)s_str);
                s_open_homing_target = 0;
            }
            InterCore_SendMotorGoto(s_str, s_open_homing_target);
            s_home_timeout_cnt = 0u;
            s_after_home       = CAL_PHASE_OPEN_SOL_BURST;
            s_phase            = CAL_PHASE_WAIT_HOME;
        }
        break;
    }

    /* ------------------------------------------------------------------
     * WAIT_HOME: poll motor position until it reaches position 0 (home).
     *
     * At CAL_MOTOR_SPEED_FAST = 3000 steps/sec the worst-case travel
     * (D3 at 9000 steps) takes ~3 s = ~18 hops.  CAL_HOME_TIMEOUT_HOPS
     * adds margin for acceleration/braking.  If the motor never reports
     * home (e.g. stall), calibration proceeds from wherever it stopped.
     * ------------------------------------------------------------------ */
    case CAL_PHASE_WAIT_HOME:
    {
        int32_t pos      = g_shared_memory.current_positions[s_str];
        int32_t home_err = pos - s_open_homing_target;

        if ((home_err >= -CAL_HOME_POS_THRESHOLD && home_err <= CAL_HOME_POS_THRESHOLD)
            || s_home_timeout_cnt >= CAL_HOME_TIMEOUT_HOPS) {

            if (s_home_timeout_cnt >= CAL_HOME_TIMEOUT_HOPS) {
                printf("[CAL] str%u: home timeout (pos=%ld, target=%ld) — proceeding\r\n",
                       (unsigned)s_str, (long)pos, (long)s_open_homing_target);
            } else {
                printf("[CAL] str%u: at homing target (pos=%ld, target=%ld)\r\n",
                       (unsigned)s_str, (long)pos, (long)s_open_homing_target);
            }
            s_phase = s_after_home;
        } else {
            s_home_timeout_cnt++;
        }
        break;
    }

    /* ------------------------------------------------------------------
     * OPEN_SOL_BURST: fire solenoid at nominal open frequency to excite
     * the string for the open-frequency PI lock.  Motor stays at current
     * position (~0 from homing).
     * ------------------------------------------------------------------ */
    case CAL_PHASE_OPEN_SOL_BURST:
    {
        s_burst_freq = s_open_freq[s_str];

        printf("[CAL] str%u: open-freq burst %.2f Hz — finding true open position\r\n",
               (unsigned)s_str, (double)s_burst_freq);

        StopSolenoid(s_str);

        /* Gate FFT before the solenoid fires */
        fft_sensing_enabled = 0u;

        if (s_burst_freq >= 40.0f && s_burst_freq <= 800.0f) {
            InterCore_TriggerSolenoidHz(s_str,
                                        s_burst_freq,
                                        CAL_SOLENOID_DUTY,
                                        SOLENOID_MODE_TIMED,
                                        (uint16_t)CAL_SOL_DURATION_MS,
                                        0u);
        }

        /* Reset PI for the upcoming OPEN_PI_LOCK phase.
         * Seed cmd_steps from the FF homing target (where the motor
         * should already be) so the PI starts correcting from there. */
        s_pi_integral    = 0.0f;
        s_pi_cmd_steps   = s_open_homing_target;
        s_pi_hop_cnt     = 0u;
        s_pi_lock_consec = 0u;
        s_pi_no_sig_cnt      = 0u;
        s_last_valid_freq    = 0.0f;
        s_retry_cnt          = 0u;
        s_is_reexciting      = 0u;
        s_pi_last_abs_error  = 9999.0f;
        s_pi_stall_cnt       = 0u;

        WaitThen(CAL_BURST_WAIT_HOPS, CAL_PHASE_OPEN_BURST_REENABLE);
        break;
    }

    /* ------------------------------------------------------------------
     * OPEN_BURST_REENABLE: restore FFT sensing, transition to PI lock
     * ------------------------------------------------------------------ */
    case CAL_PHASE_OPEN_BURST_REENABLE:
    {
        fft_sensing_enabled = 1u;
        s_phase = CAL_PHASE_OPEN_PI_LOCK;
        break;
    }

    /* ------------------------------------------------------------------
     * OPEN_PI_LOCK: PI targets the nominal open-string frequency.
     * Adjusts motor position to find exactly where the string produces
     * its nominal open frequency.  The discovered position becomes the
     * new software position 0 (re-zeroed in OPEN_DONE).
     *
     * Same PI logic as fret PI_LOCK / HOME_PI_LOCK but:
     *   - Target: s_open_freq[s_str]
     *   - PI center: s_open_homing_target (motor is already there from FF move)
     *   - Motor clamp: ±300 steps around s_open_homing_target
     *   - On lock/timeout: transitions to OPEN_DONE
     * ------------------------------------------------------------------ */
    case CAL_PHASE_OPEN_PI_LOCK:
    {
        float target_freq = s_open_freq[s_str];
        float search_band = target_freq * 0.15f;

        float meas_f = 0.0f;
        if (s_last_valid_freq > 0.0f) {
            CalGetFreq(s_last_valid_freq, CAL_TRACK_BAND_HZ, &meas_f);
            if (meas_f == 0.0f) {
                CalGetFreq(target_freq, search_band, &meas_f);
            }
        } else {
            CalGetFreq(target_freq, search_band, &meas_f);
        }

        /* ----- Valid FFT reading ----- */
        if (meas_f > 0.0f) {
            s_last_valid_freq = meas_f;
            s_pi_no_sig_cnt   = 0u;

            float error      = target_freq - meas_f;
            float abs_error  = fabsf(error);
            float error_cents = CalErrorCents(meas_f, target_freq);
            float abs_cents   = fabsf(error_cents);

            /* Dynamic trim cap:
             * If the frequency error is shrinking → PI is working, let it
             * drive as far as needed (bounded by full mechanical range).
             * If error is NOT shrinking for CAL_OPEN_STALL_HOPS consecutive
             * hops → stall / dead string / false reading → re-engage the
             * hard CAL_PI_MAX_TRIM cap to stop runaway accumulation.       */
            if (abs_error < s_pi_last_abs_error) {
                s_pi_stall_cnt = 0u;
            } else {
                if (s_pi_stall_cnt < CAL_OPEN_STALL_HOPS) s_pi_stall_cnt++;
            }
            s_pi_last_abs_error = abs_error;

            float trim_cap = (s_pi_stall_cnt < CAL_OPEN_STALL_HOPS)
                             ? (float)s_cal_steps_max[s_str]
                             : CAL_PI_MAX_TRIM;

            if (abs_cents > CAL_LOCK_CENTS) {
                s_pi_integral = CalClampf(
                    s_pi_integral + CAL_PI_KI * error * TUNING_SAMPLE_TIME_S,
                    -trim_cap, trim_cap);

                float trim = CalClampf(CAL_PI_KP * error + s_pi_integral,
                                       -trim_cap, trim_cap);
                int32_t cmd = (int32_t)CalClampf(
                    (float)s_open_homing_target + trim,
                    -(float)s_cal_steps_max[s_str],
                     (float)s_cal_steps_max[s_str]);

                int32_t delta = cmd - s_pi_cmd_steps;
                if ((delta < 0 ? -delta : delta) >= (int32_t)TUNING_MIN_MOVE_STEPS) {
                    s_pi_cmd_steps = cmd;
                    InterCore_SendMotorGoto(s_str, s_pi_cmd_steps);
                }
                s_pi_lock_consec = 0u;
            } else {
                s_pi_lock_consec++;
            }

            if (s_pi_lock_consec >= CAL_LOCK_CONSEC_HOPS) {
                printf("[CAL] str%u: OPEN LOCKED at %.2f Hz (%.1f cents, pos=%ld)\r\n",
                       (unsigned)s_str, (double)meas_f,
                       (double)error_cents,
                       (long)g_shared_memory.current_positions[s_str]);
                s_phase = CAL_PHASE_OPEN_DONE;
                break;
            }
        } else {
            /* ----- No FFT signal ----- */
            s_pi_no_sig_cnt++;

            if (s_pi_no_sig_cnt >= CAL_DEAD_HOPS) {
                s_retry_cnt++;

                if (s_retry_cnt < CAL_MAX_RETRIES) {
                    s_burst_freq = (s_last_valid_freq > 0.0f)
                                   ? s_last_valid_freq
                                   : target_freq;

                    printf("[CAL] str%u: open signal lost — re-excite %.2f Hz "
                           "(retry %u/%u)\r\n",
                           (unsigned)s_str, (double)s_burst_freq,
                           (unsigned)s_retry_cnt, (unsigned)CAL_MAX_RETRIES);

                    fft_sensing_enabled = 0u;
                    if (s_burst_freq >= 40.0f && s_burst_freq <= 800.0f) {
                        InterCore_TriggerSolenoidHz(s_str,
                                                    s_burst_freq,
                                                    CAL_SOLENOID_DUTY,
                                                    SOLENOID_MODE_TIMED,
                                                    (uint16_t)CAL_REEXCITE_DURATION_MS,
                                                    0u);
                    }
                    InterCore_SendMotorGoto(s_str, s_pi_cmd_steps);
                    s_pi_integral    = 0.0f;
                    s_pi_hop_cnt     = 0u;
                    s_pi_lock_consec = 0u;
                    s_pi_no_sig_cnt  = 0u;
                    WaitThen(CAL_REEXCITE_WAIT_HOPS, CAL_PHASE_OPEN_BURST_REENABLE);
                } else {
                    printf("[CAL] str%u: open max retries — accepting position %ld\r\n",
                           (unsigned)s_str,
                           (long)g_shared_memory.current_positions[s_str]);
                    s_phase = CAL_PHASE_OPEN_DONE;
                }
                break;
            }
        }

        /* ----- Timeout check ----- */
        s_pi_hop_cnt++;
        if (s_pi_hop_cnt >= CAL_PI_LOCK_TIMEOUT &&
            s_pi_lock_consec < CAL_LOCK_CONSEC_HOPS) {
            s_retry_cnt++;

            if (s_retry_cnt < CAL_MAX_RETRIES) {
                s_burst_freq = (s_last_valid_freq > 0.0f)
                               ? s_last_valid_freq
                               : target_freq;

                printf("[CAL] str%u: open lock timeout — re-excite %.2f Hz "
                       "(retry %u/%u)\r\n",
                       (unsigned)s_str, (double)s_burst_freq,
                       (unsigned)s_retry_cnt, (unsigned)CAL_MAX_RETRIES);

                fft_sensing_enabled = 0u;
                if (s_burst_freq >= 40.0f && s_burst_freq <= 800.0f) {
                    InterCore_TriggerSolenoidHz(s_str,
                                                s_burst_freq,
                                                CAL_SOLENOID_DUTY,
                                                SOLENOID_MODE_TIMED,
                                                (uint16_t)CAL_REEXCITE_DURATION_MS,
                                                0u);
                }
                InterCore_SendMotorGoto(s_str, s_pi_cmd_steps);
                s_pi_integral    = 0.0f;
                s_pi_hop_cnt     = 0u;
                s_pi_lock_consec = 0u;
                s_pi_no_sig_cnt  = 0u;
                WaitThen(CAL_REEXCITE_WAIT_HOPS, CAL_PHASE_OPEN_BURST_REENABLE);
            } else {
                printf("[CAL] str%u: open timeout after %u retries — pos=%ld\r\n",
                       (unsigned)s_str, (unsigned)CAL_MAX_RETRIES,
                       (long)g_shared_memory.current_positions[s_str]);
                s_phase = CAL_PHASE_OPEN_DONE;
            }
        }
        break;
    }

    /* ------------------------------------------------------------------
     * OPEN_DONE: record the offset and proceed to fret calibration
     * ------------------------------------------------------------------ */
    case CAL_PHASE_OPEN_DONE:
    {
        StopSolenoid(s_str);

        int32_t locked_pos = g_shared_memory.current_positions[s_str];

        printf("[CAL] str%u: open locked at %.2f Hz, pos=%ld — re-zeroing motor\r\n",
               (unsigned)s_str, (double)s_last_valid_freq, (long)locked_pos);

        /* Re-zero: set the current physical position as the new software
         * position 0 on CM4.  After this, position 0 = true open string. */
        InterCore_SendMotorZero(s_str);

        /* The polynomial cal_poly_coeff was fitted in a PREVIOUS coordinate
         * system.  In that system, poly(open_freq) != 0 in general.  After
         * re-zeroing, position 0 = open, so we need:
         *   motor_target = poly(target_freq) - poly(open_freq)
         * Set the offset to -poly(open_freq) so SOL_BURST adds it. */
        {
            float _a = cal_poly_coeff[s_str][0];
            float _b = cal_poly_coeff[s_str][1];
            float _c = cal_poly_coeff[s_str][2];
            float _of = s_open_freq[s_str];
            float poly_at_open = _a + _b * _of + _c * _of * _of;
            s_open_position_offset = -(int32_t)roundf(poly_at_open);
            printf("[CAL] str%u: poly offset = %ld (poly(%.1f Hz) = %.1f)\r\n",
                   (unsigned)s_str, (long)s_open_position_offset,
                   (double)_of, (double)poly_at_open);
        }

        /* Open-string point will be measured in the unified loop at
         * index 2 (0 semitones).  No s_buf recording here. */
        s_buf_n = 0u;

        /* Reset state for measurement loop */
        s_retry_cnt     = 0u;
        s_is_reexciting = 0u;

        /* Proceed to unified measurement loop (starts at s_fret=0 = −1 semitone) */
        s_phase = CAL_PHASE_SOL_BURST;
        break;
    }

    /* ------------------------------------------------------------------
     * SOL_BURST: fire solenoid and (on primary burst) start motor transit.
     *
     * Primary burst (s_is_reexciting == 0):
     *   – Fires at the string's ACTUAL current resonance:
     *       fret 1: s_open_detected_freq (measured during pluck detection)
     *       fret N: s_buf[N-2].freq (last recorded calibration measurement)
     *     Using the measured frequency instead of theoretical maximises
     *     energy transfer to the string.
     *   – Motor target is estimated from the calibrated polynomial
     *     (cal_poly_coeff) clamped to s_cal_steps_max[str], replacing the
     *     old linear fret*1600 approximation.
     *   – Uses CAL_SOL_DURATION_MS (1 s) and CAL_BURST_WAIT_HOPS (8).
     *
     * Re-excite burst (s_is_reexciting == 1):
     *   – Fires at s_burst_freq, which was set to s_last_valid_freq in
     *     PI_LOCK's early-exit path.  Firing at the string's actual
     *     current resonance (rather than the target fret freq) maximises
     *     the energy imparted to a partially-decayed string.
     *   – Re-issues the motor goto so transit continues if it had stopped.
     *   – Uses CAL_REEXCITE_DURATION_MS (300 ms) and CAL_REEXCITE_WAIT_HOPS (4).
     * ------------------------------------------------------------------ */
    case CAL_PHASE_SOL_BURST:
    {
        if (!s_is_reexciting) {
            /* --- Primary burst --- */

            /* Fire solenoid at the TARGET frequency for this motor position.
             * Using the target (not the previous measurement) ensures the
             * solenoid excites the correct resonant mode and avoids
             * contaminating the FFT with residual energy from a different
             * frequency. */
            s_burst_freq = Cal_TargetFreq(s_str, s_fret);

            /* Estimate target motor position using the calibrated polynomial.
             * cal_poly_coeff is seeded at boot from the base LUT and updated
             * after every completed calibration run — estimates improve over
             * time.  The per-string safety cap prevents over-travel.          */
            {
                float _tf   = Cal_TargetFreq(s_str, s_fret);
                float _a    = cal_poly_coeff[s_str][0];
                float _b    = cal_poly_coeff[s_str][1];
                float _c    = cal_poly_coeff[s_str][2];
                float _poly = _a + _b * _tf + _c * _tf * _tf;
                _poly += (float)s_open_position_offset;
                s_steps_expected = (int32_t)CalClampf(_poly,
                                                      (float)TUNING_STEPS_MIN_DEFAULT,
                                                      (float)s_cal_steps_max[s_str]);
            }

            /* Warn if motor is far from where it should be for the previous fret */
            int32_t actual_pos    = g_shared_memory.current_positions[s_str];
            int32_t prev_expected = (s_buf_n > 0u)
                                    ? (int32_t)s_buf[s_buf_n - 1u].steps
                                    : 0;
            int32_t pos_err       = actual_pos - prev_expected;
            printf("[CAL] str%u t%u (%.1f st) | burst %.2f Hz, motor pos=%ld err=%ld%s\r\n",
                   (unsigned)s_str, (unsigned)s_fret,
                   (double)(((float)s_fret - 2.0f) * 0.5f),
                   (double)s_burst_freq, (long)actual_pos, (long)pos_err,
                   (pos_err > CAL_STALL_WARN_STEPS || pos_err < -CAL_STALL_WARN_STEPS)
                       ? " [STALL?]" : "");

            /* Start motor moving toward next fret target DURING the burst+wait.
             * At 3000 steps/sec the wait window of 8×170 ms = 1360 ms gives
             * the motor time to arrive before the FFT re-opens.
             * Motor's own braking-distance logic handles the final approach. */
            InterCore_SendMotorMaxSpeed(s_str, CAL_MOTOR_SPEED_FAST);
            InterCore_SendMotorGoto(s_str, s_steps_expected);

        } else {
            /* --- Re-excite burst --- */
            /* s_burst_freq was already set to s_last_valid_freq in PI_LOCK.
             * Re-issue the goto so the motor continues its remaining travel. */
            printf("[CAL] str%u fret%u | re-excite %.2f Hz, continue to pos=%ld (retry %u/%u)\r\n",
                   (unsigned)s_str, (unsigned)s_fret,
                   (double)s_burst_freq, (long)s_steps_expected,
                   (unsigned)s_retry_cnt, (unsigned)CAL_MAX_RETRIES);
            InterCore_SendMotorGoto(s_str, s_steps_expected);
        }

        /* Gate FFT before the solenoid fires */
        fft_sensing_enabled = 0u;

        uint16_t dur       = s_is_reexciting
                             ? (uint16_t)CAL_REEXCITE_DURATION_MS
                             : (uint16_t)CAL_SOL_DURATION_MS;
        uint8_t  wait_hops = s_is_reexciting
                             ? (uint8_t)CAL_REEXCITE_WAIT_HOPS
                             : (uint8_t)CAL_BURST_WAIT_HOPS;

        if (s_burst_freq >= 40.0f && s_burst_freq <= 800.0f) {
            InterCore_TriggerSolenoidHz(s_str,
                                        s_burst_freq,
                                        CAL_SOLENOID_DUTY,
                                        SOLENOID_MODE_TIMED,
                                        dur, 0u);
        }

        /* Reset tracking state for the upcoming RECORD phase.
         * Primary burst: motor moved to a new target — old frequency is stale.
         * Re-excite: motor barely moved — seed tracking with the burst freq. */
        s_last_valid_freq = s_is_reexciting ? s_burst_freq : 0.0f;

        WaitThen(wait_hops, CAL_PHASE_BURST_REENABLE);
        break;
    }

    /* ------------------------------------------------------------------
     * BURST_REENABLE: restore FFT sensing, settle, then record
     * ------------------------------------------------------------------ */
    case CAL_PHASE_BURST_REENABLE:
    {
        fft_sensing_enabled = 1u;
        s_is_reexciting     = 0u;
        WaitThen(CAL_SETTLE_HOPS, CAL_PHASE_RECORD);
        break;
    }

    /* ------------------------------------------------------------------
     * RECORD: save the (steps, freq) measurement for this target.
     *
     * No PI lock — just read whatever frequency the string resonates at
     * after the solenoid burst + settle window.  Wide ±20% search band
     * centered on burst frequency.  One re-excite attempt if no signal.
     *
     * Priority order for the recorded frequency:
     *   1. Current FFT reading (wide search).
     *   2. s_last_valid_freq (tracking fallback).
     *   3. Expected target frequency (fully estimated — logged with marker).
     * ------------------------------------------------------------------ */
    case CAL_PHASE_RECORD:
    {
        float search_center = Cal_TargetFreq(s_str, s_fret);
        float search_band   = search_center * 0.20f;   /* ±20% wide search */

        float meas_f = 0.0f;
        CalGetFreq(search_center, search_band, &meas_f);

        /* Tracking fallback if wide search missed */
        if (meas_f == 0.0f && s_last_valid_freq > 0.0f) {
            CalGetFreq(s_last_valid_freq, CAL_TRACK_BAND_HZ, &meas_f);
        }

        int32_t actual_steps = g_shared_memory.current_positions[s_str];

        /* If no signal at all and haven't retried yet, try one re-excite */
        if (meas_f == 0.0f && s_last_valid_freq == 0.0f && s_retry_cnt == 0u) {
            s_retry_cnt     = 1u;
            s_is_reexciting = 1u;
            s_burst_freq    = Cal_TargetFreq(s_str, s_fret);
            printf("[CAL] str%u t%u: no signal — re-excite %.2f Hz\r\n",
                   (unsigned)s_str, (unsigned)s_fret, (double)s_burst_freq);
            s_phase = CAL_PHASE_SOL_BURST;
            break;
        }

        /* Record whatever we have */
        if (s_buf_n < CAL_MAX_SAMPLES) {
            if (meas_f > 0.0f) {
                s_buf[s_buf_n].steps = (float)actual_steps;
                s_buf[s_buf_n].freq  = meas_f;
                s_last_valid_freq    = meas_f;
                printf("[CAL] %u, %ld, %.2f\r\n",
                       (unsigned)s_str, (long)actual_steps, (double)meas_f);
            } else if (s_last_valid_freq > 0.0f) {
                s_buf[s_buf_n].steps = (float)actual_steps;
                s_buf[s_buf_n].freq  = s_last_valid_freq;
                printf("[CAL] %u, %ld, %.2f (last-valid)\r\n",
                       (unsigned)s_str, (long)actual_steps,
                       (double)s_last_valid_freq);
            } else {
                float target_freq = Cal_TargetFreq(s_str, s_fret);
                s_buf[s_buf_n].steps = (float)s_steps_expected;
                s_buf[s_buf_n].freq  = target_freq;
                printf("[CAL] %u, %ld, %.2f (EST)\r\n",
                       (unsigned)s_str, (long)s_steps_expected,
                       (double)target_freq);
            }
            s_buf_n++;
        }

        /* Reset retry state and advance to the next target */
        s_retry_cnt     = 0u;
        s_is_reexciting = 0u;

        if (s_fret >= (CAL_NUM_TARGETS - 1u)) {
            /* All targets measured (idx 0..16) — proceed to build the LUT */
            s_phase = CAL_PHASE_BUILD;
        } else {
            /* Check if motor can reach the next target before attempting it */
            float next_f = Cal_TargetFreq(s_str, s_fret + 1u);
            float _a = cal_poly_coeff[s_str][0];
            float _b = cal_poly_coeff[s_str][1];
            float _c = cal_poly_coeff[s_str][2];
            float est = _a + _b * next_f + _c * next_f * next_f;
            if (est > (float)s_cal_steps_max[s_str]) {
                printf("[CAL] str%u: motor limit at target %u (est %.0f > %ld) — building\r\n",
                       (unsigned)s_str, (unsigned)(s_fret + 1u),
                       (double)est, (long)s_cal_steps_max[s_str]);
                s_phase = CAL_PHASE_BUILD;
            } else {
                s_fret++;
                s_phase = CAL_PHASE_SOL_BURST;
            }
        }
        break;
    }

    /* ------------------------------------------------------------------
     * BUILD: derive LUT, fit polynomial, persist to SRAM, return home
     * ------------------------------------------------------------------ */
    case CAL_PHASE_BUILD:
    {
        Cal_BuildTable(s_str);
        Cal_FitPolynomial(s_str);
        Cal_FitSemiPolynomial(s_str);
        Cal_SaveToFlash();

        /* Transition to PI-based acoustic homing — drive motor back to
         * the open-string position using FFT feedback instead of relying
         * on the motor position command alone. */
        s_phase = CAL_PHASE_HOME_SOL_BURST;
        break;
    }

    /* ------------------------------------------------------------------
     * HOME_SOL_BURST: fire solenoid at last measured frequency, command
     * motor toward position 0, inhibit FFT during burst.
     * ------------------------------------------------------------------ */
    case CAL_PHASE_HOME_SOL_BURST:
    {
        /* Burst at the OPEN-string frequency — not the last fret's freq.
         * The motor is returning to position 0 (open), so exciting near
         * 82 Hz (E2) ensures the string rings at its natural resonance. */
        s_burst_freq = s_open_freq[s_str];

        printf("[CAL] str%u: homing — burst %.2f Hz, motor → pos %ld\r\n",
               (unsigned)s_str, (double)s_burst_freq, (long)s_open_position_offset);

        StopSolenoid(s_str);

        /* Command motor toward measured open-string position */
        InterCore_SendMotorMaxSpeed(s_str, CAL_MOTOR_SPEED_FAST);
        InterCore_SendMotorGoto(s_str, s_open_position_offset);

        /* Gate FFT before the solenoid fires */
        fft_sensing_enabled = 0u;

        if (s_burst_freq >= 40.0f && s_burst_freq <= 800.0f) {
            InterCore_TriggerSolenoidHz(s_str,
                                        s_burst_freq,
                                        CAL_SOLENOID_DUTY,
                                        SOLENOID_MODE_TIMED,
                                        (uint16_t)CAL_SOL_DURATION_MS,
                                        0u);
        }

        /* Reset PI for the upcoming HOME_PI_LOCK phase */
        s_pi_integral    = 0.0f;
        s_pi_cmd_steps   = s_open_position_offset;
        s_pi_hop_cnt     = 0u;
        s_pi_lock_consec = 0u;
        s_pi_no_sig_cnt  = 0u;
        s_last_valid_freq = 0.0f;
        s_retry_cnt      = 0u;
        s_is_reexciting  = 0u;

        WaitThen(CAL_BURST_WAIT_HOPS, CAL_PHASE_HOME_BURST_REENABLE);
        break;
    }

    /* ------------------------------------------------------------------
     * HOME_BURST_REENABLE: restore FFT sensing, transition to PI lock
     * ------------------------------------------------------------------ */
    case CAL_PHASE_HOME_BURST_REENABLE:
    {
        fft_sensing_enabled = 1u;
        s_phase = CAL_PHASE_HOME_PI_LOCK;
        break;
    }

    /* ------------------------------------------------------------------
     * HOME_PI_LOCK: PI targets the open-string frequency.
     * Same logic as the fret PI_LOCK but targeting s_open_freq[s_str].
     * On lock or timeout, proceeds to HOME_DONE.
     * ------------------------------------------------------------------ */
    case CAL_PHASE_HOME_PI_LOCK:
    {
        float target_freq = s_open_freq[s_str];
        float search_band = target_freq * 0.15f;   /* ±15% */

        float meas_f = 0.0f;
        if (s_last_valid_freq > 0.0f) {
            CalGetFreq(s_last_valid_freq, CAL_TRACK_BAND_HZ, &meas_f);
            if (meas_f == 0.0f) {
                CalGetFreq(target_freq, search_band, &meas_f);
            }
        } else {
            CalGetFreq(target_freq, search_band, &meas_f);
        }

        /* ----- Valid FFT reading ----- */
        if (meas_f > 0.0f) {
            s_last_valid_freq = meas_f;
            s_pi_no_sig_cnt   = 0u;

            float error       = target_freq - meas_f;
            float error_cents = CalErrorCents(meas_f, target_freq);
            float abs_cents   = fabsf(error_cents);

            if (abs_cents > CAL_LOCK_CENTS) {
                s_pi_integral = CalClampf(
                    s_pi_integral + CAL_PI_KI * error * TUNING_SAMPLE_TIME_S,
                    -CAL_PI_MAX_TRIM, CAL_PI_MAX_TRIM);

                float trim = CalClampf(CAL_PI_KP * error + s_pi_integral,
                                       -CAL_PI_MAX_TRIM, CAL_PI_MAX_TRIM);
                int32_t cmd = (int32_t)CalClampf(
                    (float)s_open_position_offset + trim,
                    (float)TUNING_STEPS_MIN_DEFAULT,
                    (float)s_cal_steps_max[s_str]);

                int32_t delta = cmd - s_pi_cmd_steps;
                if ((delta < 0 ? -delta : delta) >= (int32_t)TUNING_MIN_MOVE_STEPS) {
                    s_pi_cmd_steps = cmd;
                    InterCore_SendMotorGoto(s_str, s_pi_cmd_steps);
                }
                s_pi_lock_consec = 0u;
            } else {
                s_pi_lock_consec++;
            }

            if (s_pi_lock_consec >= CAL_LOCK_CONSEC_HOPS) {
                printf("[CAL] str%u: HOME LOCKED at %.2f Hz (%.1f cents, pos=%ld)\r\n",
                       (unsigned)s_str, (double)meas_f,
                       (double)error_cents,
                       (long)g_shared_memory.current_positions[s_str]);
                s_phase = CAL_PHASE_HOME_DONE;
                break;
            }
        } else {
            /* ----- No FFT signal ----- */
            s_pi_no_sig_cnt++;

            if (s_pi_no_sig_cnt >= CAL_DEAD_HOPS) {
                s_retry_cnt++;

                if (s_retry_cnt < CAL_HOME_MAX_RETRIES) {
                    /* Always re-excite at the open-string frequency —
                     * s_last_valid_freq may be a stale fret reading. */
                    s_burst_freq = s_open_freq[s_str];

                    printf("[CAL] str%u: home signal lost — re-excite %.2f Hz "
                           "(retry %u/%u)\r\n",
                           (unsigned)s_str, (double)s_burst_freq,
                           (unsigned)s_retry_cnt, (unsigned)CAL_HOME_MAX_RETRIES);

                    /* Re-excite: inhibit FFT, fire longer burst, re-issue goto.
                     * CAL_HOME_REEXCITE_MS is longer than the fret re-excite to
                     * give the decayed string a strong ring for PI convergence. */
                    fft_sensing_enabled = 0u;
                    if (s_burst_freq >= 40.0f && s_burst_freq <= 800.0f) {
                        InterCore_TriggerSolenoidHz(s_str,
                                                    s_burst_freq,
                                                    CAL_SOLENOID_DUTY,
                                                    SOLENOID_MODE_TIMED,
                                                    (uint16_t)CAL_HOME_REEXCITE_MS,
                                                    0u);
                    }
                    InterCore_SendMotorGoto(s_str, s_open_position_offset);
                    s_pi_integral    = 0.0f;
                    s_pi_hop_cnt     = 0u;
                    s_pi_lock_consec = 0u;
                    s_pi_no_sig_cnt  = 0u;
                    WaitThen(CAL_BURST_WAIT_HOPS, CAL_PHASE_HOME_BURST_REENABLE);
                } else {
                    printf("[CAL] str%u: home max retries — accepting position %ld\r\n",
                           (unsigned)s_str,
                           (long)g_shared_memory.current_positions[s_str]);
                    s_phase = CAL_PHASE_HOME_DONE;
                }
                break;
            }
        }

        /* ----- Timeout check ----- */
        s_pi_hop_cnt++;
        if (s_pi_hop_cnt >= CAL_PI_LOCK_TIMEOUT &&
            s_pi_lock_consec < CAL_LOCK_CONSEC_HOPS) {
            /* Haven't achieved full lock within the timeout window.
             * This fires even if s_pi_lock_consec > 0 (partial lock). */
            s_retry_cnt++;

            if (s_retry_cnt < CAL_HOME_MAX_RETRIES) {
                /* Always re-excite at the open-string frequency */
                s_burst_freq = s_open_freq[s_str];

                printf("[CAL] str%u: home lock timeout — re-excite %.2f Hz "
                       "(retry %u/%u)\r\n",
                       (unsigned)s_str, (double)s_burst_freq,
                       (unsigned)s_retry_cnt, (unsigned)CAL_HOME_MAX_RETRIES);

                fft_sensing_enabled = 0u;
                if (s_burst_freq >= 40.0f && s_burst_freq <= 800.0f) {
                    InterCore_TriggerSolenoidHz(s_str,
                                                s_burst_freq,
                                                CAL_SOLENOID_DUTY,
                                                SOLENOID_MODE_TIMED,
                                                (uint16_t)CAL_HOME_REEXCITE_MS,
                                                0u);
                }
                InterCore_SendMotorGoto(s_str, s_open_position_offset);
                s_pi_integral    = 0.0f;
                s_pi_hop_cnt     = 0u;
                s_pi_lock_consec = 0u;
                s_pi_no_sig_cnt  = 0u;
                WaitThen(CAL_BURST_WAIT_HOPS, CAL_PHASE_HOME_BURST_REENABLE);
            } else {
                printf("[CAL] str%u: home timeout after %u retries — pos=%ld\r\n",
                       (unsigned)s_str, (unsigned)CAL_HOME_MAX_RETRIES,
                       (long)g_shared_memory.current_positions[s_str]);
                s_phase = CAL_PHASE_HOME_DONE;
            }
        }
        break;
    }

    /* ------------------------------------------------------------------
     * HOME_DONE: stop solenoid, advance to next string or finish
     * ------------------------------------------------------------------ */
    case CAL_PHASE_HOME_DONE:
    {
        StopSolenoid(s_str);

        /* Advance to next string in mask */
        do { s_str++; } while (s_str < NUM_STRINGS && !((s_string_mask >> s_str) & 1u));

        if (s_str >= NUM_STRINGS) {
            fft_sensing_enabled = 1u;   /* ensure FFT is re-enabled */
            printf("[CAL] ================================================\r\n");
            printf("[CAL] Calibration complete — LUT and polynomial saved.\r\n");
            printf("[CAL] All motors returned to open-string position.\r\n");
            printf("[CAL] ================================================\r\n");
            s_phase  = CAL_PHASE_DONE;
            s_status = CAL_STATUS_DONE;
        } else {
            WaitThen(CAL_HOME_SETTLE_HOPS, CAL_PHASE_STR_INIT);
        }
        break;
    }

    /* ------------------------------------------------------------------
     * Terminal states
     * ------------------------------------------------------------------ */
    case CAL_PHASE_IDLE:
    case CAL_PHASE_DONE:
    case CAL_PHASE_ERROR:
    default:
        break;
    }
}

/* =========================================================================
 * Tune controller — open-string pitch correction (bidirectional)
 *
 * Detects the current open-string frequency via FFT, then adjusts the
 * motor forward (if flat) or backward (if sharp) using a PI controller
 * until the string plays at the standard tuning frequency.  The resulting
 * motor offset is stored and applied by the tuning controller's
 * FF_Interpolate() so all subsequent song playback is compensated.
 *
 * State machine phases (per string):
 *   TUNE_STR_INIT → TUNE_WAIT_HOME → TUNE_WAIT_PLUCK
 *   → TUNE_MOTOR_MOVE → TUNE_WAIT_MOTOR → TUNE_SOL_BURST
 *   → TUNE_BURST_REENABLE → TUNE_PI_LOCK
 *   → TUNE_RECORD → (next string or TUNE_DONE)
 *
 * Reuses existing helpers: CalGetFreq(), CalClampf(), WaitThen(),
 *                          StopSolenoid(), InterCore_* API.
 * ========================================================================= */

/* ---- Tune parameters ---- */

/** Tune proportional gain for incremental open-string correction. */
#define TUNE_PI_KP                  16.0f

/** Tune integral gain (microsteps / Hz / s), used mainly near lock. */
#define TUNE_PI_KI                  4.0f

/** Maximum tune PI trim authority (±microsteps). */
#define TUNE_PI_MAX_TRIM            600.0f

/** Maximum commanded step change for large tune errors. */
#define TUNE_MAX_STEP_DELTA         800.0f

/** Minimum move threshold for the tune controller. */
#define TUNE_MIN_MOVE_STEPS         4.0f

/** Tune lock-band deadband (Hz). */
#define TUNE_LOCK_BAND_HZ           0.2f

/** Only integrate once the string is already close to target. */
#define TUNE_INTEGRAL_ENABLE_BAND_HZ  1.0f

/** Consecutive locked hops to confirm convergence. */
#define TUNE_LOCK_CONSEC_HOPS       2u

/** Max PI_LOCK hops before triggering re-excite or giving up. */
#define TUNE_PI_LOCK_TIMEOUT        40u

/** Consecutive hops with no FFT signal before re-excite. */
#define TUNE_DEAD_HOPS              4u

/** Max re-excite retries per string. */
#define TUNE_MAX_RETRIES            3u

/** Solenoid re-excite burst duration (ms). */
#define TUNE_REEXCITE_DURATION_MS   200u

/** Hops to wait after re-excite burst. */
#define TUNE_REEXCITE_WAIT_HOPS     4u

/** Motor speed during tune moves. */
#define TUNE_MOTOR_SPEED            4000

/** Max hops to wait for the user to pluck. */
#define TUNE_PLUCK_TIMEOUT          25u

/** Max hops to wait for home or tuning travel to settle. */
#define TUNE_MOTOR_WAIT_HOPS        20u

/** Strings with commissioned steppers for the tune workflow. */
#define TUNE_SUPPORTED_STRING_MASK  0x0Fu

/** Wide frequency search limits while grossly out of tune.
 *  Max must stay below 2.0 to exclude the 2nd harmonic — otherwise the
 *  tracker can latch onto 2×target, compute a huge negative error, and
 *  drive the motor into loosening runaway. */
#define TUNE_TRACK_MIN_RATIO        0.60f
#define TUNE_TRACK_MAX_RATIO        1.50f

/** Solenoid duty cycle for tune. */
#define TUNE_SOLENOID_DUTY          50u

/* ---- Tune state machine phases ---- */

typedef enum {
    TUNE_PHASE_IDLE = 0,
    TUNE_PHASE_STR_INIT,
    TUNE_PHASE_WAIT_HOME,
    TUNE_PHASE_WAIT_PLUCK,
    TUNE_PHASE_MOTOR_MOVE,
    TUNE_PHASE_WAIT_MOTOR,
    TUNE_PHASE_WAIT,          /**< Generic hop-count wait             */
    TUNE_PHASE_SOL_BURST,
    TUNE_PHASE_BURST_REENABLE,
    TUNE_PHASE_PI_LOCK,
    TUNE_PHASE_RECORD,
    TUNE_PHASE_DONE,
    TUNE_PHASE_ERROR
} TunePhase_t;

/* ---- Tune private state ---- */

static TunePhase_t s_tune_phase       = TUNE_PHASE_IDLE;
static CalStatus_t s_tune_status      = CAL_STATUS_IDLE;
static uint8_t     s_tune_str         = 0u;
static uint8_t     s_tune_mask        = 0x0Fu;

static int32_t     s_tune_offset[NUM_STRINGS] = {0};

/* Pluck detection */
static uint8_t     s_tune_pluck_cnt   = 0u;
static float       s_tune_detected_f  = 0.0f;

/* Motor move */
static int32_t     s_tune_target_steps = 0;
static uint8_t     s_tune_motor_cnt   = 0u;

/* PI state */
static float       s_tune_pi_integral    = 0.0f;
static int32_t     s_tune_pi_cmd_steps   = 0;
static uint8_t     s_tune_pi_hop_cnt     = 0u;
static uint8_t     s_tune_pi_lock_consec = 0u;
static float       s_tune_last_valid_f   = 0.0f;
static uint8_t     s_tune_no_sig_cnt     = 0u;
static uint8_t     s_tune_retry_cnt      = 0u;
static uint8_t     s_tune_locked         = 0u;

/* Generic wait */
static uint16_t    s_tune_wait_cnt       = 0u;
static TunePhase_t s_tune_after_wait     = TUNE_PHASE_IDLE;

/* ---- Tune helpers ---- */

static void TuneWaitThen(uint16_t hops, TunePhase_t next)
{
    s_tune_wait_cnt   = hops;
    s_tune_after_wait = next;
    s_tune_phase      = TUNE_PHASE_WAIT;
}

static void TuneGetTrackedFreq(float center_freq,
                               float min_freq,
                               float max_freq,
                               float *out_freq)
{
    float best_freq = 0.0f;
    float best_mag  = 0.0f;
    float best_err  = 1.0e9f;

    for (uint8_t i = 0u; i < active_string_count; i++) {
        float freq = active_frequencies[i];
        float mag  = active_magnitudes[i];

        if (freq < min_freq || freq > max_freq || mag <= 50.0f) {
            continue;
        }

        float err = fabsf(freq - center_freq);
        if (best_freq == 0.0f || err < best_err
            || (fabsf(err - best_err) < 0.01f && mag > best_mag)) {
            best_freq = freq;
            best_mag  = mag;
            best_err  = err;
        }
    }

    *out_freq = best_freq;
}

static float TuneStepCapForError(float abs_error_hz)
{
    if (abs_error_hz > 40.0f) return TUNE_MAX_STEP_DELTA;
    if (abs_error_hz > 15.0f) return 400.0f;
    if (abs_error_hz > 5.0f)  return 160.0f;
    return 80.0f;
}

static uint8_t Tune_ClearOffsetsMask(uint8_t string_mask)
{
    uint8_t changed = 0u;

    for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
        if (((string_mask >> i) & 1u) && s_tune_offset[i] != 0) {
            s_tune_offset[i] = 0;
            changed = 1u;
        }
    }

    return changed;
}

/* ---- Tune public API ---- */

void Cal_TuneStart(uint8_t string_mask)
{
    uint8_t requested_mask = string_mask & 0x3Fu;
    uint8_t supported_mask = requested_mask & TUNE_SUPPORTED_STRING_MASK;

    if (requested_mask == 0u) {
        printf("[TUNE] ERROR: empty string mask.\r\n");
        s_tune_status = CAL_STATUS_ERROR;
        return;
    }

    if (supported_mask == 0u) {
        printf("[TUNE] ERROR: tune currently supports only strings 0-3.\r\n");
        s_tune_status = CAL_STATUS_ERROR;
        return;
    }

    if (supported_mask != requested_mask) {
        printf("[TUNE] WARNING: strings 4-5 are not commissioned; using mask 0x%02X\r\n",
               (unsigned)supported_mask);
    }

    s_tune_mask = supported_mask;
    s_tune_str  = 0u;
    while (s_tune_str < NUM_STRINGS && !((s_tune_mask >> s_tune_str) & 1u))
        s_tune_str++;

    s_tune_phase  = TUNE_PHASE_STR_INIT;
    s_tune_status = CAL_STATUS_RUNNING;

    printf("[TUNE] ================================================\r\n");
    printf("[TUNE] Open-string tuning started — mask 0x%02X\r\n",
           (unsigned)s_tune_mask);
    printf("[TUNE] ================================================\r\n");
}

CalStatus_t Cal_TuneGetStatus(void) { return s_tune_status; }

int32_t Cal_TuneGetOffset(uint8_t str)
{
    if (str >= NUM_STRINGS) return 0;
    return s_tune_offset[str];
}

void Cal_TuneClearOffsets(void)
{
    memset(s_tune_offset, 0, sizeof(s_tune_offset));
    printf("[TUNE] Offsets cleared.\r\n");
}

uint8_t Cal_TuneLoadFromSRAM(void)
{
    memset(s_tune_offset, 0, sizeof(s_tune_offset));
    printf("[TUNE] Tune-offset persistence disabled — starting with zero offsets.\r\n");
    return 0u;
}

/* ---- Tune state machine ---- */

void Cal_TuneHop(void)
{
    switch (s_tune_phase) {

    /* ------------------------------------------------------------------
     * STR_INIT: enable motor and start from current position.
     * No homing — across reboots position 0 is meaningless.  The PI
     * tunes relative to wherever the motor currently sits.
     * ------------------------------------------------------------------ */
    case TUNE_PHASE_STR_INIT:
    {
        InterCore_SendMotorEnableMask(1u << s_tune_str);
        InterCore_SendMotorMaxSpeed(s_tune_str, TUNE_MOTOR_SPEED);

        int32_t cur = g_shared_memory.current_positions[s_tune_str];

        s_tune_pluck_cnt     = 0u;
        s_tune_detected_f    = 0.0f;
        s_tune_motor_cnt     = 0u;
        s_tune_pi_integral   = 0.0f;
        s_tune_pi_cmd_steps  = cur;
        s_tune_pi_hop_cnt    = 0u;
        s_tune_pi_lock_consec = 0u;
        s_tune_last_valid_f  = 0.0f;
        s_tune_no_sig_cnt    = 0u;
        s_tune_retry_cnt     = 0u;
        s_tune_target_steps  = cur;
        s_tune_locked        = 0u;

        printf("[TUNE] -- String %u (target %.2f Hz) starting at pos=%ld\r\n",
               (unsigned)s_tune_str,
               (double)s_open_freq[s_tune_str],
               (long)cur);
        printf("[TUNE] >>> Pluck string %u now!\r\n",
               (unsigned)s_tune_str);

        s_tune_pluck_cnt = 0u;
        s_tune_phase = TUNE_PHASE_WAIT_PLUCK;
        break;
    }

    /* ------------------------------------------------------------------
     * WAIT: generic hop-count decrement
     * ------------------------------------------------------------------ */
    case TUNE_PHASE_WAIT:
    {
        if (s_tune_wait_cnt > 0u) {
            s_tune_wait_cnt--;
        } else {
            s_tune_phase = s_tune_after_wait;
        }
        break;
    }

    /* ------------------------------------------------------------------
     * WAIT_PLUCK: detect a pluck near the target open-string frequency
     * ------------------------------------------------------------------ */
    case TUNE_PHASE_WAIT_PLUCK:
    {
        float open_f = s_open_freq[s_tune_str];
        float lo = open_f * 0.70f;
        float hi = open_f * 1.80f;
        float best_freq = 0.0f;
        float best_mag  = 0.0f;
        float best_err  = hi - lo + 1.0f;

        for (uint8_t j = 0u; j < active_string_count; j++) {
            float freq = active_frequencies[j];
            float mag  = active_magnitudes[j];

            if (freq < lo || freq > hi || mag <= 50.0f) {
                continue;
            }

            float err = fabsf(freq - open_f);
            if (best_freq == 0.0f || err < best_err
                || (fabsf(err - best_err) < 0.01f && mag > best_mag)) {
                best_freq = freq;
                best_mag  = mag;
                best_err  = err;
            }
        }

        if (best_freq > 0.0f) {
            s_tune_detected_f = best_freq;
            printf("[TUNE] str%u: pluck detected at %.2f Hz\r\n",
                   (unsigned)s_tune_str,
                   (double)s_tune_detected_f);
            s_tune_phase = TUNE_PHASE_MOTOR_MOVE;
        }

        if (s_tune_phase == TUNE_PHASE_WAIT_PLUCK) {
            s_tune_pluck_cnt++;
            if (s_tune_pluck_cnt >= TUNE_PLUCK_TIMEOUT) {
                printf("[TUNE] str%u: no pluck detected — skipping\r\n",
                       (unsigned)s_tune_str);
                s_tune_phase = TUNE_PHASE_RECORD;
            }
        }
        break;
    }

    /* ------------------------------------------------------------------
     * MOTOR_MOVE: initialize the controller state from the pluck reading.
     *
     * Tune runs as a pure FFT-driven closed loop. There is no LUT or
     * polynomial-based ballistic move here; PI_LOCK will generate the first
     * motor correction from the measured Hz error on the next hop.
     * ------------------------------------------------------------------ */
    case TUNE_PHASE_MOTOR_MOVE:
    {
        float target   = s_open_freq[s_tune_str];
        float measured = s_tune_detected_f;
        float delta_f  = target - measured;
        int32_t current_pos = g_shared_memory.current_positions[s_tune_str];

        printf("[TUNE] str%u: measured=%.2f Hz target=%.2f Hz delta=%.2f Hz "
               "→ closed-loop engage at %ld steps\r\n",
               (unsigned)s_tune_str,
               (double)measured, (double)target, (double)delta_f,
               (long)current_pos);

        if (fabsf(delta_f) < TUNE_LOCK_BAND_HZ) {
            /* Already in tune — skip to record */
            s_tune_last_valid_f = measured;
            s_tune_locked       = 1u;
            printf("[TUNE] str%u: already in tune!\r\n",
                   (unsigned)s_tune_str);
            s_tune_phase = TUNE_PHASE_RECORD;
            break;
        }

        s_tune_last_valid_f = measured;
        s_tune_pi_cmd_steps = current_pos;
        s_tune_target_steps = current_pos;
        s_tune_pi_integral = 0.0f;
        s_tune_pi_hop_cnt = 0u;
        s_tune_no_sig_cnt = 0u;
        s_tune_retry_cnt = 0u;

        /* Gate FFT and wait for pluck harmonics to decay before sensing. */
        fft_sensing_enabled = 0u;
        TuneWaitThen(TUNE_REEXCITE_WAIT_HOPS, TUNE_PHASE_BURST_REENABLE);
        break;
    }

    /* ------------------------------------------------------------------
     * WAIT_MOTOR: wait for motor to finish moving
     * ------------------------------------------------------------------ */
    case TUNE_PHASE_WAIT_MOTOR:
    {
        uint8_t arrived = !g_shared_memory.motor_is_moving[s_tune_str];
        s_tune_motor_cnt++;

        if (arrived || s_tune_motor_cnt >= TUNE_MOTOR_WAIT_HOPS) {
            /* Decide whether to fire solenoid or go straight to PI */
            uint8_t has_solenoid = (s_tune_str >= 1u && s_tune_str <= 3u);
            if (has_solenoid) {
                s_tune_phase = TUNE_PHASE_SOL_BURST;
            } else {
                /* No solenoid — rely on remaining ring from pluck.
                 * Skip a couple of hops for motor vibration to settle. */
                TuneWaitThen(2u, TUNE_PHASE_PI_LOCK);
            }
        }
        break;
    }

    /* ------------------------------------------------------------------
     * SOL_BURST: re-excite at the tracked resonance when available
     * ------------------------------------------------------------------ */
    case TUNE_PHASE_SOL_BURST:
    {
        float burst_freq = (s_tune_last_valid_f > 0.0f)
                         ? s_tune_last_valid_f
                         : ((s_tune_detected_f > 0.0f)
                              ? s_tune_detected_f
                              : s_open_freq[s_tune_str]);

        /* Gate FFT */
        fft_sensing_enabled = 0u;

        if (burst_freq >= 40.0f && burst_freq <= 800.0f) {
            InterCore_TriggerSolenoidHz(s_tune_str,
                                         burst_freq,
                                         TUNE_SOLENOID_DUTY,
                                         SOLENOID_MODE_TIMED,
                                         TUNE_REEXCITE_DURATION_MS, 0u);
        }

        /* Reset PI for upcoming lock phase */
        s_tune_pi_integral    = 0.0f;
        s_tune_pi_cmd_steps   = g_shared_memory.current_positions[s_tune_str];
        s_tune_pi_hop_cnt     = 0u;
        s_tune_pi_lock_consec = 0u;
        s_tune_no_sig_cnt     = 0u;

        TuneWaitThen(TUNE_REEXCITE_WAIT_HOPS, TUNE_PHASE_BURST_REENABLE);
        break;
    }

    /* ------------------------------------------------------------------
     * BURST_REENABLE: restore FFT sensing
     * ------------------------------------------------------------------ */
    case TUNE_PHASE_BURST_REENABLE:
    {
        fft_sensing_enabled = 1u;
        s_tune_phase = TUNE_PHASE_PI_LOCK;
        break;
    }

    /* ------------------------------------------------------------------
     * PI_LOCK: PI controller targets the open-string frequency.
     * Motor adjusts in BOTH directions (positive and negative steps).
     * ------------------------------------------------------------------ */
    case TUNE_PHASE_PI_LOCK:
    {
        float target_freq = s_open_freq[s_tune_str];
        float center_freq = (s_tune_last_valid_f > 0.0f)
                          ? s_tune_last_valid_f
                          : ((s_tune_detected_f > 0.0f)
                               ? s_tune_detected_f
                               : target_freq);
        float min_freq = target_freq * TUNE_TRACK_MIN_RATIO;
        float max_freq = target_freq * TUNE_TRACK_MAX_RATIO;
        float meas_f = 0.0f;
        uint8_t motor_busy = g_shared_memory.motor_is_moving[s_tune_str];
        TuneGetTrackedFreq(center_freq, min_freq, max_freq, &meas_f);

        /* ----- Valid FFT reading ----- */
        if (meas_f > 0.0f) {
            s_tune_last_valid_f = meas_f;
            s_tune_no_sig_cnt   = 0u;

            float error = target_freq - meas_f;

            if (fabsf(error) > TUNE_LOCK_BAND_HZ) {
                if (fabsf(error) <= TUNE_INTEGRAL_ENABLE_BAND_HZ) {
                    s_tune_pi_integral = CalClampf(
                        s_tune_pi_integral + TUNE_PI_KI * error * TUNING_SAMPLE_TIME_S,
                        -TUNE_PI_MAX_TRIM, TUNE_PI_MAX_TRIM);
                } else {
                    s_tune_pi_integral = 0.0f;
                }

                if (!motor_busy) {
                    float step_cap = TuneStepCapForError(fabsf(error));
                    float delta_cmd = CalClampf(TUNE_PI_KP * error + s_tune_pi_integral,
                                                -step_cap, step_cap);
                    int32_t cmd = (int32_t)CalClampf(
                        (float)s_tune_pi_cmd_steps + delta_cmd,
                        (float)(-s_cal_steps_max[s_tune_str]),
                        (float)s_cal_steps_max[s_tune_str]);

                    int32_t delta = cmd - s_tune_pi_cmd_steps;
                    if ((delta < 0 ? -delta : delta) >= (int32_t)TUNE_MIN_MOVE_STEPS) {
                        s_tune_pi_cmd_steps = cmd;
                        s_tune_target_steps = cmd;
                        InterCore_SendMotorGoto(s_tune_str, s_tune_pi_cmd_steps);
                    }
                }
                s_tune_pi_lock_consec = 0u;
            } else {
                s_tune_pi_lock_consec++;
            }

            if (s_tune_pi_lock_consec >= TUNE_LOCK_CONSEC_HOPS) {
                s_tune_locked = 1u;
                printf("[TUNE] str%u: LOCKED at %.2f Hz (err %.3f Hz) pos=%ld\r\n",
                       (unsigned)s_tune_str,
                       (double)meas_f, (double)(target_freq - meas_f),
                       (long)g_shared_memory.current_positions[s_tune_str]);
                s_tune_phase = TUNE_PHASE_RECORD;
                break;
            }
        } else {
            /* ----- No FFT signal ----- */
            if (!motor_busy) {
                s_tune_no_sig_cnt++;
            }

            if (s_tune_no_sig_cnt >= TUNE_DEAD_HOPS) {
                s_tune_retry_cnt++;

                if (s_tune_retry_cnt < TUNE_MAX_RETRIES) {
                    uint8_t has_solenoid = (s_tune_str >= 1u && s_tune_str <= 3u);
                    if (has_solenoid) {
                        printf("[TUNE] str%u: signal lost — re-excite (retry %u/%u)\r\n",
                               (unsigned)s_tune_str,
                               (unsigned)s_tune_retry_cnt,
                               (unsigned)TUNE_MAX_RETRIES);
                        s_tune_phase = TUNE_PHASE_SOL_BURST;
                    } else {
                        printf("[TUNE] str%u: signal lost — pluck again! (retry %u/%u)\r\n",
                               (unsigned)s_tune_str,
                               (unsigned)s_tune_retry_cnt,
                               (unsigned)TUNE_MAX_RETRIES);
                        s_tune_pluck_cnt = 0u;
                        s_tune_phase = TUNE_PHASE_WAIT_PLUCK;
                    }
                } else {
                    printf("[TUNE] str%u: max retries reached — leaving offset unchanged\r\n",
                           (unsigned)s_tune_str);
                    s_tune_phase = TUNE_PHASE_RECORD;
                }
                break;
            }
        }

        /* ----- Timeout ----- */
        if (!motor_busy) {
            s_tune_pi_hop_cnt++;
        }
        if (s_tune_pi_hop_cnt >= TUNE_PI_LOCK_TIMEOUT
            && s_tune_pi_lock_consec == 0u) {
            s_tune_retry_cnt++;

            if (s_tune_retry_cnt < TUNE_MAX_RETRIES) {
                uint8_t has_solenoid = (s_tune_str >= 1u && s_tune_str <= 3u);
                if (has_solenoid) {
                    printf("[TUNE] str%u: lock timeout — re-excite (retry %u/%u)\r\n",
                           (unsigned)s_tune_str,
                           (unsigned)s_tune_retry_cnt,
                           (unsigned)TUNE_MAX_RETRIES);
                    s_tune_phase = TUNE_PHASE_SOL_BURST;
                } else {
                    printf("[TUNE] str%u: lock timeout — pluck again! (retry %u/%u)\r\n",
                           (unsigned)s_tune_str,
                           (unsigned)s_tune_retry_cnt,
                           (unsigned)TUNE_MAX_RETRIES);
                    s_tune_pluck_cnt = 0u;
                    s_tune_phase = TUNE_PHASE_WAIT_PLUCK;
                }
            } else {
                printf("[TUNE] str%u: timeout after %u retries — leaving offset unchanged\r\n",
                       (unsigned)s_tune_str,
                       (unsigned)TUNE_MAX_RETRIES);
                s_tune_phase = TUNE_PHASE_RECORD;
            }
        }
        break;
    }

    /* ------------------------------------------------------------------
     * RECORD: save only a confirmed in-tune offset, then advance
     * ------------------------------------------------------------------ */
    case TUNE_PHASE_RECORD:
    {
        int32_t actual_pos = g_shared_memory.current_positions[s_tune_str];

        /* Persist only a confirmed in-tune position. */
        if (s_tune_locked) {
            s_tune_offset[s_tune_str] = actual_pos;
            printf("[TUNE] str%u: offset = %ld steps (pos=%ld)\r\n",
                   (unsigned)s_tune_str,
                   (long)s_tune_offset[s_tune_str],
                   (long)actual_pos);
        } else {
            printf("[TUNE] str%u: no confirmed lock — offset unchanged (%ld)\r\n",
                   (unsigned)s_tune_str,
                   (long)s_tune_offset[s_tune_str]);
        }

        /* Advance to next string in mask */
        do { s_tune_str++; }
        while (s_tune_str < NUM_STRINGS && !((s_tune_mask >> s_tune_str) & 1u));

        if (s_tune_str >= NUM_STRINGS) {
            /* All strings done — complete immediately (same hop) so we
             * don't depend on an extra audio hop that may never arrive. */
            fft_sensing_enabled = 1u;

            /* Zero motor positions (we're at open fret) and disable all */
            for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
                if ((s_tune_mask >> i) & 1u) {
                    InterCore_SendMotorZero(i);
                }
            }
            InterCore_SendMotorEnableMask(0u);

            printf("[TUNE] ================================================\r\n");
            printf("[TUNE] Tuning complete. Offsets:");
            for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
                if ((s_tune_mask >> i) & 1u) {
                    printf(" str%u=%ld", (unsigned)i, (long)s_tune_offset[i]);
                }
            }
            printf("\r\n");
            printf("[TUNE] Motors zeroed and disabled.\r\n");
            printf("[TUNE] ================================================\r\n");

            s_tune_status = CAL_STATUS_DONE;
            s_tune_phase  = TUNE_PHASE_IDLE;
        } else {
            s_tune_phase = TUNE_PHASE_STR_INIT;
        }
        break;
    }

    /* ------------------------------------------------------------------
     * DONE: (kept for safety — normally reached via RECORD above)
     * ------------------------------------------------------------------ */
    case TUNE_PHASE_DONE:
    {
        fft_sensing_enabled = 1u;

        /* Zero motor positions (we're at open fret) and disable all */
        for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
            if ((s_tune_mask >> i) & 1u) {
                InterCore_SendMotorZero(i);
            }
        }
        InterCore_SendMotorEnableMask(0u);

        printf("[TUNE] ================================================\r\n");
        printf("[TUNE] Tuning complete (via DONE phase). Offsets:");
        for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
            if ((s_tune_mask >> i) & 1u) {
                printf(" str%u=%ld", (unsigned)i, (long)s_tune_offset[i]);
            }
        }
        printf("\r\n");
        printf("[TUNE] Motors zeroed and disabled.\r\n");
        printf("[TUNE] ================================================\r\n");

        s_tune_status = CAL_STATUS_DONE;
        s_tune_phase  = TUNE_PHASE_IDLE;
        break;
    }

    /* ------------------------------------------------------------------
     * Terminal / idle
     * ------------------------------------------------------------------ */
    case TUNE_PHASE_IDLE:
    case TUNE_PHASE_ERROR:
    default:
        break;
    }
}
