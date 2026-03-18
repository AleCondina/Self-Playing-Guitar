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
 *    Lock-band leeway: the PI only accumulates outside TUNING_LOCK_BAND_IN_HZ
 *    (1 Hz), and motor commands are suppressed below TUNING_MIN_MOVE_STEPS
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

/** Number of frets measured per string (open=0 is the anchor; we measure 1–7). */
#define CAL_NUM_FRETS               8u

/** Motor max speed (steps/sec) during the inter-fret transit.
 *  At 3000 steps/sec a 1600-step move takes ≈ 533 ms — well within the
 *  1360 ms burst+wait window before the PI starts. */
#define CAL_MOTOR_SPEED_FAST        3000

/** Hops to wait after homing before listening for the initial pluck. */
#define CAL_HOME_SETTLE_HOPS        6u

/** Motor position (steps) within which we consider the string "at home". */
#define CAL_HOME_POS_THRESHOLD      50

/** Max hops to wait for motor to reach home before proceeding anyway.
 *  Worst case: D3 at 9000 steps / 3000 steps/sec = 3.0 s → 18 hops.
 *  Add 12 hops for acceleration/braking margin → 30 hops total. */
#define CAL_HOME_TIMEOUT_HOPS       30u

/** Max hops to wait for the user to pluck (~4.3 s at 170 ms/hop). */
#define CAL_PLUCK_TIMEOUT           25u

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

/** Max re-excite retries during HOME_PI_LOCK.
 *  Higher than CAL_MAX_RETRIES because homing occurs after all frets have
 *  been measured — the string is old and decayed and needs more patience. */
#define CAL_HOME_MAX_RETRIES        8u

/** Re-excite burst duration (ms) used during HOME_PI_LOCK.
 *  Longer than CAL_REEXCITE_DURATION_MS to give the string a strong ring
 *  with enough decay time for the PI to converge at the open frequency.
 *  Wait hops = ceil((700 + 341) / 170) = 7 → reuse CAL_REEXCITE_WAIT_HOPS
 *  is slightly short; HOME burst re-enables after CAL_BURST_WAIT_HOPS (8). */
#define CAL_HOME_REEXCITE_MS        700u

/** Motor-position error (steps) that triggers a stall warning. */
#define CAL_STALL_WARN_STEPS        250

/** Sample buffer capacity.  7 fret measurements + headroom. */
#define CAL_MAX_SAMPLES             8u

/** Minimum valid samples required to build the LUT. */
#define CAL_MIN_SAMPLES             ((uint16_t)TUNING_FF_NUM_POINTS)

/* Backup SRAM base address and magic word (STM32H7, VBAT-backed, 4 KB). */
#define CAL_BKPSRAM_BASE            ((uint32_t)0x38800000UL)
#define CAL_PERSIST_MAGIC           0xCA115701UL

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
 * Theoretical equal-temperament fret frequencies [string][fret 0..7].
 * Derived from the named Hz constants in application_config.h — identical
 * to the values used by the CM4 solenoid controller's ARR table.
 */
static const float cal_theoretical_freq[NUM_STRINGS][CAL_NUM_FRETS] = {
    /* String 0: E2  frets 0-7 */
    { E2_HZ,  F2_HZ,  Fs2_HZ, G2_HZ,  Gs2_HZ, A2_HZ,  As2_HZ, B2_HZ  },
    /* String 1: A2  frets 0-7 */
    { A2_HZ,  As2_HZ, B2_HZ,  C3_HZ,  Cs3_HZ, D3_HZ,  Ds3_HZ, E3_HZ  },
    /* String 2: D3  frets 0-7 */
    { D3_HZ,  Ds3_HZ, E3_HZ,  F3_HZ,  Fs3_HZ, G3_HZ,  Gs3_HZ, A3_HZ  },
    /* String 3: G3  frets 0-7 */
    { G3_HZ,  Gs3_HZ, A3_HZ,  As3_HZ, B3_HZ,  C4_HZ,  Cs4_HZ, D4_HZ  },
    /* String 4: B3  frets 0-7 */
    { B3_HZ,  C4_HZ,  Cs4_HZ, D4_HZ,  Ds4_HZ, E4_HZ,  F4_HZ,  Fs4_HZ },
    /* String 5: E4  frets 0-7 */
    { E4_HZ,  F4_HZ,  Fs4_HZ, G4_HZ,  Gs4_HZ, A4_HZ,  As4_HZ, B4_HZ  },
};

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
    CAL_PHASE_OPEN_DONE,      /**< Record offset, proceed to fret cal        */
    CAL_PHASE_WAIT,           /**< Generic hop-count wait                    */
    CAL_PHASE_SOL_BURST,      /**< Fire solenoid + start motor transit       */
    CAL_PHASE_BURST_REENABLE, /**< Restore FFT, reset PI                     */
    CAL_PHASE_PI_LOCK,        /**< PI fine-tune with early-exit re-excite    */
    CAL_PHASE_RECORD,         /**< Save (steps, freq) and advance fret       */
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
 * s_fret is the measurement TARGET (1..CAL_NUM_FRETS-1).
 * The solenoid fires at fret (s_fret-1) to excite the string,
 * and the motor moves to s_fret's LUT position. */
static uint8_t     s_fret              = 1u;
static int32_t     s_steps_expected    = 0;   /* LUT target steps for s_fret */

/* Burst control */
static float       s_burst_freq        = 0.0f; /* Hz for the upcoming burst   */
static uint8_t     s_is_reexciting     = 0u;   /* 0=primary burst, 1=re-excite */

/* Retry and pluck tracking */
static uint8_t     s_retry_cnt         = 0u;
static uint8_t     s_pluck_timeout_cnt = 0u;
static uint8_t     s_pluck_detected    = 0u;
static float       s_open_detected_freq   = 0.0f; /* Hz detected at pluck time            */
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
    float open_f = s_open_freq[str];

    if (s_buf_n < CAL_MIN_SAMPLES) {
        printf("[CAL] str%u WARN: only %u samples — LUT unchanged\r\n",
               (unsigned)str, (unsigned)s_buf_n);
        return;
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
            tuning_ff_freq_bp [str][0] = open_f;
            tuning_ff_steps_bp[str][0] = 0.0f;
            printf("[CAL-LUT] %u, 0, %.2f, 0\r\n", (unsigned)str, (double)open_f);
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

/* =========================================================================
 * Persistent storage
 * ========================================================================= */

static void Cal_SaveToSRAM(void){
    Cal_EnableBkpSRAM();

    CalPersist_t *p = (CalPersist_t *)CAL_BKPSRAM_BASE;

    p->magic = CAL_PERSIST_MAGIC;
    memcpy(p->ff_freq,  tuning_ff_freq_bp,  sizeof(tuning_ff_freq_bp));
    memcpy(p->ff_steps, tuning_ff_steps_bp, sizeof(tuning_ff_steps_bp));

    for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
        p->poly_a[i] = cal_poly_coeff[i][0];
        p->poly_b[i] = cal_poly_coeff[i][1];
        p->poly_c[i] = cal_poly_coeff[i][2];
    }

    p->checksum = CalChecksum(p);
    __DSB();

    printf("[CAL] Saved to backup SRAM (0x%08lX, %u bytes)\r\n",
           (unsigned long)CAL_BKPSRAM_BASE, (unsigned)sizeof(CalPersist_t));
}

uint8_t Cal_LoadFromSRAM(void){
    Cal_EnableBkpSRAM();

    const CalPersist_t *p = (const CalPersist_t *)CAL_BKPSRAM_BASE;

    if (p->magic != CAL_PERSIST_MAGIC) {
        printf("[CAL] No stored calibration (magic 0x%08lX)\r\n",
               (unsigned long)p->magic);
        return 0u;
    }

    uint32_t expected = CalChecksum(p);
    if (p->checksum != expected) {
        printf("[CAL] Checksum mismatch (got 0x%08lX expected 0x%08lX) — ignored\r\n",
               (unsigned long)p->checksum, (unsigned long)expected);
        return 0u;
    }

    memcpy(tuning_ff_freq_bp,  p->ff_freq,  sizeof(tuning_ff_freq_bp));
    memcpy(tuning_ff_steps_bp, p->ff_steps, sizeof(tuning_ff_steps_bp));

    for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
        cal_poly_coeff[i][0] = p->poly_a[i];
        cal_poly_coeff[i][1] = p->poly_b[i];
        cal_poly_coeff[i][2] = p->poly_c[i];
    }

    printf("[CAL] Calibration restored from backup SRAM.\r\n");
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

    s_buf_n         = 0u;
    s_fret          = 1u;
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

        s_fret               = 1u;
        s_buf_n              = 0u;
        s_retry_cnt          = 0u;
        s_is_reexciting      = 0u;
        s_pluck_timeout_cnt  = 0u;
        s_pluck_detected     = 0u;
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

        printf("[CAL] -- String %u (%.2f Hz) | fret7 target %.2f Hz\r\n",
               (unsigned)s_str,
               (double)s_open_freq[s_str],
               (double)cal_theoretical_freq[s_str][7u]);
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
         * Magnitude threshold rejects broadband noise.                        */
        float lo = s_open_freq[s_str] * 0.70f;
        float hi = s_open_freq[s_str] * 1.80f;

        for (uint8_t j = 0u; j < active_string_count; j++) {
            if (active_frequencies[j] >= lo && active_frequencies[j] <= hi
                && active_magnitudes[j] > 50.0f) {
                s_pluck_detected = 1u;
                s_open_detected_freq = active_frequencies[j]; /* capture before j goes out of scope */
                printf("[CAL] str%u: pluck detected at %.2f Hz — computing homing target\r\n",
                       (unsigned)s_str, (double)s_open_detected_freq);
                break;
            }
        }

        if (s_pluck_detected) {
            /* s_open_detected_freq already captured inside the loop above. */

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

            /* Send motor to FF-estimated open position (not hardcoded 0).
             * WAIT_HOME will poll until motor reaches s_open_homing_target. */
            InterCore_SendMotorGoto(s_str, s_open_homing_target);
            s_home_timeout_cnt = 0u;
            s_after_home       = CAL_PHASE_OPEN_SOL_BURST;
            s_phase            = CAL_PHASE_WAIT_HOME;
            break;
        }

        s_pluck_timeout_cnt++;
        if (s_pluck_timeout_cnt >= CAL_PLUCK_TIMEOUT) {
            printf("[CAL] str%u: no pluck — homing to 0 and proceeding anyway\r\n",
                   (unsigned)s_str);
            s_open_homing_target = 0;
            InterCore_SendMotorGoto(s_str, 0);
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
        CalGetFreq(target_freq, search_band, &meas_f);

        /* ----- Valid FFT reading ----- */
        if (meas_f > 0.0f) {
            s_last_valid_freq = meas_f;
            s_pi_no_sig_cnt   = 0u;

            float error     = target_freq - meas_f;
            float abs_error = fabsf(error);

            /* Dynamic trim cap:
             * If the frequency error is shrinking → PI is working, let it
             * drive as far as needed (bounded by full mechanical range).
             * If error is NOT shrinking for CAL_OPEN_STALL_HOPS consecutive
             * hops → stall / dead string / false reading → re-engage the
             * hard CAL_PI_MAX_TRIM cap to stop runaway accumulation.       */
            if (abs_error < s_pi_last_abs_error) {
                s_pi_stall_cnt = 0u;                        /* converging — reset stall counter */
            } else {
                if (s_pi_stall_cnt < CAL_OPEN_STALL_HOPS) s_pi_stall_cnt++;
            }
            s_pi_last_abs_error = abs_error;

            float trim_cap = (s_pi_stall_cnt < CAL_OPEN_STALL_HOPS)
                             ? (float)s_cal_steps_max[s_str]  /* converging: full range  */
                             : CAL_PI_MAX_TRIM;               /* stalled:    hard cap    */

            if (abs_error > TUNING_LOCK_BAND_IN_HZ) {
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
                printf("[CAL] str%u: OPEN LOCKED at %.2f Hz (err %.3f Hz, pos=%ld)\r\n",
                       (unsigned)s_str, (double)meas_f,
                       (double)(target_freq - meas_f),
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
         * position 0 on CM4.  After this, poly(open_freq) ≈ 0 steps is
         * accurate and all fret polynomial targets are correct.           */
        InterCore_SendMotorZero(s_str);
        s_open_position_offset = 0;   /* offset is 0 — we re-zeroed */

        /* Reset state for fret calibration */
        s_retry_cnt     = 0u;
        s_is_reexciting = 0u;

        /* Proceed to fret 1 calibration */
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

            /* Fire solenoid at the string's ACTUAL current resonance.
             * – Fret 1 burst: use the measured open-string frequency from the
             *   pluck detection step (falls back to nominal if not detected).
             * – Later frets: use the last recorded calibration measurement so
             *   the solenoid matches the string's current tension state.      */
            if (s_buf_n == 0u) {
                s_burst_freq = (s_open_detected_freq > 0.0f)
                               ? s_open_detected_freq
                               : s_open_freq[s_str];
            } else {
                s_burst_freq = s_buf[s_buf_n - 1u].freq;
            }

            /* Estimate target motor position using the calibrated polynomial.
             * cal_poly_coeff is seeded at boot from the base LUT and updated
             * after every completed calibration run — estimates improve over
             * time.  The per-string safety cap prevents over-travel.          */
            {
                float _tf   = cal_theoretical_freq[s_str][s_fret];
                float _a    = cal_poly_coeff[s_str][0];
                float _b    = cal_poly_coeff[s_str][1];
                float _c    = cal_poly_coeff[s_str][2];
                float _poly = _a + _b * _tf + _c * _tf * _tf;
                if (_poly < 0.0f) _poly = 0.0f;
                _poly += (float)s_open_position_offset;
                s_steps_expected = (int32_t)CalClampf(_poly, 0.0f,
                                                      (float)s_cal_steps_max[s_str]);
            }

            /* Warn if motor is far from where it should be for the previous fret */
            int32_t actual_pos    = g_shared_memory.current_positions[s_str];
            int32_t prev_expected = (s_buf_n > 0u)
                                    ? (int32_t)s_buf[s_buf_n - 1u].steps
                                    : 0;
            int32_t pos_err       = actual_pos - prev_expected;
            printf("[CAL] str%u fret%u | burst %.2f Hz, motor pos=%ld err=%ld%s\r\n",
                   (unsigned)s_str, (unsigned)s_fret,
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

        /* Reset PI for the upcoming PI_LOCK phase */
        s_pi_integral    = 0.0f;
        s_pi_cmd_steps   = s_steps_expected;
        s_pi_hop_cnt     = 0u;
        s_pi_lock_consec = 0u;
        s_pi_no_sig_cnt  = 0u;
        s_last_valid_freq = 0.0f;

        WaitThen(wait_hops, CAL_PHASE_BURST_REENABLE);
        break;
    }

    /* ------------------------------------------------------------------
     * BURST_REENABLE: restore FFT sensing, transition to PI_LOCK
     * ------------------------------------------------------------------ */
    case CAL_PHASE_BURST_REENABLE:
    {
        fft_sensing_enabled = 1u;
        s_is_reexciting     = 0u;  /* clear for next SOL_BURST call */
        s_phase             = CAL_PHASE_PI_LOCK;
        break;
    }

    /* ------------------------------------------------------------------
     * PI_LOCK: fine-tune motor position using FFT feedback.
     *
     * Normal operation:
     *   Searches active_frequencies[] for the peak closest to the target
     *   fret frequency.  Runs a PI controller that adjusts the motor
     *   position around s_steps_expected.  The lock band (1 Hz) and
     *   minimum-move threshold (2 microsteps) prevent hunting once the
     *   string is in tune — these constants come from tuning_controller.h.
     *
     * Early-exit (string decayed):
     *   If CAL_DEAD_HOPS consecutive hops return no signal, the string has
     *   decayed.  The controller re-excites at s_last_valid_freq (the most
     *   recently confirmed FFT reading) so the burst resonance-matches the
     *   string's current tension state.  The motor goto is re-issued to
     *   continue any remaining travel.  After the re-excite wait, PI_LOCK
     *   resumes.  Up to CAL_MAX_RETRIES re-excites are attempted.
     *
     *   If CAL_PI_LOCK_TIMEOUT hops elapse without lock, the same
     *   re-excite logic triggers (the string may be fading).
     * ------------------------------------------------------------------ */
    case CAL_PHASE_PI_LOCK:
    {
        float target_freq = cal_theoretical_freq[s_str][s_fret];
        float search_band = target_freq * 0.15f;   /* ±15% search window  */

        float meas_f = 0.0f;
        CalGetFreq(target_freq, search_band, &meas_f);

        /* ----- Valid FFT reading ----- */
        if (meas_f > 0.0f) {
            s_last_valid_freq = meas_f;   /* update early-exit reference  */
            s_pi_no_sig_cnt   = 0u;

            float error = target_freq - meas_f;

            /* Deadband: only move motor outside the lock band.
             * TUNING_LOCK_BAND_IN_HZ (1 Hz) effectively acts as the deadband. */
            if (fabsf(error) > TUNING_LOCK_BAND_IN_HZ) {
                s_pi_integral = CalClampf(
                    s_pi_integral + CAL_PI_KI * error * TUNING_SAMPLE_TIME_S,
                    -CAL_PI_MAX_TRIM, CAL_PI_MAX_TRIM);

                float trim = CalClampf(CAL_PI_KP * error + s_pi_integral,
                                       -CAL_PI_MAX_TRIM, CAL_PI_MAX_TRIM);
                int32_t cmd = (int32_t)CalClampf(
                    (float)s_steps_expected + trim,
                    (float)TUNING_STEPS_MIN_DEFAULT,
                    (float)s_cal_steps_max[s_str]);

                /* Suppress tiny updates (TUNING_MIN_MOVE_STEPS = 2 microsteps)
                 * to prevent motor hunting at steady state. */
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
                printf("[CAL] str%u fret%u: LOCKED %.2f Hz (err %.3f Hz)\r\n",
                       (unsigned)s_str, (unsigned)s_fret,
                       (double)meas_f, (double)(target_freq - meas_f));
                s_phase = CAL_PHASE_RECORD;
                break;
            }
        } else {
            /* ----- No FFT signal ----- */
            s_pi_no_sig_cnt++;

            if (s_pi_no_sig_cnt >= CAL_DEAD_HOPS) {
                /* Early exit: string has decayed before we could lock.
                 * Re-excite at the last known resonance frequency so the
                 * burst matches the string's current state.              */
                s_retry_cnt++;

                if (s_retry_cnt < CAL_MAX_RETRIES) {
                    /* Use last valid reading; fall back to target if none */
                    s_burst_freq = (s_last_valid_freq > 0.0f)
                                   ? s_last_valid_freq
                                   : target_freq;

                    printf("[CAL] str%u fret%u: signal lost at %.2f Hz "
                           "— re-excite %.2f Hz (retry %u/%u)\r\n",
                           (unsigned)s_str, (unsigned)s_fret,
                           (double)s_last_valid_freq,
                           (double)s_burst_freq,
                           (unsigned)s_retry_cnt,
                           (unsigned)CAL_MAX_RETRIES);

                    s_is_reexciting = 1u;
                    s_phase         = CAL_PHASE_SOL_BURST;
                } else {
                    printf("[CAL] str%u fret%u WARN: max retries reached "
                           "— recording best estimate\r\n",
                           (unsigned)s_str, (unsigned)s_fret);
                    s_phase = CAL_PHASE_RECORD;
                }
                break;
            }
            /* else: keep waiting, string may still ring up */
        }

        /* ----- Timeout check (applies regardless of signal state) ----- */
        s_pi_hop_cnt++;
        if (s_pi_hop_cnt >= CAL_PI_LOCK_TIMEOUT &&
            s_pi_lock_consec < CAL_LOCK_CONSEC_HOPS) {
            /* Haven't achieved full lock within the timeout window.
             * Fires even with partial lock (s_pi_lock_consec > 0). */
            s_retry_cnt++;

            if (s_retry_cnt < CAL_MAX_RETRIES) {
                s_burst_freq = (s_last_valid_freq > 0.0f)
                               ? s_last_valid_freq
                               : target_freq;

                printf("[CAL] str%u fret%u: lock timeout (%.2f Hz) "
                       "— re-excite %.2f Hz (retry %u/%u)\r\n",
                       (unsigned)s_str, (unsigned)s_fret,
                       (double)(meas_f > 0.0f ? meas_f : 0.0f),
                       (double)s_burst_freq,
                       (unsigned)s_retry_cnt,
                       (unsigned)CAL_MAX_RETRIES);

                s_is_reexciting = 1u;
                s_phase         = CAL_PHASE_SOL_BURST;
            } else {
                printf("[CAL] str%u fret%u WARN: timeout after %u retries\r\n",
                       (unsigned)s_str, (unsigned)s_fret,
                       (unsigned)CAL_MAX_RETRIES);
                s_phase = CAL_PHASE_RECORD;
            }
        }
        break;
    }

    /* ------------------------------------------------------------------
     * RECORD: save the (steps, freq) measurement for this fret.
     *
     * Priority order for the recorded frequency:
     *   1. Current FFT reading (PI just locked).
     *   2. s_last_valid_freq (last accepted reading before decay).
     *   3. Expected fret frequency (fully estimated — logged with marker).
     * The recorded step count is always the actual motor position from
     * shared memory so it reflects any PI trim applied.
     * ------------------------------------------------------------------ */
    case CAL_PHASE_RECORD:
    {
        float target_freq = cal_theoretical_freq[s_str][s_fret];
        float search_band = target_freq * 0.15f;

        float meas_f = 0.0f;
        CalGetFreq(target_freq, search_band, &meas_f);

        int32_t actual_steps = g_shared_memory.current_positions[s_str];

        if (s_buf_n < CAL_MAX_SAMPLES) {
            if (meas_f > 0.0f) {
                /* Best case: current FFT is valid */
                s_buf[s_buf_n].steps = (float)actual_steps;
                s_buf[s_buf_n].freq  = meas_f;
                printf("[CAL] %u, %ld, %.2f\r\n",
                       (unsigned)s_str, (long)actual_steps, (double)meas_f);
            } else if (s_last_valid_freq > 0.0f) {
                /* Fallback: last valid reading before decay */
                s_buf[s_buf_n].steps = (float)actual_steps;
                s_buf[s_buf_n].freq  = s_last_valid_freq;
                printf("[CAL] %u, %ld, %.2f (last-valid)\r\n",
                       (unsigned)s_str, (long)actual_steps,
                       (double)s_last_valid_freq);
            } else {
                /* Fully estimated: use LUT expected values */
                s_buf[s_buf_n].steps = (float)s_steps_expected;
                s_buf[s_buf_n].freq  = target_freq;
                printf("[CAL] %u, %ld, %.2f (EST)\r\n",
                       (unsigned)s_str, (long)s_steps_expected,
                       (double)target_freq);
            }
            s_buf_n++;
        }

        /* Reset retry state and advance to the next fret */
        s_retry_cnt     = 0u;
        s_is_reexciting = 0u;

        if (s_fret >= (CAL_NUM_FRETS - 1u)) {
            /* All frets measured — proceed to build the LUT */
            s_phase = CAL_PHASE_BUILD;
        } else {
            /* Increment fret.  SOL_BURST will fire at the CURRENT (just-
             * measured) fret frequency and move toward the next one.       */
            s_fret++;
            s_phase = CAL_PHASE_SOL_BURST;
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
        Cal_SaveToSRAM();

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
        /* Use last measured fret frequency for excitation; fall back to
         * open-string frequency if no samples were recorded. */
        s_burst_freq = (s_buf_n > 0u) ? s_buf[s_buf_n - 1u].freq
                                       : s_open_freq[s_str];

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
        CalGetFreq(target_freq, search_band, &meas_f);

        /* ----- Valid FFT reading ----- */
        if (meas_f > 0.0f) {
            s_last_valid_freq = meas_f;
            s_pi_no_sig_cnt   = 0u;

            float error = target_freq - meas_f;

            if (fabsf(error) > TUNING_LOCK_BAND_IN_HZ) {
                s_pi_integral = CalClampf(
                    s_pi_integral + CAL_PI_KI * error * TUNING_SAMPLE_TIME_S,
                    -CAL_PI_MAX_TRIM, CAL_PI_MAX_TRIM);

                float trim = CalClampf(CAL_PI_KP * error + s_pi_integral,
                                       -CAL_PI_MAX_TRIM, CAL_PI_MAX_TRIM);
                int32_t cmd = (int32_t)CalClampf(
                    (float)s_open_position_offset + trim,   /* target is measured open position */
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
                printf("[CAL] str%u: HOME LOCKED at %.2f Hz (err %.3f Hz, pos=%ld)\r\n",
                       (unsigned)s_str, (double)meas_f,
                       (double)(target_freq - meas_f),
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
                    s_burst_freq = (s_last_valid_freq > 0.0f)
                                   ? s_last_valid_freq
                                   : target_freq;

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
                s_burst_freq = (s_last_valid_freq > 0.0f)
                               ? s_last_valid_freq
                               : target_freq;

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
