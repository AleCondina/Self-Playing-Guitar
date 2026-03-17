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
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_pwr_ex.h"   /* HAL_PWREx_EnableBkSRAM             */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* =========================================================================
 * Calibration parameters
 * ========================================================================= */

/** Number of frets measured per string (open=0 is the anchor; we measure 1–7). */
#define CAL_NUM_FRETS               8u

/** Microsteps for one semitone (1/16 step, 200-step motor: 200×16/12 ≈ 267).
 *  Empirically one fret ≈ 1/12 of a full revolution = 1600 microsteps. */
#define CAL_STEPS_PER_FRET          1600

/** Motor max speed (steps/sec) during the inter-fret transit.
 *  At 3000 steps/sec a 1600-step move takes ≈ 533 ms — well within the
 *  1360 ms burst+wait window before the PI starts. */
#define CAL_MOTOR_SPEED_FAST        3000

/** Hops to wait after homing before listening for the initial pluck. */
#define CAL_HOME_SETTLE_HOPS        6u

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
    CAL_PHASE_STR_INIT,       /**< Enable motor, home, prompt user to pluck  */
    CAL_PHASE_WAIT_PLUCK,     /**< Wait for FFT pluck detection              */
    CAL_PHASE_WAIT,           /**< Generic hop-count wait                    */
    CAL_PHASE_SOL_BURST,      /**< Fire solenoid + start motor transit       */
    CAL_PHASE_BURST_REENABLE, /**< Restore FFT, reset PI                     */
    CAL_PHASE_PI_LOCK,        /**< PI fine-tune with early-exit re-excite    */
    CAL_PHASE_RECORD,         /**< Save (steps, freq) and advance fret       */
    CAL_PHASE_BUILD,          /**< Build LUT + polynomial, save SRAM         */
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

/* PI state (inline controller, reset per fret) */
static float       s_pi_integral       = 0.0f;
static int32_t     s_pi_cmd_steps      = 0;
static uint8_t     s_pi_hop_cnt        = 0u;
static uint8_t     s_pi_lock_consec    = 0u;

/* Last-valid-frequency tracking (for early-exit re-excite) */
static float       s_last_valid_freq   = 0.0f;
static uint8_t     s_pi_no_sig_cnt     = 0u;   /* consecutive no-signal hops  */

/* Generic wait */
static uint16_t    s_wait_cnt          = 0u;
static CalPhase_t  s_after_wait        = CAL_PHASE_IDLE;

/* Sample buffer */
static CalPoint_t  s_buf[CAL_MAX_SAMPLES];
static uint16_t    s_buf_n             = 0u;

/* =========================================================================
 * Private helpers
 * ========================================================================= */

static inline float CalClampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void WaitThen(uint16_t hops, CalPhase_t next)
{
    s_wait_cnt   = hops;
    s_after_wait = next;
    s_phase      = CAL_PHASE_WAIT;
}

static void StopSolenoid(uint8_t str_id)
{
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
static void CalGetFreq(float target, float search_band, float *out_freq)
{
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

static uint32_t CalChecksum(const CalPersist_t *p)
{
    const uint32_t *w         = (const uint32_t *)p;
    uint32_t        size_words = (sizeof(CalPersist_t) / sizeof(uint32_t)) - 2u;
    uint32_t        csum       = 0u;
    for (uint32_t i = 1u; i <= size_words; i++) csum ^= w[i];
    return csum;
}

static void Cal_EnableBkpSRAM(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_EnableBkSRAM();
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
static void Cal_BuildTable(uint8_t str)
{
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
static void Cal_FitPolynomial(uint8_t str)
{
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

static void Cal_SaveToSRAM(void)
{
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

uint8_t Cal_LoadFromSRAM(void)
{
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
void Cal_FitFromBaseLUT(void)
{
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

void Cal_Start(uint8_t string_mask)
{
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

void Cal_StartStr(uint8_t str_id)
{
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

void Cal_Hop(void)
{
    switch (s_phase) {

    /* ------------------------------------------------------------------
     * STR_INIT: enable motor, home to position 0, prompt manual pluck
     * ------------------------------------------------------------------ */
    case CAL_PHASE_STR_INIT:
    {
        InterCore_SendMotorEnableMask(1u << s_str);
        InterCore_SendMotorMaxSpeed(s_str, CAL_MOTOR_SPEED_FAST);
        InterCore_SendMotorGoto(s_str, 0);

        s_fret              = 1u;
        s_buf_n             = 0u;
        s_retry_cnt         = 0u;
        s_is_reexciting     = 0u;
        s_pluck_timeout_cnt = 0u;
        s_pluck_detected    = 0u;
        s_last_valid_freq   = 0.0f;
        s_pi_integral       = 0.0f;
        s_pi_cmd_steps      = 0;
        s_pi_hop_cnt        = 0u;
        s_pi_lock_consec    = 0u;
        s_pi_no_sig_cnt     = 0u;

        printf("[CAL] -- String %u (%.2f Hz) | fret7 target %.2f Hz\r\n",
               (unsigned)s_str,
               (double)s_open_freq[s_str],
               (double)cal_theoretical_freq[s_str][7u]);
        printf("[CAL] >>> Pluck string %u now!\r\n", (unsigned)s_str);

        /* Wait for motor to reach home, then listen for pluck */
        WaitThen(CAL_HOME_SETTLE_HOPS, CAL_PHASE_WAIT_PLUCK);
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
     * WAIT_PLUCK: detect manual pluck near the open-string frequency.
     * Times out after CAL_PLUCK_TIMEOUT hops so calibration always proceeds.
     * ------------------------------------------------------------------ */
    case CAL_PHASE_WAIT_PLUCK:
    {
        float lo = s_open_freq[s_str] * 0.70f;
        float hi = s_open_freq[s_str] * 1.35f;

        for (uint8_t j = 0u; j < active_string_count; j++) {
            if (active_frequencies[j] >= lo && active_frequencies[j] <= hi) {
                s_pluck_detected = 1u;
                printf("[CAL] str%u: pluck detected at %.2f Hz\r\n",
                       (unsigned)s_str, (double)active_frequencies[j]);
                break;
            }
        }

        if (s_pluck_detected) {
            /* s_fret=1, s_is_reexciting=0 → SOL_BURST will fire at fret 0
             * and move motor toward fret 1 LUT position.                    */
            s_phase = CAL_PHASE_SOL_BURST;
            break;
        }

        s_pluck_timeout_cnt++;
        if (s_pluck_timeout_cnt >= CAL_PLUCK_TIMEOUT) {
            printf("[CAL] str%u: no pluck — proceeding without reference\r\n",
                   (unsigned)s_str);
            s_phase = CAL_PHASE_SOL_BURST;
        }
        break;
    }

    /* ------------------------------------------------------------------
     * SOL_BURST: fire solenoid and (on primary burst) start motor transit.
     *
     * Primary burst (s_is_reexciting == 0):
     *   – Fires at the PREVIOUS fret's resonant frequency (s_fret − 1).
     *     This excites the string at its current resonance for maximum
     *     energy transfer.
     *   – Immediately commands motor to the NEXT fret's LUT position at
     *     CAL_MOTOR_SPEED_FAST.  The motor travels during the burst and
     *     wait so it arrives before the FFT re-opens.
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
            /* Fire at the fret the string is CURRENTLY AT (s_fret - 1) */
            s_burst_freq     = cal_theoretical_freq[s_str][s_fret - 1u];
            s_steps_expected = (int32_t)s_fret * CAL_STEPS_PER_FRET;

            /* Warn if motor is far from expected (possible stall on prev fret) */
            int32_t actual_pos = g_shared_memory.current_positions[s_str];
            int32_t pos_err    = actual_pos - (int32_t)((s_fret - 1u) * CAL_STEPS_PER_FRET);
            printf("[CAL] str%u fret%u | burst %.2f Hz, motor pos=%ld err=%ld%s\r\n",
                   (unsigned)s_str, (unsigned)s_fret,
                   (double)s_burst_freq, (long)actual_pos, (long)pos_err,
                   (pos_err > CAL_STALL_WARN_STEPS || pos_err < -CAL_STALL_WARN_STEPS)
                       ? " [STALL?]" : "");

            /* Start motor moving toward next LUT target DURING the burst+wait.
             * At 3000 steps/sec the 1600-step transit takes ≈ 533 ms;
             * the wait lasts 8×170 = 1360 ms → motor arrives well before
             * the FFT window re-opens.  Motor's own braking handles final decel. */
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
                    (float)TUNING_STEPS_MAX_DEFAULT);

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
        if (s_pi_hop_cnt >= CAL_PI_LOCK_TIMEOUT && s_pi_lock_consec == 0u) {
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

        StopSolenoid(s_str);

        /* Return motor to open-string position */
        InterCore_SendMotorMaxSpeed(s_str, CAL_MOTOR_SPEED_FAST);
        InterCore_SendMotorGoto(s_str, 0);

        /* Advance to next string in mask */
        do { s_str++; } while (s_str < NUM_STRINGS && !((s_string_mask >> s_str) & 1u));

        if (s_str >= NUM_STRINGS) {
            fft_sensing_enabled = 1u;   /* ensure FFT is re-enabled */
            printf("[CAL] ================================================\r\n");
            printf("[CAL] Calibration complete — LUT and polynomial saved.\r\n");
            printf("[CAL] All motors returning to open-string position.\r\n");
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
