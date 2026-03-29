/**
 ******************************************************************************
 * @file    solenoid_control.c
 * @brief   Solenoid control driver implementation
 * @note    PWM-based frequency control for guitar string vibration
 ******************************************************************************
 */

#include "solenoid_control.h"
#include "application_config.h"
#include "stm32h7xx_hal.h"

/* External timer handles (defined in main.c) — D2-domain timers only */
extern TIM_HandleTypeDef htim2;   /* Solenoid 2: A2 string (PA5) */
extern TIM_HandleTypeDef htim3;   /* Solenoid 1: E2 string (PB5) */
extern TIM_HandleTypeDef htim4;   /* Solenoid 3: D3 string (PB6) */

/**
 * @brief Solenoid state structure for multi-solenoid management
 */
typedef struct {
    TIM_HandleTypeDef *htim;     /* Timer handle */
    uint32_t channel;             /* TIM_CHANNEL_1 or TIM_CHANNEL_2 */
    uint8_t current_string;       /* Last set string (0-5) */
    uint8_t current_fret;         /* Last set fret (0-7) */
    uint8_t duty_cycle;           /* Current duty cycle (1-99) */
    uint8_t is_active;            /* 1 if PWM running, 0 if stopped */

    /* Auto-stop timing control */
    uint32_t start_timestamp_ms;      /* HAL_GetTick() when started */
    uint16_t target_duration_ms;      /* Milliseconds to run (0 = infinite) */
    uint16_t target_burst_cycles;     /* Cycles to run (0 = disabled) */
    uint32_t accumulated_cycles;      /* Cycles completed so far (calculated) */
    uint8_t auto_stop_mode;           /* 0=infinite, 1=timed, 2=burst */
} SolenoidState_t;

/* Solenoid state array (6 solenoids, one per guitar string) */
static SolenoidState_t solenoid_states[SOLENOID_COUNT];

/**
 * @brief Frequency lookup table: 6 strings × 8 frets → ARR values
 * @note  ARR = (2,200,000 / target_frequency) - 1
 * @note  Timer clock: 198 MHz, Prescaler: 89 (÷90) → 2.20 MHz count frequency
 *
 * Musical Note Reference (Standard Guitar Tuning):
 * String | Open Note | Freq (Hz) | Fret 7 (Hz)
 * -------|-----------|-----------|-------------
 *   0    | E2        |   82.41   |   123.47
 *   1    | A2        |  110.00   |   164.81
 *   2    | D3        |  146.83   |   220.00
 *   3    | G3        |  196.00   |   293.66
 *   4    | B3        |  246.94   |   369.99
 *   5    | E4        |  329.63   |   493.88
 */

static const uint16_t SOLENOID_FREQ_TABLE_STANDARD[6][8] = {
    /* String 0: E2 (82.41 Hz) - Low E */
    {26694, 25196, 23783, 22447, 21187, 19999, 18876, 17816},  /* Frets 0-7 */

    /* String 1: A2 (110.00 Hz) */
    {19999, 18876, 17816, 16816, 15873, 14982, 14141, 13347},  /* Frets 0-7 */

    /* String 2: D3 (146.83 Hz) */
    {14982, 14141, 13347, 12598, 11891, 11223, 10594, 9999},  /* Frets 0-7 */

    /* String 3: G3 (196.00 Hz) */
    {11223, 10594, 9999, 9438, 8907, 8407, 7936, 7491},  /* Frets 0-7 */

    /* String 4: B3 (246.94 Hz) */
    {8907, 8407, 7936, 7491, 7069, 6672, 6298, 5945},  /* Frets 0-7 */

    /* String 5: E4 (329.63 Hz) - High E */
    {6672, 6298, 5945, 5611, 5296, 4998, 4717, 4453},  /* Frets 0-7 */
};

/**
 * @brief Drop D tuning: String 0 = D2, String 1 = E2, Strings 2-5 unchanged
 * @note  ARR = round(2,200,000 / freq) - 1
 */
static const uint16_t SOLENOID_FREQ_TABLE_DROP_D[6][8] = {
    /* String 0: D2 (73.42 Hz) - Drop D */
    {29965, 28285, 26694, 25196, 23783, 22448, 21187, 19999},  /* Frets 0-7 */

    /* String 1: E2 (82.41 Hz) - was A2 in standard */
    {26694, 25196, 23783, 22447, 21187, 19999, 18876, 17816},  /* Frets 0-7 */

    /* String 2: D3 (146.83 Hz) - unchanged */
    {14982, 14141, 13347, 12598, 11891, 11223, 10594, 9999},  /* Frets 0-7 */

    /* String 3: G3 (196.00 Hz) - unchanged */
    {11223, 10594, 9999, 9438, 8907, 8407, 7936, 7491},  /* Frets 0-7 */

    /* String 4: B3 (246.94 Hz) - unchanged */
    {8907, 8407, 7936, 7491, 7069, 6672, 6298, 5945},  /* Frets 0-7 */

    /* String 5: E4 (329.63 Hz) - unchanged */
    {6672, 6298, 5945, 5611, 5296, 4998, 4717, 4453},  /* Frets 0-7 */
};

/**
 * @brief Drop G2 tuning: String 1 = G2 (A2 tuned down), all others unchanged
 * @note  ARR = round(2,200,000 / freq) - 1
 */
static const uint16_t SOLENOID_FREQ_TABLE_DROP_G2[6][8] = {
    /* String 0: E2 (82.41 Hz) - unchanged */
    {26694, 25196, 23783, 22447, 21187, 19999, 18876, 17816},  /* Frets 0-7 */

    /* String 1: G2 (98.00 Hz) - was A2 in standard */
    {22447, 21187, 19999, 18876, 17816, 16816, 15873, 14982},  /* Frets 0-7 */

    /* String 2: D3 (146.83 Hz) - unchanged */
    {14982, 14141, 13347, 12598, 11891, 11223, 10594, 9999},  /* Frets 0-7 */

    /* String 3: G3 (196.00 Hz) - unchanged */
    {11223, 10594, 9999, 9438, 8907, 8407, 7936, 7491},  /* Frets 0-7 */

    /* String 4: B3 (246.94 Hz) - unchanged */
    {8907, 8407, 7936, 7491, 7069, 6672, 6298, 5945},  /* Frets 0-7 */

    /* String 5: E4 (329.63 Hz) - unchanged */
    {6672, 6298, 5945, 5611, 5296, 4998, 4717, 4453},  /* Frets 0-7 */
};

/* Active frequency table pointer (switched by Solenoid_SetTuning) */
static const uint16_t (*active_freq_table)[8] = SOLENOID_FREQ_TABLE_STANDARD;

/**
 * @brief Initialize all solenoid PWM channels
 */
int32_t Solenoid_Init(void)
{
    /* Verify available timer handles are initialized */
    if (htim2.Instance != TIM2 || htim2.State == HAL_TIM_STATE_RESET) {
        return -1;  /* TIM2 not initialized */
    }
    if (htim3.Instance != TIM3 || htim3.State == HAL_TIM_STATE_RESET) {
        return -1;  /* TIM3 not initialized */
    }
    if (htim4.Instance != TIM4 || htim4.State == HAL_TIM_STATE_RESET) {
        return -1;  /* TIM4 not initialized */
    }

    /* Initialize solenoid 0: TIM3 CH2 (E2 string) on PB5 */
    solenoid_states[SOLENOID_STRING_E2].htim = &htim3;
    solenoid_states[SOLENOID_STRING_E2].channel = TIM_CHANNEL_2;
    solenoid_states[SOLENOID_STRING_E2].current_string = 0;
    solenoid_states[SOLENOID_STRING_E2].current_fret = 0;
    solenoid_states[SOLENOID_STRING_E2].duty_cycle = SOLENOID_DEFAULT_DUTY_CYCLE;
    solenoid_states[SOLENOID_STRING_E2].is_active = 0;

    /* Initialize solenoid 1: TIM2 CH1 (A2 string) on PA5 */
    solenoid_states[SOLENOID_STRING_A2].htim = &htim2;
    solenoid_states[SOLENOID_STRING_A2].channel = TIM_CHANNEL_1;
    solenoid_states[SOLENOID_STRING_A2].current_string = 1;
    solenoid_states[SOLENOID_STRING_A2].current_fret = 0;
    solenoid_states[SOLENOID_STRING_A2].duty_cycle = SOLENOID_DEFAULT_DUTY_CYCLE;
    solenoid_states[SOLENOID_STRING_A2].is_active = 0;

    /* Initialize solenoid 2: TIM4 CH1 (D3 string) on PB6 */
    solenoid_states[SOLENOID_STRING_D3].htim = &htim4;
    solenoid_states[SOLENOID_STRING_D3].channel = TIM_CHANNEL_1;
    solenoid_states[SOLENOID_STRING_D3].current_string = 2;
    solenoid_states[SOLENOID_STRING_D3].current_fret = 0;
    solenoid_states[SOLENOID_STRING_D3].duty_cycle = SOLENOID_DEFAULT_DUTY_CYCLE;
    solenoid_states[SOLENOID_STRING_D3].is_active = 0;

    /* Solenoid 3: G3 — no timer assigned */
    solenoid_states[SOLENOID_STRING_G3].htim = NULL;
    solenoid_states[SOLENOID_STRING_G3].channel = 0;
    solenoid_states[SOLENOID_STRING_G3].current_string = 3;
    solenoid_states[SOLENOID_STRING_G3].current_fret = 0;
    solenoid_states[SOLENOID_STRING_G3].duty_cycle = SOLENOID_DEFAULT_DUTY_CYCLE;
    solenoid_states[SOLENOID_STRING_G3].is_active = 0;

    /* Solenoid 4: B3 — no timer assigned (TIM16 removed, D1 domain) */
    solenoid_states[SOLENOID_STRING_B3].htim = NULL;
    solenoid_states[SOLENOID_STRING_B3].channel = 0;
    solenoid_states[SOLENOID_STRING_B3].current_string = 4;
    solenoid_states[SOLENOID_STRING_B3].current_fret = 0;
    solenoid_states[SOLENOID_STRING_B3].duty_cycle = SOLENOID_DEFAULT_DUTY_CYCLE;
    solenoid_states[SOLENOID_STRING_B3].is_active = 0;

    /* Solenoid 5: E4 — no timer assigned (TIM17 removed, D1 domain) */
    solenoid_states[SOLENOID_STRING_E4].htim = NULL;
    solenoid_states[SOLENOID_STRING_E4].channel = 0;
    solenoid_states[SOLENOID_STRING_E4].current_string = 5;
    solenoid_states[SOLENOID_STRING_E4].current_fret = 0;
    solenoid_states[SOLENOID_STRING_E4].duty_cycle = SOLENOID_DEFAULT_DUTY_CYCLE;
    solenoid_states[SOLENOID_STRING_E4].is_active = 0;

    return 0;
}

/**
 * @brief Set frequency for a specific solenoid by string and fret
 */
int32_t Solenoid_SetFrequency(uint8_t solenoid_id, uint8_t string, uint8_t fret)
{
    /* Validate inputs */
    if (solenoid_id >= SOLENOID_COUNT || string >= 6 || fret >= 8) {
        return -1;
    }

    SolenoidState_t *state = &solenoid_states[solenoid_id];

    /* Skip if no timer assigned (e.g. E2/B3/E4 without D2-domain timers) */
    if (state->htim == NULL) {
        state->current_string = string;
        state->current_fret = fret;
        return 0;
    }

    /* Get new ARR value from active tuning lookup table */
    uint16_t new_arr = active_freq_table[string][fret] * TIMER_CALIBRATION_FACTOR + 0.5f;

    /* Calculate new CCR for current duty cycle */
    uint16_t new_ccr = ((new_arr + 1) * state->duty_cycle) / 100;

    /* Update ARR and CCR atomically with auto-reload preload enabled */
    /* Both values will update simultaneously on next timer overflow */
    __HAL_TIM_SET_AUTORELOAD(state->htim, new_arr);
    __HAL_TIM_SET_COMPARE(state->htim, state->channel, new_ccr);

    /* Update state */
    state->current_string = string;
    state->current_fret = fret;

    return 0;
}

/**
 * @brief Set duty cycle for a specific solenoid
 */
int32_t Solenoid_SetDutyCycle(uint8_t solenoid_id, uint8_t duty_percent)
{
    /* Validate inputs */
    if (solenoid_id >= SOLENOID_COUNT) {
        return -1;
    }

    /* Clamp duty cycle to safe range (1-99%) */
    if (duty_percent == 0) {
        duty_percent = SOLENOID_DEFAULT_DUTY_CYCLE;
    } else if (duty_percent >= 100) {
        duty_percent = 99;
    }

    SolenoidState_t *state = &solenoid_states[solenoid_id];
    state->duty_cycle = duty_percent;

    /* Skip if no timer assigned */
    if (state->htim == NULL) {
        return 0;
    }

    /* Recalculate CCR based on current ARR (period = ARR + 1 counts) */
    uint16_t current_arr = __HAL_TIM_GET_AUTORELOAD(state->htim);
    uint16_t new_ccr = ((current_arr + 1) * duty_percent) / 100;

    /* Update CCR (will take effect on next timer period) */
    __HAL_TIM_SET_COMPARE(state->htim, state->channel, new_ccr);

    return 0;
}

/**
 * @brief Start PWM output for a specific solenoid
 */
void Solenoid_Start(uint8_t solenoid_id)
{
    if (solenoid_id >= SOLENOID_COUNT) {
        return;  /* Invalid ID, fail silently */
    }

    SolenoidState_t *state = &solenoid_states[solenoid_id];

    /* Start PWM output if not already running (skip if no timer assigned) */
    if (!state->is_active && state->htim != NULL) {
        if (HAL_TIM_PWM_Start(state->htim, state->channel) == HAL_OK) {
            state->is_active = 1;
        }
    }
}

/**
 * @brief Stop PWM output for a specific solenoid
 */
void Solenoid_Stop(uint8_t solenoid_id)
{
    if (solenoid_id >= SOLENOID_COUNT) {
        return;  /* Invalid ID, fail silently */
    }

    SolenoidState_t *state = &solenoid_states[solenoid_id];

    /* Stop PWM output if currently running (skip if no timer assigned) */
    if (state->is_active && state->htim != NULL) {
        HAL_TIM_PWM_Stop(state->htim, state->channel);
        state->is_active = 0;
    }
}

/**
 * @brief Stop all solenoids immediately
 */
void Solenoid_StopAll(void)
{
    for (uint8_t i = 0; i < SOLENOID_COUNT; i++) {
        Solenoid_Stop(i);
    }
}

/**
 * @brief Get active solenoid mask (bit field)
 */
uint8_t Solenoid_GetActiveMask(void)
{
    uint8_t mask = 0;

    for (uint8_t i = 0; i < SOLENOID_COUNT; i++) {
        if (solenoid_states[i].is_active) {
            mask |= (1U << i);
        }
    }

    return mask;
}

/**
 * @brief Get current frequency for a solenoid (diagnostics)
 */
float Solenoid_GetFrequency(uint8_t solenoid_id)
{
    if (solenoid_id >= SOLENOID_COUNT) {
        return 0;  /* Invalid ID */
    }

    SolenoidState_t *state = &solenoid_states[solenoid_id];

    /* No timer assigned */
    if (state->htim == NULL) {
        return 0;
    }

    /* Get current ARR value */
    uint16_t arr = __HAL_TIM_GET_AUTORELOAD(state->htim);

    /* ARR was set with calibration factor baked in, so the actual output
     * frequency is the nominal calculation divided by the factor */
    float freq_hz = ((float)TIM_PRESCALED_CLK_HZ / (arr + 1)) * TIMER_CALIBRATION_FACTOR;

    return freq_hz;
}

/**
 * @brief Start solenoid with auto-stop timing
 * @param solenoid_id Solenoid index (0-5)
 * @param mode 0=infinite, 1=timed, 2=burst
 * @param time_ms Duration in milliseconds (for timed mode, 0=infinite)
 * @param cycles Number of PWM cycles (for burst mode, 0=disabled)
 */
void Solenoid_StartTimed(uint8_t solenoid_id, uint8_t mode,
                         uint16_t time_ms, uint16_t cycles)
{
    if (solenoid_id >= SOLENOID_COUNT) {
        return;
    }

    SolenoidState_t *state = &solenoid_states[solenoid_id];

    /* Record timing parameters */
    state->auto_stop_mode = mode;
    state->target_duration_ms = time_ms;
    state->target_burst_cycles = cycles;
    state->start_timestamp_ms = HAL_GetTick();
    state->accumulated_cycles = 0;

    /* Start PWM output */
    Solenoid_Start(solenoid_id);
}

/**
 * @brief Update auto-stop timing for all solenoids
 * @return Bitmask of solenoids that auto-stopped this call
 * @note Call every 1ms from CM4 control loop
 */
uint8_t Solenoid_UpdateAutoStop(void)
{
    uint8_t stopped_mask = 0;
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < SOLENOID_COUNT; i++) {
        SolenoidState_t *state = &solenoid_states[i];

        /* Skip if not active or in infinite mode */
        if (!state->is_active || state->auto_stop_mode == 0) {
            continue;
        }

        /* Handle TIMED mode */
        if (state->auto_stop_mode == 1) {
            uint32_t elapsed_ms = now - state->start_timestamp_ms;

            if (elapsed_ms >= state->target_duration_ms) {
                Solenoid_Stop(i);
                stopped_mask |= (1 << i);
            }
        }
        /* Handle BURST mode */
        else if (state->auto_stop_mode == 2) {
            /* Calculate cycles completed from elapsed time and frequency */
            uint32_t elapsed_ms = now - state->start_timestamp_ms;
            uint32_t freq_hz = Solenoid_GetFrequency(i);

            /* Cycles = (elapsed_ms * freq_hz) / 1000 */
            uint32_t cycles = (elapsed_ms * freq_hz) / 1000;

            if (cycles >= state->target_burst_cycles) {
                Solenoid_Stop(i);
                stopped_mask |= (1 << i);
            }
        }
    }

    return stopped_mask;
}

/**
 * @brief Set solenoid drive frequency directly in Hz
 * @note  Timer count frequency = 2.20 MHz → ARR = (2,200,000 / freq_hz) - 1
 */
int32_t Solenoid_SetFrequencyHz(uint8_t solenoid_id, float freq_hz)
{
    if (solenoid_id >= SOLENOID_COUNT || freq_hz == 0u) {
        return -1;
    }

    /* Guard against ARR overflow (uint16_t max = 65535).
     * 2200000 / 65535 ≈ 34 Hz minimum; 2200000 / 1 = 2.2 MHz maximum.
     * For guitar strings 40–800 Hz is the expected range. */
    if (freq_hz < 40u || freq_hz > 800u) {
        return -1;
    }

    SolenoidState_t *state = &solenoid_states[solenoid_id];

    /* No timer assigned */
    if (state->htim == NULL) {
        return -1;
    }

    uint16_t new_arr = (uint16_t)(((TIM_PRESCALED_CLK_HZ / freq_hz) - 1.0f) * TIMER_CALIBRATION_FACTOR + 0.5f);
    uint16_t new_ccr = (uint16_t)(((new_arr + 1) * state->duty_cycle) / 100);

    __HAL_TIM_SET_AUTORELOAD(state->htim, new_arr);
    __HAL_TIM_SET_COMPARE(state->htim, state->channel, new_ccr);

    return 0;
}

/**
 * @brief Switch the active frequency lookup table between tuning modes
 * @param tuning_id  TUNING_STANDARD (0), TUNING_DROP_D (1), or TUNING_DROP_G2 (2)
 */
void Solenoid_SetTuning(uint8_t tuning_id)
{
    if (tuning_id == TUNING_DROP_D) {
        active_freq_table = SOLENOID_FREQ_TABLE_DROP_D;
    } else if (tuning_id == TUNING_DROP_G2) {
        active_freq_table = SOLENOID_FREQ_TABLE_DROP_G2;
    } else {
        active_freq_table = SOLENOID_FREQ_TABLE_STANDARD;
    }
}
