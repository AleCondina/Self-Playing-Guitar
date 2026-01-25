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

/* External timer handles (defined in main.c) */
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;

/**
 * @brief Solenoid state structure for multi-solenoid management
 */
typedef struct {
    TIM_HandleTypeDef *htim;     /* Timer handle */
    uint32_t channel;             /* TIM_CHANNEL_1 or TIM_CHANNEL_2 */
    uint8_t current_string;       /* Last set string (0-2) */
    uint8_t current_fret;         /* Last set fret (0-7) */
    uint8_t duty_cycle;           /* Current duty cycle (1-99) */
    uint8_t is_active;            /* 1 if PWM running, 0 if stopped */
} SolenoidState_t;

/* Solenoid state array (3 solenoids) */
static SolenoidState_t solenoid_states[SOLENOID_COUNT];

/**
 * @brief Frequency lookup table: 6 strings × 8 frets → ARR values
 * @note  ARR = (640,000 / target_frequency) - 1
 * @note  Timer clock: 64 MHz, Prescaler: 99 (÷100) → 640 kHz count frequency
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
static const uint16_t SOLENOID_FREQ_TABLE[6][8] = {
    /* String 0: E2 (82.41 Hz) - Low E */
    {7764, 7329, 6918, 6530, 6163, 5816, 5488, 5179},  /* Frets 0-7 */

    /* String 1: A2 (110.00 Hz) */
    {5817, 5490, 5181, 4890, 4615, 4356, 4112, 3881},  /* Frets 0-7 */

    /* String 2: D3 (146.83 Hz) */
    {4358, 4113, 3882, 3664, 3459, 3265, 3082, 2910},  /* Frets 0-7 */

    /* String 3: G3 (196.00 Hz) */
    {3264, 3081, 2909, 2746, 2592, 2446, 2309, 2179},  /* Frets 0-7 */

    /* String 4: B3 (246.94 Hz) */
    {2591, 2445, 2308, 2179, 2056, 1941, 1832, 1729},  /* Frets 0-7 */

    /* String 5: E4 (329.63 Hz) - High E */
    {1941, 1832, 1729, 1632, 1540, 1454, 1372, 1295},  /* Frets 0-7 */
};

/**
 * @brief Initialize all solenoid PWM channels
 */
int32_t Solenoid_Init(void)
{
    /* Verify timer handles are initialized */
    if (htim12.Instance != TIM12 || htim12.State == HAL_TIM_STATE_RESET) {
        return -1;  /* TIM12 not initialized */
    }
    if (htim13.Instance != TIM13 || htim13.State == HAL_TIM_STATE_RESET) {
        return -1;  /* TIM13 not initialized */
    }

    /* Initialize solenoid 0: TIM12 CH1 (E2 string) */
    solenoid_states[SOLENOID_STRING_E2].htim = &htim12;
    solenoid_states[SOLENOID_STRING_E2].channel = TIM_CHANNEL_1;
    solenoid_states[SOLENOID_STRING_E2].current_string = 0;
    solenoid_states[SOLENOID_STRING_E2].current_fret = 0;
    solenoid_states[SOLENOID_STRING_E2].duty_cycle = SOLENOID_DEFAULT_DUTY_CYCLE;
    solenoid_states[SOLENOID_STRING_E2].is_active = 0;

    /* Initialize solenoid 1: TIM12 CH2 (A2 string) */
    solenoid_states[SOLENOID_STRING_A2].htim = &htim12;
    solenoid_states[SOLENOID_STRING_A2].channel = TIM_CHANNEL_2;
    solenoid_states[SOLENOID_STRING_A2].current_string = 1;
    solenoid_states[SOLENOID_STRING_A2].current_fret = 0;
    solenoid_states[SOLENOID_STRING_A2].duty_cycle = SOLENOID_DEFAULT_DUTY_CYCLE;
    solenoid_states[SOLENOID_STRING_A2].is_active = 0;

    /* Initialize solenoid 2: TIM13 CH1 (D3 string) */
    solenoid_states[SOLENOID_STRING_D3].htim = &htim13;
    solenoid_states[SOLENOID_STRING_D3].channel = TIM_CHANNEL_1;
    solenoid_states[SOLENOID_STRING_D3].current_string = 2;
    solenoid_states[SOLENOID_STRING_D3].current_fret = 0;
    solenoid_states[SOLENOID_STRING_D3].duty_cycle = SOLENOID_DEFAULT_DUTY_CYCLE;
    solenoid_states[SOLENOID_STRING_D3].is_active = 0;

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

    /* Get new ARR value from lookup table */
    uint16_t new_arr = SOLENOID_FREQ_TABLE[string][fret];

    /* Calculate new CCR for current duty cycle */
    uint16_t new_ccr = (new_arr * state->duty_cycle) / 100;

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

    /* Recalculate CCR based on current ARR */
    uint16_t current_arr = __HAL_TIM_GET_AUTORELOAD(state->htim);
    uint16_t new_ccr = (current_arr * duty_percent) / 100;

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

    /* Start PWM output if not already running */
    if (!state->is_active) {
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

    /* Stop PWM output if currently running */
    if (state->is_active) {
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
uint32_t Solenoid_GetFrequency(uint8_t solenoid_id)
{
    if (solenoid_id >= SOLENOID_COUNT) {
        return 0;  /* Invalid ID */
    }

    SolenoidState_t *state = &solenoid_states[solenoid_id];

    /* Get current ARR value */
    uint16_t arr = __HAL_TIM_GET_AUTORELOAD(state->htim);

    /* Calculate frequency: freq = 640,000 / (ARR + 1) */
    uint32_t freq_hz = 640000UL / (arr + 1);

    return freq_hz;
}
