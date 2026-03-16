# Calibration Controller Redesign

## Problem with Current Implementation

1. **Crosstalk**: Solenoid runs `SOLENOID_MODE_INFINITE` while FFT senses simultaneously — actuator noise leaks into pickup.
2. **Wrong step range**: `CAL_MAX_STEPS=1200` but 1 semitone = 1600 microsteps, so the sweep can't even reach fret 1.
3. **Open-string only excitation**: Solenoid always drives at open-string frequency regardless of fret position — strings only vibrate at resonance, so higher frets won't ring.
4. **No interleaving**: Actuation and sensing happen in the same FFT hop.

---

## New Design

**Fret-by-fret burst/sense interleaving:**

1. Move motor to each fret position (0–7) one at a time.
2. Settle (let motor stop and any overshoot die out).
3. Fire a **short solenoid burst** at the expected frequency for that fret.
4. Wait `CAL_SENSE_WAIT_HOPS` for the burst to finish and the FFT window to be fully post-burst.
5. Sense with FFT — record `(steps, measured_freq)` if valid.
6. Retry up to `CAL_MAX_RETRIES` times if no signal (with escalating burst energy on retry ≥2).
7. Skip fret (log warning) if all retries exhausted.
8. Build LUT from collected points. Abort only if fewer than `TUNING_FF_NUM_POINTS` valid samples.

---

## File to Modify

**`CM7/App/Src/calibration_controller.c`** — only file that changes.

---

## Constants

### Remove
```c
#define CAL_STEP_INCREMENT   20
#define CAL_MAX_STEPS        1200
#define CAL_HOME_HOPS        4u
#define CAL_SETTLE_HOPS      4u
#define CAL_NO_SIG_LIMIT     10u
```

### Add
```c
/** 1 semitone = 180° = 1600 microsteps at 1/16 step, 200-step motor */
#define CAL_STEPS_PER_FRET    1600

/** Number of frets to calibrate (open string + frets 1–7) */
#define CAL_NUM_FRETS         8

/** Motor max speed during calibration sweep (steps/sec).
 *  1200 steps/sec = 1.3 sec per fret move — moderate speed reduces overshoot. */
#define CAL_MOTOR_SPEED       1200

/** Hops to wait at home (fret 0) before first burst. Lets motor settle + string still. */
#define CAL_HOME_HOPS         10

/** Hops to wait after InterCore_SendMotorGoto() before bursting.
 *  At 1200 steps/sec, a 1600-step move takes ~8 hops. 8 settle hops gives ~2.7s total. */
#define CAL_FRET_SETTLE_HOPS  8

/** PWM cycles per solenoid burst. At E2 (82 Hz) 6 cycles ≈ 73ms — enough to excite resonance. */
#define CAL_BURST_CYCLES      6

/** Hops after burst before FFT sense. 2 hops = 340ms — ensures FFT window is fully post-burst.
 *  FFT window = 341ms (2048 samples @ 6kHz), hop = 170ms (1024 new samples). */
#define CAL_SENSE_WAIT_HOPS   2

/** Max burst retries at a single fret before skipping it. */
#define CAL_MAX_RETRIES       4
```

---

## State Variables

### Remove
```c
static uint8_t s_no_sig_cnt;
```

### Add
```c
static uint8_t s_fret;        /* Current fret index 0–7 */
static uint8_t s_retry_cnt;   /* Burst retry count for current fret */
```

---

## Phase Enum

### Remove
```c
CAL_PHASE_ADVANCE,
```

### Add
```c
CAL_PHASE_SOL_BURST,      /* Fire solenoid burst at current fret frequency */
CAL_PHASE_ADVANCE_FRET,   /* Send motor to next fret, arm settle wait      */
```

---

## New/Changed Functions

### Replace `StartSolenoid()` → `BurstSolenoid(str, fret)`

Old (INFINITE mode, always open-string freq):
```c
static void StartSolenoid(uint8_t str_id) {
    float freq = s_open_freq[str_id];
    InterCore_TriggerSolenoidHz(str_id, freq, CAL_SOLENOID_DUTY,
                                SOLENOID_MODE_INFINITE, 0u, 0u);
}
```

New (BURST mode, fret-correct frequency):
```c
static void BurstSolenoid(uint8_t str_id, uint8_t fret_id)
{
    /* Expected resonance frequency at this fret: open_freq * 2^(fret/12) */
    float freq = s_open_freq[str_id] * powf(2.0f, (float)fret_id / 12.0f);

    /* Escalate burst energy on retries (retry >= 2 → double cycles) */
    uint16_t cycles = (s_retry_cnt >= 2u) ? (CAL_BURST_CYCLES * 2u) : CAL_BURST_CYCLES;

    InterCore_TriggerSolenoidHz(str_id, freq, CAL_SOLENOID_DUTY,
                                SOLENOID_MODE_BURST, 0u, cycles);
}
```

---

## Rewritten `Cal_Hop()` State Machine

### `CAL_PHASE_STR_INIT`
```c
InterCore_SendMotorMaxSpeed(s_str, CAL_MOTOR_SPEED);
InterCore_SendMotorGoto(s_str, 0);

s_fret      = 0u;
s_steps     = 0;
s_buf_n     = 0u;
s_retry_cnt = 0u;

printf("[CAL] -- String %u | open=%.2f Hz | fret7=%.2f Hz\r\n", ...);
WaitThen(CAL_HOME_HOPS, CAL_PHASE_SOL_BURST);
```

### `CAL_PHASE_SOL_BURST`
```c
/* SUGGESTION: log motor position vs. expected to catch stalls */
int32_t actual_pos = g_shared_memory.current_positions[s_str];
printf("[CAL] str%u fret%u | motor_pos=%ld (expected %ld)\r\n",
       s_str, s_fret, (long)actual_pos, (long)s_steps);

BurstSolenoid(s_str, s_fret);
WaitThen(CAL_SENSE_WAIT_HOPS, CAL_PHASE_MEASURE);
```

### `CAL_PHASE_MEASURE`
```c
float open_f    = s_open_freq[s_str];
float expected  = open_f * powf(2.0f, (float)s_fret / 12.0f);
float meas_f    = (active_string_count > 0u) ? active_frequencies[0] : 0.0f;

/* Accept if within ±10% of expected fret frequency */
uint8_t valid = (active_string_count > 0u) &&
                (meas_f >= expected * 0.90f) &&
                (meas_f <= expected * 1.10f);

if (valid) {
    /* Record point */
    s_buf[s_buf_n].steps = (float)s_steps;
    s_buf[s_buf_n].freq  = meas_f;
    s_buf_n++;
    s_retry_cnt = 0u;
    printf("[CAL] %u, %ld, %.2f\r\n", s_str, (long)s_steps, (double)meas_f);

    s_phase = (s_fret == (CAL_NUM_FRETS - 1u)) ? CAL_PHASE_BUILD : CAL_PHASE_ADVANCE_FRET;

} else {
    s_retry_cnt++;
    if (s_retry_cnt < CAL_MAX_RETRIES) {
        /* Retry burst (BurstSolenoid will escalate cycles on retry >= 2) */
        s_phase = CAL_PHASE_SOL_BURST;
    } else {
        /* Skip this fret, warn, continue */
        printf("[CAL] str%u fret%u WARN: no signal after %u retries — skipping\r\n",
               s_str, s_fret, CAL_MAX_RETRIES);
        s_retry_cnt = 0u;
        s_phase = (s_fret == (CAL_NUM_FRETS - 1u)) ? CAL_PHASE_BUILD : CAL_PHASE_ADVANCE_FRET;
    }
}
```

### `CAL_PHASE_ADVANCE_FRET`
```c
s_fret++;
s_steps = (int32_t)s_fret * CAL_STEPS_PER_FRET;
InterCore_SendMotorGoto(s_str, s_steps);
WaitThen(CAL_FRET_SETTLE_HOPS, CAL_PHASE_SOL_BURST);
```

### `CAL_PHASE_BUILD`
```c
Cal_BuildTable(s_str);   /* unchanged — picks 4 evenly-spaced breakpoints from buffer */

StopSolenoid(s_str);     /* safety: kill any lingering solenoid */
InterCore_SendMotorGoto(s_str, 0);

/* Advance to next string in mask */
do { s_str++; } while (s_str < NUM_STRINGS && !((s_string_mask >> s_str) & 1u));

if (s_str >= NUM_STRINGS) {
    s_phase  = CAL_PHASE_DONE;
    s_status = CAL_STATUS_DONE;
} else {
    WaitThen(CAL_HOME_HOPS, CAL_PHASE_STR_INIT);
}
```

---

## `Cal_BuildTable()` — No Changes Needed

Existing logic picks `TUNING_FF_NUM_POINTS` (4) evenly-spaced frequency breakpoints from the `s_buf` array. With 8 fret samples it will select approximately frets 0, 2, 5, 7. The function already handles fewer-than-expected samples gracefully (`CAL_MIN_SAMPLES` guard).

---

## Timing Budget

| Phase | Hops | Time |
|---|---|---|
| Home settle (per string) | 10 | ~1.7s |
| Per-fret transition × 7 advances | 8 settle + 1 burst + 2 wait + 1 measure = 12 | ~2.0s each |
| Open string (fret 0) burst+sense | 1 + 2 + 1 = 4 | ~0.7s |
| **Total per string** | ~98 hops | **~16.6s** |
| **All 6 strings** | ~590 hops | **~1.7 min** |

---

## Suggestions / Future Improvements

1. **Motor stall detection**: In `SOL_BURST`, compare `g_shared_memory.current_positions[s_str]` vs. `s_steps`. If error > 200 steps, log a stall warning — the motor may have missed steps.

2. **Escalating burst energy**: Already implemented — on `s_retry_cnt >= 2`, `BurstSolenoid()` doubles `CAL_BURST_CYCLES` to 12.

3. **FFT amplitude logging**: Log the raw FFT amplitude alongside frequency to help tune `TUNING_SNR_THRESHOLD` post-calibration.

4. **Ring-out guard**: If a previous fret measurement was valid, the string may still be ringing at the new fret after motor move. On the first attempt at each fret, optionally skip the burst and just sense directly — only burst if no signal is detected.

5. **Finer LUT resolution**: If 8 fret points prove insufficient, `CAL_STEPS_PER_FRET` can be halved and `CAL_NUM_FRETS` doubled to calibrate at half-semitone intervals (requires re-tuning `CAL_FRET_SETTLE_HOPS`).

---

## Verification

1. Build CM7 firmware and flash.
2. Open UART monitor (115200 baud).
3. Send serial command: `calibrate 63` (all 6 strings, mask = 0x3F).
4. Expected UART output per string:
   ```
   [CAL] -- String 0 | open=82.41 Hz | fret7=123.47 Hz
   [CAL] 0, 0, 82.XX        ← fret 0
   [CAL] 0, 1600, 87.XX     ← fret 1
   ...
   [CAL] 0, 11200, 123.XX   ← fret 7
   [CAL-LUT] 0, 0, 82.XX, 0
   [CAL-LUT] 0, 1, 95.XX, 3733
   [CAL-LUT] 0, 2, 109.XX, 7467
   [CAL-LUT] 0, 3, 123.XX, 11200
   ```
5. No `ERROR: no signal` lines (retry WARNs are OK and expected on first runs).
6. After `CAL_STATUS_DONE`, resume normal play and verify tuning controller tracks correctly.
