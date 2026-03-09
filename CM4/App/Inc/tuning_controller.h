#ifndef TUNING_CONTROLLER_H
#define TUNING_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float target_hz;
    float measured_hz;

    // controller state
    float err_hz;
    float i_err;
    float kp, ki;     // simple PI is enough to start
    float max_step_per_update;  // limit motor move rate

    int32_t motor_pos;     // your absolute position units
    int32_t motor_pos_min;
    int32_t motor_pos_max;

    uint8_t active;
    uint32_t note_end_ms;  // when to stop exciting

    // excite parameters
    uint8_t velocity;
} StringTuner_t;

void TuningController_Init(void);

/**
 * Called when CM7 requests a new note (motor-based tuning).
 */
void TuningController_StartNote(uint8_t string_id,
                                float target_hz,
                                uint16_t duration_ms,
                                uint8_t velocity,
                                uint32_t now_ms);

/**
 * Call every 1ms (or 2ms). Provide latest measured pitch per string (Hz).
 * If you only measure one string at a time, pass 0 for others.
 */
void TuningController_Update(uint32_t now_ms, const float measured_hz[6]);

#ifdef __cplusplus
}
#endif

#endif /* TUNING_CONTROLLER_H */
