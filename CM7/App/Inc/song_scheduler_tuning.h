#ifndef SONG_SCHEDULER_TUNING_H
#define SONG_SCHEDULER_TUNING_H

#include <stdint.h>
#include <stddef.h>
#include "musicxml_parser.h"
#include "inter_core_cm7.h"  
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const MusicXml_NoteEvent_t* events;
    size_t count;
    size_t index;
    uint32_t song_start_ms;

    // simple monophonic: one active note at a time
    uint32_t active_until_ms;

    // default behavior: choose a string automatically (min motor movement)
    uint8_t  default_velocity;  // 0..127
} SongSchedulerTuning_t;

void SongSchedulerTuning_Init(SongSchedulerTuning_t* s,
                              const MusicXml_NoteEvent_t* events,
                              size_t count,
                              uint32_t start_ms);

void SongSchedulerTuning_Update(SongSchedulerTuning_t* s, uint32_t now_ms);

/**
 * Hook into your CM7->CM4 comms.
 * CM7 requests CM4 to tune a string to target_freq_hz and excite for duration_ms.
 *
 * string_id: 0..5
 * start_time_ms: absolute time (HAL_GetTick based) when CM4 should start (optional; you can ignore and execute immediately)
 */
int InterCore_PlayNote_Tune(uint8_t string_id,
                            float target_freq_hz,
                            uint16_t duration_ms,
                            uint8_t velocity,
                            uint32_t start_time_ms);

int InterCore_StopAll(void);

#ifdef __cplusplus
}
#endif

#endif /* SONG_SCHEDULER_TUNING_H */
