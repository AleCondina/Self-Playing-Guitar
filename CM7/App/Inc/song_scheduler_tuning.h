#ifndef SONG_SCHEDULER_TUNING_H
#define SONG_SCHEDULER_TUNING_H

#include <stdint.h>
#include "song_event.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    SongEvent_t* events;
    uint32_t count;
    uint32_t index;
    uint32_t start_time_ms;

    uint8_t  note_active;
    uint32_t note_end_time;

} SongScheduler_t;
/**
 * Initialize scheduler
 *
 * events      : array of parsed SongEvent_t
 * count       : number of events
 * start_time  : usually HAL_GetTick() at boot
 */
void SongScheduler_Init(SongScheduler_t* s,
                        SongEvent_t* events,
                        uint32_t count,
                        uint32_t start_time);


/**
 * Call periodically in main loop
 */
void SongScheduler_Update(SongScheduler_t* s,
                          uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif 