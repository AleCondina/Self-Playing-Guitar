#ifndef SONG_SCHEDULER_TUNING_H
#define SONG_SCHEDULER_TUNING_H

#include <stdint.h>
#include "song_event.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_ACTIVE_NOTES 8

typedef struct
{
    uint8_t active;
    SongEvent_t event;
    uint32_t end_time_ms;

} ActiveNote_t;

typedef struct
{
    SongEvent_t* events;
    uint32_t count;
    uint32_t index;
    uint32_t start_time_ms;
    uint8_t finished;

    ActiveNote_t active_notes[MAX_ACTIVE_NOTES];

} SongScheduler_t;

void SongScheduler_Init(SongScheduler_t* s,
                        SongEvent_t* events,
                        uint32_t count,
                        uint32_t start_time);

void SongScheduler_Update(SongScheduler_t* s,
                          uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif