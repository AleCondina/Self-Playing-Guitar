#ifndef SONG_EVENT_H
#define SONG_EVENT_H

#include <stdint.h>
#include "note_lookup.h"

typedef struct
{
    uint32_t start_time_ms;
    uint32_t duration_ms;

    float    freq;

    uint8_t  string_id;
    Note_t note;

    uint8_t  velocity;

} SongEvent_t;

#endif