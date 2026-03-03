#include "song_scheduler_tuning.h"
#include <stdio.h>

//#include "inter_core_cm7.h" //uncoment later

void SongScheduler_Init(SongScheduler_t* s, SongEvent_t* events,uint32_t count,uint32_t start_time)
{
    s->events = events;
    s->count = count;
    s->index = 0;
    s->start_time_ms = start_time;
    s->note_active = 0;
    s->note_end_time = 0;
}

void SongScheduler_Update(SongScheduler_t* s,uint32_t now_ms)
{
    if (s->index >= s->count)
        return;

    uint32_t elapsed = now_ms - s->start_time_ms;

    SongEvent_t* e = &s->events[s->index];

    /* Start note */
   /* Start note */
if (!s->note_active && elapsed >= e->start_time_ms)
{
    printf("NOTE START: string=%d freq=%.2f dur=%lu at %lu\n",
           e->string_id,
           e->freq,
           e->duration_ms,
           now_ms);

    s->note_active = 1;
    s->note_end_time = e->start_time_ms + e->duration_ms;
}
else if (s->note_active && elapsed >= s->note_end_time)
{
    printf("NOTE STOP\n");

    s->note_active = 0;
    s->index++;
}
}