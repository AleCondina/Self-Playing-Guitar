#include "song_scheduler_tuning.h"
#include <stdio.h>

void SongScheduler_Init(SongScheduler_t* s,
                        SongEvent_t* events,
                        uint32_t count,
                        uint32_t start_time)
{
    s->events = events;
    s->count = count;
    s->index = 0;
    s->start_time_ms = start_time;

    for (int i = 0; i < MAX_ACTIVE_NOTES; i++)
    {
        s->active_notes[i].active = 0;
    }
}

void SongScheduler_Update(SongScheduler_t* s, uint32_t now_ms)
{
    uint32_t elapsed = now_ms - s->start_time_ms;

    // -------- START NOTES --------
    while (s->index < s->count)
    {
        SongEvent_t* e = &s->events[s->index];

        if (elapsed < e->start_time_ms)
            break;

        printf("NOTE START: idx=%lu string=%u motor=%u freq=%d dur=%lu vel=%u\n",
               s->index,
               e->string_id,
               e->motor_position,
               (int)e->freq,
               e->duration_ms,
               e->velocity);    // Replace with InterCore_PlayNote_Tune(...)

        for (int i = 0; i < MAX_ACTIVE_NOTES; i++)
        {
            if (!s->active_notes[i].active)
            {
                s->active_notes[i].active = 1;
                s->active_notes[i].event = *e;
                s->active_notes[i].end_time_ms =
                    e->start_time_ms + e->duration_ms;
                break;
            }
        }

        s->index++;
    }

    // -------- STOP NOTES --------
    for (int i = 0; i < MAX_ACTIVE_NOTES; i++)
    {
        if (s->active_notes[i].active)
        {
            if (elapsed >= s->active_notes[i].end_time_ms)
            {
                SongEvent_t* e = &s->active_notes[i].event;

                printf("NOTE STOP: string=%u motor=%u freq=%d at %lu\n",
                       e->string_id,
                       e->motor_position,
                       (int)e->freq,
                       now_ms); // Replace with InterCore_StopNote(...)

                s->active_notes[i].active = 0;
            }
        }
    }
}