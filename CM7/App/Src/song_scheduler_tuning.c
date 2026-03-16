#include "song_scheduler_tuning.h"
#include "inter_core_cm7.h"
#include "note_lookup.h"
#include "stm32h7xx_hal.h"
#include "shared_memory.h"
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

        int32_t pos = LookupMotorPosition(e->string_id, e->note);
        InterCore_SendMotorGoto(e->string_id, pos);

        HAL_Delay(20);

        InterCore_TriggerSolenoids(1 << e->string_id, e->duration_ms);
        printf("NOTE START: idx=%lu string=%u freq=%d pos=%ld dur=%lu vel=%u\n",
       s->index,
       e->string_id,
       (int)e->freq,
       (long)pos,
       e->duration_ms,
       e->velocity);


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

              g_shared_memory.solenoid_id = e->string_id;
              g_shared_memory.motor_command_flags |= CMD_STOP_SOLENOID;
              InterCore_NotifyCM4();

              printf("NOTE STOP: string=%u note=%u freq=%.2f at %lu\n",
              e->string_id,
              e->note,
              e->freq,
              now_ms);
            }
        }
    }
}