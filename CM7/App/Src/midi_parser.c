#include "midi_parser.h"
#include "note_mapping.h"
#include "song_event.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define DEFAULT_TEMPO_US_PER_QN 500000

uint16_t unused_motor;

static uint32_t read_be32(const uint8_t* p)
{
    return (p[0]<<24)|(p[1]<<16)|(p[2]<<8)|p[3];
}

static uint16_t read_be16(const uint8_t* p)
{
    return (p[0]<<8)|p[1];
}

static uint32_t read_varlen(const uint8_t* data, uint32_t* index)
{
    uint32_t value = 0;
    uint8_t c;

    do {
        c = data[(*index)++];
        value = (value << 7) | (c & 0x7F);
    } while (c & 0x80);

    return value;
}

static float midi_to_freq(uint8_t note)
{
    return 440.0f * powf(2.0f, (note - 69) / 12.0f);
}

int MIDI_Parse(const uint8_t* data,
               uint32_t len,
               SongEvent_t* out_events,
               uint32_t max_events,
               uint32_t* out_count)
{
    if (len < 14) return -1;


    if (memcmp(data, "MThd", 4) != 0)
{
    printf("Invalid MIDI header\n");
    return -1;
}

    uint32_t index = 0;

    // Skip header chunk
    index += 4; // "MThd"
    index += 4; // header length
    index += 2; // format
    index += 2; // ntrks

    uint16_t ticks_per_qn = read_be16(&data[index]);
    index += 2;

    // Track chunk
    if (memcmp(&data[index], "MTrk", 4) != 0)
    {
        printf("Invalid Track header\n");
        return -2;
    }
    index += 4;

    uint32_t track_len = read_be32(&data[index]);
    index += 4;

    uint32_t track_end = index + track_len;

    uint32_t tempo_us = DEFAULT_TEMPO_US_PER_QN; //TODO: see what this is
    uint32_t current_time_ms = 0;

    typedef struct {
        uint8_t active;
        uint32_t start_time_ms;
        uint8_t velocity;
    } ActiveNote_t;

    ActiveNote_t active[128] = {0};

    uint32_t event_count = 0;
    uint8_t running_status = 0;

    while (index < track_end)
    {
        uint32_t delta = read_varlen(data, &index);

        uint32_t delta_ms = (uint32_t)(((uint64_t)delta * tempo_us) /((uint64_t)ticks_per_qn * 1000ULL));

        


        current_time_ms += delta_ms;

        uint8_t status = data[index];

        if (status & 0x80)
        {
            running_status = status;
            index++;
        }
        else
        {
            status = running_status;
        }

        if ((status & 0xF0) == 0x90) // Note On
        {
            uint8_t note = data[index++];
            uint8_t vel  = data[index++];

            if (note >= 128)continue;

            if (vel > 0)
            {
                active[note].active = 1;
                active[note].start_time_ms = current_time_ms;
                active[note].velocity = vel;
            }
            else
            {
                // Treat as note off
                if (active[note].active && event_count < max_events)
                {
                    SongEvent_t* e = &out_events[event_count++];

                    e->start_time_ms = active[note].start_time_ms;
                    e->duration_ms   = current_time_ms - active[note].start_time_ms;
                    e->freq          = midi_to_freq(note);
                    e->velocity      = active[note].velocity;

                 
                    NoteMap_Resolve(note, &e->string_id, &unused_motor);

                    active[note].active = 0;
                }
            }
        }
        else if ((status & 0xF0) == 0x80) // Note Off
        {
            uint8_t note = data[index++];
            index++; // velocity

            if (note >= 128) continue;

            if (active[note].active && event_count < max_events)
            {
                SongEvent_t* e = &out_events[event_count++];

                e->start_time_ms = active[note].start_time_ms;
                e->duration_ms   = current_time_ms - active[note].start_time_ms;
                e->freq          = midi_to_freq(note);
                e->velocity      = active[note].velocity;

                
                NoteMap_Resolve(note, &e->string_id, &unused_motor);

                active[note].active = 0;
            }
        }
        else if (status == 0xFF) // Meta
        {
            uint8_t type = data[index++];
            uint32_t length = read_varlen(data, &index);

            if (type == 0x51 && length == 3)
            {
                tempo_us =
                    (data[index]<<16) |
                    (data[index+1]<<8) |
                    data[index+2];
            }

            index += length;
        }
        else
        {
            // Skip other events
            index++;
            index++;
        }
    }

    *out_count = event_count;
    return 0;
}