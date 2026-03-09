#ifndef MIDI_PARSER_H
#define MIDI_PARSER_H

#include <stdint.h>
#include "song_event.h"

int MIDI_Parse(const uint8_t* data,
               uint32_t len,
               SongEvent_t* out_events,
               uint32_t max_events,
               uint32_t* out_count);

#endif