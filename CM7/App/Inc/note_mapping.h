#ifndef NOTE_MAPPING_H
#define NOTE_MAPPING_H

#include <stdint.h>

void NoteMap_Resolve(uint8_t midi_note,
                     uint8_t* string_id,
                     uint16_t* motor_position);

#endif