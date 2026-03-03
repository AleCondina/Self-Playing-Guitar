#include "note_mapping.h"

void NoteMap_Resolve(uint8_t midi_note,
                     uint8_t* string_id,
                     uint16_t* motor_position)
{
    // Example simple mapping:
    // Divide MIDI range across 6 strings
    *string_id = midi_note % 6;

    // Example motor position calculation
    *motor_position = 1000 + (midi_note * 5);
}