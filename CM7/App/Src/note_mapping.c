#include "note_mapping.h"

static const uint8_t string_base_note[6] =
{
    40, // E2
    45, // A2
    50, // D3
    55, // G3
    59, // B3
    64  // E4
};

void NoteMap_Resolve(uint8_t midi_note,
                     uint8_t* string_id,
                     uint16_t* motor_position)
{
    int best_string = -1;
    int best_fret = 255;

    for(int s = 0; s < 6; s++)
    {
        int fret = midi_note - string_base_note[s];

        if(fret >= 0 && fret < best_fret && fret <= 20)
        {
            best_fret = fret;
            best_string = s;
        }
    }

    if(best_string == -1)
    {
        *string_id = 0;
        *motor_position = 0;
        return;
    }

    *string_id = best_string;

    // convert fret → motor steps
    *motor_position = best_fret * 100;
}