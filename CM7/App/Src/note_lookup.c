#include "note_lookup.h"

static const int32_t motor_table[6][NOTE_COUNT] =
{
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0}
};

int32_t LookupMotorPosition(uint8_t string_id, Note_t note)
{
    if (string_id >= 6 || note >= NOTE_COUNT)
    return 0;
    
    return motor_table[string_id][note];
}