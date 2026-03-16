#ifndef NOTE_LOOKUP_H
#define NOTE_LOOKUP_H

#include <stdint.h>

typedef enum
{
    NOTE_E2 = 0,
    NOTE_F2,
    NOTE_G2,
    NOTE_A2,
    NOTE_B2,
    NOTE_COUNT
} Note_t;

int32_t LookupMotorPosition(uint8_t string_id, Note_t note);

#endif