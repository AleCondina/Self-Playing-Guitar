#ifndef MUSICXML_PARSER_H
#define MUSICXML_PARSER_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t start_ms;   // note start time from beginning of song (ms)
    uint16_t dur_ms;     // duration (ms)
    uint8_t  midi;       // MIDI note number
    uint8_t  velocity;   // placeholder (MusicXML velocity not always present)
} MusicXml_NoteEvent_t;

typedef struct {
    uint16_t divisions;  // ticks per quarter note (MusicXML "divisions")
    uint16_t tempo_bpm;  // beats per minute (quarter note)
} MusicXml_Info_t;

/**
 * @brief Parse a plain-text MusicXML buffer into a list of note events.
 *
 * Supported (MVP):
 *  - <divisions> inside <attributes>
 *  - tempo from <sound tempo="..."> OR <per-minute>
 *  - <note> with <pitch>/<step>, optional <alter>, <octave>
 *  - <duration>
 *  - <rest/> (advances time)
 *  - <chord/> (same start time as previous note, no time advance)
 *
 * Not supported:
 *  - compressed .mxl
 *  - multi-part/voice logic, tuplets, complex directions
 */
int MusicXML_ParseBuffer(const char* xml,
                         size_t xml_len,
                         MusicXml_NoteEvent_t* out_events,
                         size_t max_events,
                         size_t* out_count,
                         MusicXml_Info_t* out_info);

#ifdef __cplusplus
}
#endif

#endif /* MUSICXML_PARSER_H */
