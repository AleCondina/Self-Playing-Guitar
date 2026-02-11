#include "musicxml_parser.h"
#include <string.h>
#include <stdlib.h>

/* ---- Small helpers ---- */

static const char* find_substr_limited(const char* hay, size_t hay_len, const char* needle)
{
    size_t nlen = strlen(needle);
    if (nlen == 0 || hay_len < nlen) return NULL;

    /* Simple bounded search */
    for (size_t i = 0; i + nlen <= hay_len; i++) {
        if (memcmp(hay + i, needle, nlen) == 0) {
            return hay + i;
        }
    }
    return NULL;
}

/* Reads integer that appears between open and close tags, within a bounded segment.
   Example: <divisions>480</divisions> */
static int read_int_between_tags(const char* seg, size_t seg_len,
                                const char* open_tag,
                                const char* close_tag,
                                int* out_val)
{
    const char* p1 = find_substr_limited(seg, seg_len, open_tag);
    if (!p1) return -1;
    p1 += strlen(open_tag);

    size_t remain = (size_t)(seg + seg_len - p1);
    const char* p2 = find_substr_limited(p1, remain, close_tag);
    if (!p2) return -1;

    /* Copy digits to temp buffer */
    char buf[24];
    size_t n = (size_t)(p2 - p1);
    if (n == 0 || n >= sizeof(buf)) return -1;

    memcpy(buf, p1, n);
    buf[n] = '\0';

    *out_val = atoi(buf);
    return 0;
}

/* Reads integer attribute like tempo="120" (bounded segment).
   Returns 0 if found. */
static int read_int_attribute(const char* seg, size_t seg_len,
                              const char* attr_prefix, /* e.g. tempo=" */
                              int* out_val)
{
    const char* p = find_substr_limited(seg, seg_len, attr_prefix);
    if (!p) return -1;
    p += strlen(attr_prefix);

    /* Read until quote or end */
    char buf[24];
    size_t i = 0;
    while (i + 1 < sizeof(buf) && (size_t)(p - seg) < seg_len) {
        char c = *p++;
        if (c == '"' || c == '\'') break;
        buf[i++] = c;
    }
    buf[i] = '\0';

    if (i == 0) return -1;
    *out_val = atoi(buf);
    return 0;
}

static uint8_t step_to_semitone(char step)
{
    switch (step) {
        case 'C': return 0;
        case 'D': return 2;
        case 'E': return 4;
        case 'F': return 5;
        case 'G': return 7;
        case 'A': return 9;
        case 'B': return 11;
        default:  return 0;
    }
}

/* Parse one <note>...</note> block (bounded) */
static int parse_note_block(const char* note, size_t note_len,
                            int* is_rest,
                            int* is_chord,
                            int* duration_ticks,
                            int* octave,
                            int* alter,
                            char* step)
{
    *is_rest = (find_substr_limited(note, note_len, "<rest") != NULL);
    *is_chord = (find_substr_limited(note, note_len, "<chord") != NULL);

    *duration_ticks = 0;
    *octave = 0;
    *alter = 0;
    *step = 'C';

    /* duration is required for timing */
    (void)read_int_between_tags(note, note_len, "<duration>", "</duration>", duration_ticks);

    /* pitch fields (ignored if rest) */
    int tmp = 0;
    if (read_int_between_tags(note, note_len, "<octave>", "</octave>", &tmp) == 0) {
        *octave = tmp;
    }
    if (read_int_between_tags(note, note_len, "<alter>", "</alter>", &tmp) == 0) {
        *alter = tmp;
    }

    /* <step>X</step> */
    const char* p = find_substr_limited(note, note_len, "<step>");
    if (p && (size_t)(p - note) + 6 < note_len) {
        *step = p[6]; /* char after "<step>" */
    }

    return 0;
}

int MusicXML_ParseBuffer(const char* xml,
                         size_t xml_len,
                         MusicXml_NoteEvent_t* out_events,
                         size_t max_events,
                         size_t* out_count,
                         MusicXml_Info_t* out_info)
{
    if (!xml || xml_len == 0 || !out_events || max_events == 0 || !out_count || !out_info)
        return -1;

    /* Defaults */
    out_info->divisions = 1;
    out_info->tempo_bpm = 120;
    *out_count = 0;

    /* Try to find <divisions> anywhere (good enough for MVP) */
    {
        int div = 0;
        if (read_int_between_tags(xml, xml_len, "<divisions>", "</divisions>", &div) == 0 && div > 0) {
            out_info->divisions = (uint16_t)div;
        }
    }

    /* Tempo: prefer <sound tempo="..."> then <per-minute> */
    {
        int tempo = 0;
        if (read_int_attribute(xml, xml_len, "tempo=\"", &tempo) == 0 && tempo > 0) {
            out_info->tempo_bpm = (uint16_t)tempo;
        } else if (read_int_between_tags(xml, xml_len, "<per-minute>", "</per-minute>", &tempo) == 0 && tempo > 0) {
            out_info->tempo_bpm = (uint16_t)tempo;
        }
        if (out_info->tempo_bpm == 0) out_info->tempo_bpm = 120;
    }

    /* Time accumulator in MusicXML ticks (divisions units) */
    uint32_t cur_ticks = 0;

    const uint32_t ms_per_quarter = (out_info->tempo_bpm > 0) ? (60000U / out_info->tempo_bpm) : 500U;
    const uint32_t divisions = (out_info->divisions > 0) ? out_info->divisions : 1;

    /* Scan through notes */
    size_t pos = 0;
    while (pos < xml_len) {
        const char* p_note = find_substr_limited(xml + pos, xml_len - pos, "<note");
        if (!p_note) break;

        /* Find end of this note */
        const char* p_end = find_substr_limited(p_note, (size_t)(xml + xml_len - p_note), "</note>");
        if (!p_end) break;

        const char* note_block = p_note;
        size_t note_len = (size_t)(p_end - p_note) + strlen("</note>");

        int is_rest = 0, is_chord = 0;
        int dur_ticks = 0, octave = 0, alter = 0;
        char step = 'C';
        parse_note_block(note_block, note_len, &is_rest, &is_chord, &dur_ticks, &octave, &alter, &step);

        if (dur_ticks < 0) dur_ticks = 0;

        if (is_rest) {
            /* rests advance time */
            cur_ticks += (uint32_t)dur_ticks;
        } else {
            /* compute MIDI */
            uint8_t midi = (uint8_t)(12 * (octave + 1) + step_to_semitone(step) + alter);

            uint32_t start_ticks = cur_ticks; /* chord notes share same start */
            uint32_t start_ms = (start_ticks * ms_per_quarter) / divisions;
            uint32_t dur_ms = ((uint32_t)dur_ticks * ms_per_quarter) / divisions;

            if (*out_count < max_events) {
                MusicXml_NoteEvent_t* ev = &out_events[*out_count];
                ev->start_ms = start_ms;
                ev->dur_ms   = (dur_ms > 0xFFFFu) ? 0xFFFFu : (uint16_t)dur_ms;
                ev->midi     = midi;
                ev->velocity = 100; /* MVP default */
                (*out_count)++;
            }

            /* only advance time for non-chord notes */
            if (!is_chord) {
                cur_ticks += (uint32_t)dur_ticks;
            }
        }

        /* Move scanning position past this note */
        pos = (size_t)((p_end + strlen("</note>")) - xml);
    }

    return 0;
}
