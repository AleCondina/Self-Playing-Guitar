#include "song_scheduler_tuning.h"
#include "musicxml_parser.h"



#include <math.h>

/* Standard MIDI -> frequency */
static float midi_to_freq(uint8_t midi)
{
    // f = 440 * 2^((midi-69)/12)
    return 440.0f * powf(2.0f, ((float)midi - 69.0f) / 12.0f);
}

/* Open string frequencies (standard guitar): E2 A2 D3 G3 B3 E4
   We use these to choose the best string (closest required pitch change). */
static const float g_open_freq[6] = {
    82.4069f, 110.0f, 146.8324f, 195.9977f, 246.9417f, 329.6276f
};

/* You should replace this later with a "current tuned frequency" estimate per string
   (from your calibration / last known motor position). For MVP we assume open tuning. */
static float g_est_current_freq[6];

static uint8_t choose_string_min_delta(float target_hz)
{
    uint8_t best = 0;
    float best_cost = 1e9f;
    for (uint8_t s = 0; s < 6; s++) {
        float cur = g_est_current_freq[s];
        float cost = fabsf(target_hz - cur);  // simplest “movement” metric
        if (cost < best_cost) { best_cost = cost; best = s; }
    }
    return best;
}

void SongSchedulerTuning_Init(SongSchedulerTuning_t* s,
                              const MusicXml_NoteEvent_t* events,
                              size_t count,
                              uint32_t start_ms)
{
    s->events = events;
    s->count  = count;
    s->index  = 0;
    s->song_start_ms = start_ms;
    s->active_until_ms = 0;
    s->default_velocity = 100;

    for (uint8_t i = 0; i < 6; i++)
        g_est_current_freq[i] = g_open_freq[i];
}

void SongSchedulerTuning_Update(SongSchedulerTuning_t* s, uint32_t now_ms)
{
    // stop everything if current note is done (optional)
    if (s->active_until_ms && now_ms >= s->active_until_ms) {
        (void)InterCore_StopAll();
        s->active_until_ms = 0;
    }

    if (s->index >= s->count)
        return;

    const MusicXml_NoteEvent_t* ev = &s->events[s->index];
    const uint32_t ev_abs_ms = s->song_start_ms + ev->start_ms;

    if (now_ms < ev_abs_ms)
        return;

    const float target_hz = midi_to_freq(ev->midi);

    // choose string that requires least change (MVP)
    const uint8_t string_id = choose_string_min_delta(target_hz);

    const uint8_t vel = (ev->velocity != 0) ? ev->velocity : s->default_velocity;

    // Ask CM4 to tune & excite
    (void)InterCore_PlayNote_Tune(string_id, target_hz, ev->dur_ms, vel, ev_abs_ms);

    // Update our estimate (helps choose next string)
    g_est_current_freq[string_id] = target_hz;

    s->active_until_ms = ev_abs_ms + ev->dur_ms;
    s->index++;
}
