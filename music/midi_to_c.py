
#How to run
# exmaple : python midi_to_song.py (midi_file_name).mid target_BPM(Leave_empty_for original)
import sys
import mido
from mido import MidiFile

# =========================
# NOTE DEFINES (must match application_config.h)
# =========================

NOTE_MAP = {
    82.41: "E2_HZ",
    110.00: "A2_HZ",
    146.83: "D3_HZ",
    174.61: "F3_HZ",
    196.00: "G3_HZ",
    207.65: "Gs3_HZ",
    233.08: "As3_HZ",
    246.94: "B3_HZ",
    261.63: "C4_HZ",
    277.18: "Cs4_HZ",
}

VALID_FREQS = list(NOTE_MAP.keys())

# =========================
# HELPERS
# =========================

def midi_to_freq(note):
    return 440.0 * (2 ** ((note - 69) / 12.0))

def snap_freq(freq):
    return min(VALID_FREQS, key=lambda x: abs(x - freq))

# =========================
# PARSE MIDI
# =========================

def parse_midi(file, target_bpm=None):
    mid = MidiFile(file)

    tempo = 500000
    tpb = mid.ticks_per_beat

    time = 0
    active = {}
    notes = []

    original_bpm = 120

    for msg in mid.tracks[0]:
        time += msg.time

        if msg.type == 'set_tempo':
            tempo = msg.tempo
            original_bpm = 60000000 / tempo
            print(f"Detected MIDI BPM: {original_bpm:.2f}")

        if msg.type == 'note_on' and msg.velocity > 0:
            active[msg.note] = time

        elif msg.type in ['note_off', 'note_on']:
            if msg.note in active:
                start = active[msg.note]
                dur = time - start

                if target_bpm is None:
                    bpm_scale = 1.0
                else:
                    bpm_scale = original_bpm / target_bpm

                start_ms = int(mido.tick2second(start, tpb, tempo) * 1000 * bpm_scale)
                dur_ms   = int(mido.tick2second(dur, tpb, tempo) * 1000 * bpm_scale)

                freq = snap_freq(midi_to_freq(msg.note))

                notes.append({
                    "start": start_ms,
                    "duration": dur_ms,
                    "freq": freq
                })

                del active[msg.note]

    return notes

# =========================
# GROUP INTO CHORDS
# =========================

def build_song(notes):
    grouped = {}

    for n in notes:
        grouped.setdefault(n["start"], []).append(n)

    song = []

    for start in sorted(grouped.keys()):
        ns = grouped[start][:2]

        if len(ns) == 1:
            n = ns[0]
            song.append(("NOTE", n["freq"], n["duration"]))

        else:
            n1, n2 = ns
            duration = max(n1["duration"], n2["duration"])
            song.append(("CHORD", n1["freq"], n2["freq"], duration))

    return song

# =========================
# WRITE TO song_data.c
# =========================

def write_c_file(song, filename="song_data.c"):
    with open(filename, "w") as f:

        f.write('#include "song_scheduler.h"\n\n')

        f.write("static const SongNote_t song2_notes[] = {\n\n")

        for s in song:
            if s[0] == "NOTE":
                freq_name = NOTE_MAP[s[1]]
                f.write(f"    NOTE(1u, {freq_name}, {s[2]}u, 35u),\n")

            else:
                f1 = NOTE_MAP[s[1]]
                f2 = NOTE_MAP[s[2]]
                f.write(f"    CHORD({f1}, {f2}, {s[3]}u, 250u),\n")

        f.write("\n};\n")

    print(f"\n Generated {filename}")

# =========================
# MAIN
# =========================

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python midi_to_c.py <file.mid> [BPM]")
        sys.exit(1)

    midi_file = sys.argv[1]

    if not midi_file.endswith(".mid"):
        print("❌ Error: First argument must be a .mid file")
        sys.exit(1)

    target_bpm = int(sys.argv[2]) if len(sys.argv) > 2 else None

    print(f"\nProcessing: {midi_file}")

    if target_bpm is None:
        print("Using original MIDI BPM\n")
    else:
        print(f"Target BPM: {target_bpm}\n")

    notes = parse_midi(midi_file, target_bpm)
    song = build_song(notes)

    print(f"Generated {len(song)} notes/chords")

    write_c_file(song)