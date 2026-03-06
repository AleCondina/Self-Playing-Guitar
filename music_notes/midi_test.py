from mido import MidiFile
import os
import math

def midi_to_freq(note):
    return 440.0 * (2 ** ((note - 69) / 12))

note_names = ['C', 'C#', 'D', 'D#', 'E', 'F',
              'F#', 'G', 'G#', 'A', 'A#', 'B']

def note_name(note):
    name = note_names[note % 12]
    octave = (note // 12) - 1
    return f"{name}{octave}"



base_dir = os.path.dirname(__file__)
midi_path = os.path.join(base_dir, "110bpm_Smoke_on_the_Water.mid")

mid = MidiFile(midi_path)

current_time = 0
active_notes = {}

print("SOURCE MIDI EVENTS:\n")

for msg in mid:
    current_time += msg.time * 1000

    if msg.type == 'note_on' and msg.velocity > 0:
        active_notes[msg.note] = current_time

    elif msg.type == 'note_off' or (msg.type == 'note_on' and msg.velocity == 0):
        if msg.note in active_notes:
            start = active_notes[msg.note]
            duration = current_time - start
            freq = midi_to_freq(msg.note)

            print(
                f"{note_name(msg.note):4} | "
                f"start={int(start):6} ms | "
                f"dur={int(duration):4} ms | "
                f"freq={freq:.2f} Hz | "
                f"midi={msg.note}"
            )

            del active_notes[msg.note]