import sys
from pathlib import Path


def midi_to_c(input_file, output_file):
    with open(input_file, "rb") as f:
        data = f.read()

    with open(output_file, "w") as f:

        f.write("#include <stdint.h>\n\n")

        f.write("const uint8_t g_midi_data[] = {\n")

        for i, byte in enumerate(data):

            if i % 12 == 0:
                f.write("    ")

            f.write(f"0x{byte:02X},")

            if i % 12 == 11:
                f.write("\n")

        f.write("\n};\n\n")

        f.write("const uint32_t g_midi_len = sizeof(g_midi_data);\n")

    print("Generated:", output_file)
    print("MIDI size:", len(data), "bytes")


if __name__ == "__main__":

    if len(sys.argv) != 3:
        print("Usage:")
        print("python midi_to_c.py input.mid output.c")
        sys.exit(1)

    midi_to_c(sys.argv[1], sys.argv[2])