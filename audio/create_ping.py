import math
import struct
import wave

SAMPLE_RATE = 48_000
DURATION_SECONDS = 0.08
FREQUENCY_HZ = 850.0
MAX_AMPLITUDE = 0.22

output_path = "sounds/ping.wav"

frame_count = int(
    SAMPLE_RATE * DURATION_SECONDS
)

with wave.open(output_path, "w") as wav_file:
    wav_file.setnchannels(1)
    wav_file.setsampwidth(2)
    wav_file.setframerate(SAMPLE_RATE)

    for frame in range(frame_count):
        time_seconds = frame / SAMPLE_RATE

        # Quickly fade the sound out to avoid a harsh click.
        envelope = 1.0 - (
            frame / frame_count
        )

        sample = (
            MAX_AMPLITUDE
            * envelope
            * math.sin(
                2.0
                * math.pi
                * FREQUENCY_HZ
                * time_seconds
            )
        )

        integer_sample = int(
            sample * 32767
        )

        wav_file.writeframes(
            struct.pack(
                "<h",
                integer_sample
            )
        )

print(
    f"Created {output_path}"
)