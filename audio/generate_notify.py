#!/usr/bin/env python3
"""Generate the spoken obstacle-notification clips with the ElevenLabs API.

One-time, dev-machine script (the Pi never talks to the API): produces the 13
clips bat_audio's announcer plays — "Obstacle ahead." plus "Obstacle, N
degrees left/right" for N in 15..90 — as 48 kHz mono PCM16 WAVs in
audio/sounds/notify/, named by the announcer's bucket keys (ahead.wav,
15_left.wav, ... 90_right.wav).

Requires (never stored in the repo — see audio/generate_sounds.sh):
    ELEVENLABS_API_KEY    your API key
    ELEVENLABS_VOICE_ID   the voice to use (same one as the object sounds)

    python3 audio/generate_notify.py [--force]

Existing clips are skipped unless --force (no accidental re-billing).
Stdlib-only.
"""

import argparse
import array
import os
import sys
import urllib.request
import wave

MODEL_ID = "eleven_flash_v2_5"  # same as generate_sounds.sh
API_RATE = 24000                # pcm_24000: raw S16LE mono
OUT_RATE = 48000                # the pipeline's audio rate


def phrases():
    yield "ahead", "Obstacle ahead."
    for degrees in (15, 30, 45, 60, 75, 90):
        for side in ("left", "right"):
            yield "%d_%s" % (degrees, side), \
                  "Obstacle, %d degrees %s." % (degrees, side)


def tts_pcm(text, api_key, voice_id):
    """Return raw S16LE mono PCM at API_RATE for `text`."""
    url = ("https://api.elevenlabs.io/v1/text-to-speech/%s?output_format=pcm_%d"
           % (voice_id, API_RATE))
    body = ('{"text": %s, "model_id": "%s"}'
            % (json_string(text), MODEL_ID)).encode()
    request = urllib.request.Request(
        url, data=body,
        headers={"xi-api-key": api_key, "Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=60) as response:
        return response.read()


def json_string(text):
    return '"%s"' % text.replace("\\", "\\\\").replace('"', '\\"')


def upsample_2x(pcm_bytes):
    """24 kHz -> 48 kHz by linear midpoint interpolation."""
    src = array.array("h")
    src.frombytes(pcm_bytes[:len(pcm_bytes) & ~1])
    if sys.byteorder == "big":
        src.byteswap()
    out = array.array("h", bytes(4 * len(src)))
    for i, s in enumerate(src):
        nxt = src[i + 1] if i + 1 < len(src) else s
        out[2 * i] = s
        out[2 * i + 1] = (s + nxt) // 2
    if sys.byteorder == "big":
        out.byteswap()
    return out.tobytes()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true",
                        help="regenerate clips that already exist")
    args = parser.parse_args()

    api_key = os.environ.get("ELEVENLABS_API_KEY")
    voice_id = os.environ.get("ELEVENLABS_VOICE_ID")
    if not api_key:
        sys.exit("Error: ELEVENLABS_API_KEY is not set.")
    if not voice_id:
        sys.exit("Error: ELEVENLABS_VOICE_ID is not set.")

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "sounds", "notify")
    os.makedirs(out_dir, exist_ok=True)

    for key, text in phrases():
        path = os.path.join(out_dir, key + ".wav")
        if os.path.exists(path) and not args.force:
            print("exists, skipping: %s" % path)
            continue
        print("generating %s  (%r)" % (path, text))
        pcm48 = upsample_2x(tts_pcm(text, api_key, voice_id))
        with wave.open(path, "w") as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(OUT_RATE)
            wav_file.writeframes(pcm48)

    print("done — clips in %s" % out_dir)


if __name__ == "__main__":
    main()
