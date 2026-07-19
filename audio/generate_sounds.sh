#!/bin/bash

set -euo pipefail

if [[ -z "${ELEVENLABS_API_KEY:-}" ]]; then
    echo "Error: ELEVENLABS_API_KEY is not set."
    exit 1
fi

if [[ -z "${ELEVENLABS_VOICE_ID:-}" ]]; then
    echo "Error: ELEVENLABS_VOICE_ID is not set."
    exit 1
fi

OUTPUT_DIR="sounds"

mkdir -p "$OUTPUT_DIR"

OBJECTS=(
    "person"
    "chair"
    "table"
    "door"
    "bench"
    "car"
    "bicycle"
    "backpack"
    "bottle"
    "obstacle"
)

for object in "${OBJECTS[@]}"; do
    echo "Generating ${object}.mp3..."

    capitalized_object="$(printf '%s' "$object" | awk '{
        first = toupper(substr($0, 1, 1))
        rest = substr($0, 2)
        print first rest
    }')"

    request_body="$(printf \
        '{"text":"%s.","model_id":"eleven_flash_v2_5"}' \
        "$capitalized_object"
    )"

    curl \
        --fail \
        --silent \
        --show-error \
        -X POST \
        "https://api.elevenlabs.io/v1/text-to-speech/${ELEVENLABS_VOICE_ID}?output_format=mp3_44100_128" \
        -H "xi-api-key: ${ELEVENLABS_API_KEY}" \
        -H "Content-Type: application/json" \
        -d "$request_body" \
        --output "${OUTPUT_DIR}/${object}.mp3"

    echo "Created ${OUTPUT_DIR}/${object}.mp3"
done

echo "Finished generating object sounds."