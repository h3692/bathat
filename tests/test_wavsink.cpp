// Host unit test for WavFileSink: the dev-machine AudioSink that renders the
// hum to a WAV file instead of QNX audio. Verifies the RIFF header fields and
// the sample payload byte-for-byte. Build and run with
//   make -C tests
#include "wavfilesink.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <vector>

static int failures = 0;

#define CHECK(cond, msg)                                                        \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::printf("FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__);       \
            ++failures;                                                         \
        }                                                                       \
    } while (0)

static uint32_t u32(const uint8_t* p) {
    return p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

static uint16_t u16(const uint8_t* p) { return p[0] | (p[1] << 8); }

int main(int argc, char** argv) {
    if (argc != 2) {
        std::printf("usage: %s <wav-file>\n", argv[0]);
        return 2;
    }
    const char* path = argv[1];
    unlink(path);

    // Two writes of interleaved stereo frames; sizes must accumulate.
    const int16_t frames1[] = {100, -100, 2000, -2000};       // 2 frames
    const int16_t frames2[] = {32767, -32768};                // 1 frame
    {
        WavFileSink sink(path);
        CHECK(sink.start(), "sink opens the file");
        CHECK(sink.write(frames1, 2), "first write succeeds");
        CHECK(sink.write(frames2, 1), "second write succeeds");
        sink.stop();
    }

    FILE* f = std::fopen(path, "rb");
    CHECK(f != nullptr, "wav file exists");
    if (!f) return 1;
    uint8_t hdr[44];
    CHECK(std::fread(hdr, 1, sizeof(hdr), f) == sizeof(hdr), "44-byte header");

    const uint32_t data_bytes = 3 * 2 * 2;  // 3 frames, stereo, S16
    CHECK(std::memcmp(hdr, "RIFF", 4) == 0, "RIFF tag");
    CHECK(u32(hdr + 4) == 36 + data_bytes, "RIFF size covers the data");
    CHECK(std::memcmp(hdr + 8, "WAVE", 4) == 0, "WAVE tag");
    CHECK(std::memcmp(hdr + 12, "fmt ", 4) == 0, "fmt chunk tag");
    CHECK(u32(hdr + 16) == 16, "fmt chunk size");
    CHECK(u16(hdr + 20) == 1, "PCM format");
    CHECK(u16(hdr + 22) == 2, "stereo");
    CHECK(u32(hdr + 24) == 48000, "48 kHz");
    CHECK(u32(hdr + 28) == 48000 * 4, "byte rate");
    CHECK(u16(hdr + 32) == 4, "block align");
    CHECK(u16(hdr + 34) == 16, "16-bit samples");
    CHECK(std::memcmp(hdr + 36, "data", 4) == 0, "data chunk tag");
    CHECK(u32(hdr + 40) == data_bytes, "data chunk size");

    std::vector<uint8_t> data(data_bytes);
    CHECK(std::fread(data.data(), 1, data.size(), f) == data.size(), "payload present");
    CHECK(std::fgetc(f) == EOF, "nothing after the payload");
    const int16_t want[] = {100, -100, 2000, -2000, 32767, -32768};
    CHECK(std::memcmp(data.data(), want, sizeof(want)) == 0,
          "samples stored little-endian in order");
    std::fclose(f);
    unlink(path);

    if (failures == 0) {
        std::printf("PASS: wavsink (all checks)\n");
        return 0;
    }
    std::printf("%d check(s) FAILED\n", failures);
    return 1;
}
