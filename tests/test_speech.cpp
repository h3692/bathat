// Host unit test for the spoken-notification player: WAV loading, hum ducking
// while a clip plays, panning, queueing, and clean recovery afterwards.
// Build and run with
//   make -C tests
#include "speech.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <sys/stat.h>
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

namespace {

void put_u32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}

void put_u16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}

// Write a minimal PCM16 WAV of `n` constant-valued samples.
void write_wav(const std::string& path, uint16_t channels, uint32_t rate,
               int n, int16_t value) {
    const uint32_t data_bytes = static_cast<uint32_t>(n) * channels * 2;
    uint8_t hdr[44];
    std::memcpy(hdr, "RIFF", 4);
    put_u32(hdr + 4, 36 + data_bytes);
    std::memcpy(hdr + 8, "WAVE", 4);
    std::memcpy(hdr + 12, "fmt ", 4);
    put_u32(hdr + 16, 16);
    put_u16(hdr + 20, 1);
    put_u16(hdr + 22, channels);
    put_u32(hdr + 24, rate);
    put_u32(hdr + 28, rate * channels * 2);
    put_u16(hdr + 32, static_cast<uint16_t>(channels * 2));
    put_u16(hdr + 34, 16);
    std::memcpy(hdr + 36, "data", 4);
    put_u32(hdr + 40, data_bytes);
    FILE* f = std::fopen(path.c_str(), "wb");
    std::fwrite(hdr, 1, sizeof(hdr), f);
    std::vector<int16_t> samples(static_cast<size_t>(n) * channels, value);
    std::fwrite(samples.data(), 2, samples.size(), f);
    std::fclose(f);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::printf("usage: %s <scratch-dir>\n", argv[0]);
        return 2;
    }
    const std::string dir = argv[1];
    mkdir(dir.c_str(), 0777);
    write_wav(dir + "/ahead.wav", 1, 48000, 4800, 8000);   // 100 ms, mono 48k
    write_wav(dir + "/45_left.wav", 1, 48000, 2400, 8000); // 50 ms
    write_wav(dir + "/stereo.wav", 2, 48000, 100, 8000);   // must be rejected
    write_wav(dir + "/slow.wav", 1, 24000, 100, 8000);     // must be rejected

    // Loader: good clip parses; wrong channel count / rate are rejected.
    CHECK(load_wav_mono48(dir + "/ahead.wav").size() == 4800, "mono 48k loads");
    CHECK(load_wav_mono48(dir + "/stereo.wav").empty(), "stereo rejected");
    CHECK(load_wav_mono48(dir + "/slow.wav").empty(), "wrong rate rejected");
    CHECK(load_wav_mono48(dir + "/missing.wav").empty(), "missing file rejected");

    SpeechPlayer player;
    CHECK(player.load(dir), "player loads the clip directory");
    CHECK(player.has("ahead") && player.has("45_left"), "clips keyed by stem");
    CHECK(!player.has("stereo") && !player.has("nope"), "bad clips not loaded");

    // Idle mix leaves the hum untouched.
    const int N = 24000;  // 500 ms
    std::vector<int16_t> block(static_cast<size_t>(N) * 2, 10000);
    player.mix(block.data(), N);
    CHECK(block[0] == 10000 && block[2 * N - 1] == 10000, "idle mix is a no-op");

    // A hard-left announcement: the hum ducks in both ears and the speech
    // rides on the left only. At a late-but-still-speaking sample the
    // left-right difference is exactly the clip contribution.
    player.announce("ahead", -1.0f);
    CHECK(player.speaking(), "speaking after announce");
    std::fill(block.begin(), block.end(), static_cast<int16_t>(10000));
    player.mix(block.data(), N);
    const int probe = 4500;  // ~94 ms in: duck settled, clip still playing
    const int l = block[2 * probe], r = block[2 * probe + 1];
    CHECK(r > 3500 && r < 5200, "hum ducked to ~0.4 in the speech-free ear");
    CHECK(l - r > 6200 && l - r < 6600, "speech (0.8 gain) rides the left ear");
    // Long after the clip (100 ms) ends, the duck has released fully.
    CHECK(block[2 * (N - 1)] >= 9800 && block[2 * (N - 1) + 1] >= 9800,
          "hum restores after the clip ends");
    CHECK(!player.speaking(), "not speaking once the clip is done");

    // A second announce during playback queues (depth 1) instead of cutting.
    player.announce("ahead", 0.0f);
    std::vector<int16_t> chunk(1024 * 2, 0);
    player.mix(chunk.data(), 1024);  // starts the first clip
    player.announce("45_left", -1.0f);
    CHECK(player.speaking(), "second clip pends during the first");
    int frames_speaking = 1024;
    while (player.speaking() && frames_speaking < 48000) {
        std::fill(chunk.begin(), chunk.end(), 0);
        player.mix(chunk.data(), 1024);
        frames_speaking += 1024;
    }
    CHECK(frames_speaking >= 4800 + 2400, "both clips played back to back");
    CHECK(frames_speaking < 48000, "playback terminates");

    if (failures == 0) {
        std::printf("PASS: speech (all checks)\n");
        return 0;
    }
    std::printf("%d check(s) FAILED\n", failures);
    return 1;
}
