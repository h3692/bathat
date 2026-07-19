#include "speech.h"

#include <dirent.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "synth.h"

namespace {

constexpr float kDuck = 0.4f;        // hum gain while a clip plays
constexpr float kSpeechGain = 0.8f;  // clip gain into the bus
const float kDuckK =                 // ~30 ms duck ramp, no steps/clicks
    1.0f - std::exp(-1.0f / (0.03f * synth::kSampleRate));

uint32_t get_u32(const uint8_t* p) {
    return p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

uint16_t get_u16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

int16_t clamp_s16(float v) {
    return static_cast<int16_t>(std::lrintf(std::min(32767.0f, std::max(-32768.0f, v))));
}

}  // namespace

std::vector<int16_t> load_wav_mono48(const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return {};
    std::fseek(f, 0, SEEK_END);
    const long size = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    std::vector<uint8_t> bytes(size > 0 ? static_cast<size_t>(size) : 0);
    const bool read_ok =
        !bytes.empty() && std::fread(bytes.data(), 1, bytes.size(), f) == bytes.size();
    std::fclose(f);

    const char* why = nullptr;
    std::vector<int16_t> samples;
    if (!read_ok || bytes.size() < 44 || std::memcmp(bytes.data(), "RIFF", 4) != 0 ||
        std::memcmp(bytes.data() + 8, "WAVE", 4) != 0) {
        why = "not a RIFF/WAVE file";
    } else {
        // Walk the chunks: the fmt chunk must say PCM16 mono 48 kHz, then the
        // data chunk holds the samples. Extra chunks (LIST etc.) are skipped.
        bool fmt_ok = false;
        size_t off = 12;
        while (off + 8 <= bytes.size()) {
            const uint32_t chunk_size = get_u32(bytes.data() + off + 4);
            const uint8_t* body = bytes.data() + off + 8;
            if (off + 8 + chunk_size > bytes.size()) break;
            if (std::memcmp(bytes.data() + off, "fmt ", 4) == 0 && chunk_size >= 16) {
                fmt_ok = get_u16(body) == 1 && get_u16(body + 2) == 1 &&
                         get_u32(body + 4) == static_cast<uint32_t>(synth::kSampleRate) &&
                         get_u16(body + 14) == 16;
                if (!fmt_ok) {
                    why = "need PCM16 mono 48 kHz";
                    break;
                }
            } else if (std::memcmp(bytes.data() + off, "data", 4) == 0) {
                if (!fmt_ok) {
                    why = "data before fmt";
                    break;
                }
                samples.resize(chunk_size / 2);
                std::memcpy(samples.data(), body, samples.size() * 2);
                break;
            }
            off += 8 + chunk_size + (chunk_size & 1u);
        }
        if (!why && samples.empty()) why = "no data chunk";
    }
    if (why) {
        std::fprintf(stderr, "speech: skipping %s (%s)\n", path.c_str(), why);
        return {};
    }
    return samples;
}

bool SpeechPlayer::load(const std::string& dir) {
    DIR* d = opendir(dir.c_str());
    if (!d) return false;
    while (struct dirent* e = readdir(d)) {
        const std::string name = e->d_name;
        if (name.size() < 5 || name.compare(name.size() - 4, 4, ".wav") != 0)
            continue;
        std::vector<int16_t> clip = load_wav_mono48(dir + "/" + name);
        if (!clip.empty())
            clips_[name.substr(0, name.size() - 4)] = std::move(clip);
    }
    closedir(d);
    return !clips_.empty();
}

void SpeechPlayer::announce(const std::string& key, float pan) {
    if (clips_.count(key) == 0) return;
    pending_ = true;
    pending_key_ = key;
    pending_pan_ = pan;
}

void SpeechPlayer::mix(int16_t* block, int frames) {
    for (int f = 0; f < frames; ++f) {
        if (!playing_ && pending_) {
            cur_ = &clips_[pending_key_];
            pos_ = 0;
            synth::pan_gains(pending_pan_ * 90.0f, &pan_l_, &pan_r_);
            playing_ = true;
            pending_ = false;
        }
        duck_ += ((playing_ ? kDuck : 1.0f) - duck_) * kDuckK;
        if (!playing_ && duck_ > 0.999f) break;  // idle, fully released
        float l = block[2 * f] * duck_;
        float r = block[2 * f + 1] * duck_;
        if (playing_) {
            const float s = (*cur_)[pos_] * kSpeechGain;
            l += s * pan_l_;
            r += s * pan_r_;
            if (++pos_ >= cur_->size()) playing_ = false;
        }
        block[2 * f] = clamp_s16(l);
        block[2 * f + 1] = clamp_s16(r);
    }
}
