#ifndef BATHAT_SPEECH_H
#define BATHAT_SPEECH_H

#include <cstdint>
#include <map>
#include <string>
#include <vector>

// Spoken obstacle notifications, mixed into the same stereo stream as the hum.
//
// Clips are pre-generated 48 kHz mono PCM16 WAVs (audio/generate_notify.py ->
// audio/sounds/notify/), keyed by filename stem ("ahead", "45_left", ...).
// While a clip plays, the already-rendered hum is ducked (with short ramps,
// never a step) so the words stay intelligible without interrupting the hum;
// the clip itself is constant-power panned toward the obstacle's ear zone.
// One clip at a time; an announcement arriving mid-clip queues (depth 1).
//
// Pure DSP over the block buffer — no I/O in mix() — so it is host-testable.

// Parse a PCM16 mono 48 kHz WAV; anything else (or unreadable) returns empty
// with a warning on stderr.
std::vector<int16_t> load_wav_mono48(const std::string& path);

class SpeechPlayer {
public:
    // Load every usable .wav in `dir`; returns false if none loaded (the
    // caller disables notifications and everything else runs unchanged).
    bool load(const std::string& dir);

    bool has(const std::string& key) const { return clips_.count(key) != 0; }

    // Queue the clip for `key`, panned to `pan` in [-1 left .. +1 right].
    // Unknown keys are ignored. A clip already playing finishes first.
    void announce(const std::string& key, float pan);

    // True while a clip is playing or pending.
    bool speaking() const { return playing_ || pending_; }

    // Mix into an already-rendered interleaved stereo block, ducking it while
    // speech is active. Call right after synth::Engine::render each block.
    void mix(int16_t* block, int frames);

private:
    std::map<std::string, std::vector<int16_t>> clips_;

    const std::vector<int16_t>* cur_ = nullptr;
    size_t pos_ = 0;
    float pan_l_ = 0.707f, pan_r_ = 0.707f;
    bool playing_ = false;

    bool pending_ = false;
    std::string pending_key_;
    float pending_pan_ = 0.0f;

    float duck_ = 1.0f;  // slewed hum gain: 1 idle, ~0.4 while speaking
};

#endif  // BATHAT_SPEECH_H
