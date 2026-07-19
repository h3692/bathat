#include "synth.h"

#include <algorithm>
#include <cmath>

namespace synth {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kCarrierHz = 110.0f;
constexpr float kPartialGain = 0.35481f;   // -9 dB, one octave up
constexpr float kAmDepth = 0.85f;          // pulse swings amplitude 0.15..1.0
constexpr float kClosenessFloor = 0.15f;   // below this an object is "background"
constexpr float kRankPenalty = 0.1f;       // -20 dB for every rank behind the nearest

// Perceptual steepening: both the pulse rate and the level ride closeness^2.5,
// so a distant object barely murmurs (slow pulse, deep in the -48 dB range)
// and the last stretch of approach escalates hard.
constexpr float kResponseGamma = 3.5f;
constexpr float kLevelRangeDb = 48.0f;
constexpr float kPulseMinHz = 0.8f;
constexpr float kPulseMaxHz = 10.0f;

// One-pole coefficient for a time constant in seconds at the sample rate.
float slew_k(float tau_s) {
    return 1.0f - std::exp(-1.0f / (tau_s * kSampleRate));
}

const float kAttackK = slew_k(0.08f);   // gain rises (new voice / approach)
const float kReleaseK = slew_k(0.20f);  // gain falls (lost voice / retreat)
// Slow pan on purpose: when the trackers' azimuth jumps (overlap handoffs,
// camera disagreement) the hum glides through the middle of the stereo image
// instead of snapping ear to ear.
const float kPanK = slew_k(0.25f);
const float kPulseK = slew_k(0.15f);

}  // namespace

float pulse_hz(float closeness) {
    const float c = std::min(std::max(closeness, 0.0f), 1.0f);
    return kPulseMinHz + (kPulseMaxHz - kPulseMinHz) * std::pow(c, kResponseGamma);
}

float voice_gain(float closeness, int rank) {
    if (rank < 0 || rank >= kMaxVoices || closeness < kClosenessFloor) return 0.0f;
    const float c = std::min(closeness, 1.0f);
    const float level_db = -kLevelRangeDb * (1.0f - std::pow(c, kResponseGamma));
    const float gain = std::pow(10.0f, level_db / 20.0f);
    return rank == 0 ? gain : gain * kRankPenalty;
}

void pan_gains(float azimuth_deg, float* left, float* right) {
    const float pan = std::min(std::max(azimuth_deg / 90.0f, -1.0f), 1.0f);
    const float theta = (pan + 1.0f) * kPi / 4.0f;
    *left = std::cos(theta);
    *right = std::sin(theta);
}

Engine::Engine(float master) : master_(master) {}

void Engine::set_targets(const VoiceTarget* targets, int count) {
    for (int i = 0; i < kMaxVoices; ++i) {
        Voice& v = voices_[i];
        if (i < count && targets[i].active) {
            v.amp_target = voice_gain(targets[i].closeness, i);
            v.pan_target = std::min(std::max(targets[i].azimuth_deg / 90.0f, -1.0f), 1.0f);
            v.pulse_target = pulse_hz(targets[i].closeness);
        } else {
            v.amp_target = 0.0f;  // pan/pulse hold their values through the fade
        }
    }
}

void Engine::render(int16_t* out, int frames) {
    const double carrier_step = 2.0 * kPi * kCarrierHz / kSampleRate;
    for (int f = 0; f < frames; ++f) {
        float bus_l = 0.0f, bus_r = 0.0f;
        for (Voice& v : voices_) {
            v.amp += (v.amp_target - v.amp) *
                     (v.amp_target > v.amp ? kAttackK : kReleaseK);
            v.pan += (v.pan_target - v.pan) * kPanK;
            v.pulse += (v.pulse_target - v.pulse) * kPulseK;
            if (v.amp < 1e-5f && v.amp_target == 0.0f) continue;

            // Raised-cosine tremolo: smooth from 1-depth up to 1, no clicks.
            const float am = (1.0f - kAmDepth) +
                             kAmDepth * 0.5f * (1.0f - std::cos(static_cast<float>(v.am_phase)));
            const float hum = std::sin(static_cast<float>(v.carrier_phase)) +
                              kPartialGain * std::sin(static_cast<float>(2.0 * v.carrier_phase));
            const float s = master_ * v.amp * am * hum;

            const float theta = (v.pan + 1.0f) * kPi / 4.0f;
            bus_l += s * std::cos(theta);
            bus_r += s * std::sin(theta);

            v.carrier_phase += carrier_step;
            if (v.carrier_phase > 2.0 * kPi) v.carrier_phase -= 2.0 * kPi;
            v.am_phase += 2.0 * kPi * v.pulse / kSampleRate;
            if (v.am_phase > 2.0 * kPi) v.am_phase -= 2.0 * kPi;
        }
        bus_l = std::min(std::max(bus_l, -1.0f), 1.0f);
        bus_r = std::min(std::max(bus_r, -1.0f), 1.0f);
        out[2 * f] = static_cast<int16_t>(std::lrintf(bus_l * 32767.0f));
        out[2 * f + 1] = static_cast<int16_t>(std::lrintf(bus_r * 32767.0f));
    }
}

}  // namespace synth
