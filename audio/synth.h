#ifndef BATHAT_SYNTH_H
#define BATHAT_SYNTH_H

#include <cstdint>

// Continuous directional hum synthesizer. Each tracked object drives one
// voice: a warm hum (110 Hz sine + one octave-up partial at -9 dB) whose
// loudness follows closeness, whose amplitude-modulation "pulse" speeds up as
// the object nears, and whose constant-power stereo pan follows the object's
// azimuth. Every control parameter is slewed per sample, so the ~4 fps
// detection updates glide instead of clicking.
//
// MiDaS depth is relative, so loudness is too: the nearest object (rank 0)
// carries the mix and every later rank sits 20 dB down — the "closest thing
// hums, background is silent" contract.
//
// Pure DSP over plain buffers (no I/O, no ring, no QNX), host-testable.
namespace synth {

constexpr int kSampleRate = 48000;
constexpr int kMaxVoices = 3;

// Pulse (tremolo) rate for a closeness in [0,1]: 0.8 Hz far -> 10 Hz close,
// riding a steep power curve so the acceleration lands near the end of the
// approach.
float pulse_hz(float closeness);

// Linear gain (master excluded) for a voice: closeness maps -48..0 dB along
// the same steep curve (distant objects barely murmur, the final approach
// swells hard), ranks behind the nearest lose 20 dB, and anything below the
// closeness floor or beyond rank 2 is silent.
float voice_gain(float closeness, int rank);

// Constant-power pan across the 180-degree forward arc; azimuth is clamped
// to [-90, +90] degrees (0 = ahead, positive = right).
void pan_gains(float azimuth_deg, float* left, float* right);

// Control-rate target for one voice, ranked nearest-first by the caller.
struct VoiceTarget {
    float closeness;    // [0,1], 1 = nearest
    float azimuth_deg;  // world azimuth in degrees
    bool active;        // false fades the voice out
};

class Engine {
public:
    explicit Engine(float master = 0.30f);

    // Replace the voice targets (index = rank). Fewer than kMaxVoices targets
    // fades the remaining voices out.
    void set_targets(const VoiceTarget* targets, int count);

    // Render interleaved stereo S16 into `out` (2 * frames samples).
    void render(int16_t* out, int frames);

private:
    struct Voice {
        float amp = 0.0f;           // slewed gain (attack/release asymmetric)
        float pan = 0.0f;           // slewed pan position in [-1, 1]
        float pulse = 1.5f;         // slewed AM rate, Hz
        float amp_target = 0.0f;
        float pan_target = 0.0f;
        float pulse_target = 1.5f;
        double carrier_phase = 0.0;  // radians, phase-continuous
        double am_phase = 0.0;
    };

    float master_;
    Voice voices_[kMaxVoices];
};

}  // namespace synth

#endif  // BATHAT_SYNTH_H
