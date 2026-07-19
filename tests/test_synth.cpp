// Host unit test for the hum synthesizer: control laws (pulse rate, gain,
// constant-power pan) and the rendered stereo behavior (panning, rank
// attenuation, attack/release, silence floor). Build and run with
//   make -C tests
#include "synth.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

static int failures = 0;

#define CHECK(cond, msg)                                                        \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::printf("FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__);       \
            ++failures;                                                         \
        }                                                                       \
    } while (0)

static bool close_to(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

// RMS of one channel (0 = left, 1 = right) over the last quarter of a render.
static double tail_rms(const std::vector<int16_t>& buf, int channel) {
    const size_t frames = buf.size() / 2;
    const size_t start = frames - frames / 4;
    double acc = 0.0;
    for (size_t i = start; i < frames; ++i) {
        const double v = buf[2 * i + channel];
        acc += v * v;
    }
    return std::sqrt(acc / static_cast<double>(frames - start));
}

static std::vector<int16_t> render_seconds(synth::Engine& engine, double seconds) {
    const int frames = static_cast<int>(seconds * synth::kSampleRate);
    std::vector<int16_t> buf(static_cast<size_t>(frames) * 2);
    engine.render(buf.data(), frames);
    return buf;
}

int main() {
    // Pulse rate: 0.8 Hz far, 10 Hz danger-close, rising along a steep power
    // curve (gamma 2.5) so most of the acceleration happens near the end.
    CHECK(close_to(synth::pulse_hz(0.0f), 0.8f, 1e-6f), "pulse far end");
    CHECK(close_to(synth::pulse_hz(1.0f), 10.0f, 1e-5f), "pulse near end");
    const float mid_pulse = 0.8f + 9.2f * std::pow(0.5f, 2.5f);
    CHECK(close_to(synth::pulse_hz(0.5f), mid_pulse, 1e-5f),
          "pulse midpoint sits low on the curve");
    CHECK(synth::pulse_hz(0.9f) - synth::pulse_hz(0.8f) >
              synth::pulse_hz(0.5f) - synth::pulse_hz(0.4f),
          "pulse accelerates as the object nears");

    // Gain law: closeness maps -48..0 dB along the same steep curve; ranks
    // behind the nearest lose 20 dB; below the floor or beyond rank 2 silent.
    CHECK(close_to(synth::voice_gain(1.0f, 0), 1.0f, 1e-4f), "nearest at 0 dB");
    CHECK(close_to(synth::voice_gain(1.0f, 1), 0.1f, 1e-4f), "rank 1 at -20 dB");
    CHECK(close_to(synth::voice_gain(1.0f, 2), 0.1f, 1e-4f), "rank 2 at -20 dB");
    CHECK(synth::voice_gain(1.0f, 3) == 0.0f, "rank 3+ silent");
    const float half_db = -48.0f * (1.0f - std::pow(0.5f, 2.5f));
    CHECK(close_to(synth::voice_gain(0.5f, 0), std::pow(10.0f, half_db / 20.0f), 1e-4f),
          "half closeness sits deep on the curve (~-40 dB)");
    CHECK(synth::voice_gain(0.9f, 0) / synth::voice_gain(0.8f, 0) >
              synth::voice_gain(0.5f, 0) / synth::voice_gain(0.4f, 0),
          "loudness ratio steepens as the object nears");
    CHECK(synth::voice_gain(0.1f, 0) == 0.0f, "below the closeness floor = silent");

    // Constant-power pan across the 180-degree arc.
    float l, r;
    synth::pan_gains(0.0f, &l, &r);
    CHECK(close_to(l, r, 1e-6f) && close_to(l, 0.70710678f, 1e-4f), "centre is -3 dB");
    synth::pan_gains(-90.0f, &l, &r);
    CHECK(close_to(l, 1.0f, 1e-4f) && close_to(r, 0.0f, 1e-4f), "hard left");
    synth::pan_gains(90.0f, &l, &r);
    CHECK(close_to(l, 0.0f, 1e-4f) && close_to(r, 1.0f, 1e-4f), "hard right");
    synth::pan_gains(-150.0f, &l, &r);
    CHECK(close_to(l, 1.0f, 1e-4f), "beyond the arc clamps");
    for (float az = -90.0f; az <= 90.0f; az += 15.0f) {
        synth::pan_gains(az, &l, &r);
        CHECK(close_to(l * l + r * r, 1.0f, 1e-4f), "constant power everywhere");
    }

    // A single hard-left voice: loud on the left, near-nothing on the right.
    {
        synth::Engine engine;
        synth::VoiceTarget t[] = {{1.0f, -90.0f, true}};
        engine.set_targets(t, 1);
        const std::vector<int16_t> buf = render_seconds(engine, 1.0);
        const double left = tail_rms(buf, 0), right = tail_rms(buf, 1);
        CHECK(left > 1000.0, "hard-left voice is audible on the left");
        CHECK(right < left / 50.0, "hard-left voice is silent on the right");
    }

    // Rank attenuation in the mix: two equally-close voices, hard left and
    // hard right; the rank-1 channel sits 20 dB below the rank-0 channel.
    {
        synth::Engine engine;
        synth::VoiceTarget t[] = {{1.0f, -90.0f, true}, {1.0f, 90.0f, true}};
        engine.set_targets(t, 2);
        const std::vector<int16_t> buf = render_seconds(engine, 1.5);
        const double left = tail_rms(buf, 0), right = tail_rms(buf, 1);
        CHECK(close_to(static_cast<float>(right / left), 0.1f, 0.02f),
              "rank 1 renders 20 dB under rank 0");
    }

    // Attack: the hum fades in (successive windows get louder), no hard step.
    {
        synth::Engine engine;
        synth::VoiceTarget t[] = {{1.0f, 0.0f, true}};
        engine.set_targets(t, 1);
        const int win = synth::kSampleRate / 20;  // 50 ms
        std::vector<int16_t> a(static_cast<size_t>(win) * 2), b(a.size());
        engine.render(a.data(), win);
        engine.render(b.data(), win);
        double rms_a = 0.0, rms_b = 0.0;
        for (int i = 0; i < win; ++i) {
            rms_a += static_cast<double>(a[2 * i]) * a[2 * i];
            rms_b += static_cast<double>(b[2 * i]) * b[2 * i];
        }
        CHECK(rms_b > rms_a * 1.5, "attack ramps up over the first windows");
        CHECK(std::abs(a[0]) < 1000, "no click at voice start");
    }

    // Release: deactivating the voice fades it out to silence.
    {
        synth::Engine engine;
        synth::VoiceTarget on[] = {{1.0f, 0.0f, true}};
        engine.set_targets(on, 1);
        render_seconds(engine, 0.5);
        synth::VoiceTarget off[] = {{1.0f, 0.0f, false}};
        engine.set_targets(off, 1);
        const std::vector<int16_t> buf = render_seconds(engine, 1.5);
        CHECK(tail_rms(buf, 0) < 20.0 && tail_rms(buf, 1) < 20.0,
              "released voice decays to silence");
    }

    // Below the closeness floor nothing hums; three full voices never clip.
    {
        synth::Engine engine;
        synth::VoiceTarget t[] = {{0.1f, 0.0f, true}};
        engine.set_targets(t, 1);
        const std::vector<int16_t> buf = render_seconds(engine, 1.0);
        CHECK(tail_rms(buf, 0) < 20.0, "sub-floor closeness stays silent");
    }
    {
        synth::Engine engine;
        synth::VoiceTarget t[] = {{1.0f, 0.0f, true}, {1.0f, 0.0f, true},
                                  {1.0f, 0.0f, true}};
        engine.set_targets(t, 3);
        const std::vector<int16_t> buf = render_seconds(engine, 1.0);
        int rail = 0;
        for (int16_t v : buf) rail += (v == INT16_MAX || v == INT16_MIN);
        CHECK(rail == 0, "full mix never hits the rails");
    }

    if (failures == 0) {
        std::printf("PASS: synth (all checks)\n");
        return 0;
    }
    std::printf("%d check(s) FAILED\n", failures);
    return 1;
}
