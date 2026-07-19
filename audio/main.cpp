// bat_audio: the spatial-audio stage. Reads the depth worker's detections
// (/bat_det0, /bat_det1), fuses the two cameras into ranked object tracks,
// and renders the directional hum to the USB headphones (QNX QSA) — nearest
// object loudest and pulsing fastest, panned to its azimuth.
//
// The render loop is paced by the sink's blocking writes (~21 ms blocks), and
// the detection rings are re-sampled once per block, so control runs at
// ~47 Hz while the synth slews every parameter per sample.
//
// On the Pi:      ./bat_audio                       (after the pipeline is up)
// Dev machine:    ./bat_audio --dets ./bat_det0.ring --wav out.wav --seconds 10
//                 (feed the ring with tools/detfeed.py)
// Bring-up:       ./bat_audio --tone                (1 s test tone, then exit)

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <cmath>
#include <unistd.h>

#include "bat_ring.h"
#include "fusion.h"
#include "qsasink.h"
#include "synth.h"
#include "wavfilesink.h"

namespace {

volatile sig_atomic_t g_run = 1;
void on_signal(int) { g_run = 0; }

constexpr int kBlockFrames = 1024;  // ~21 ms at 48 kHz
// Detections count as live for as long as the depth cadence can realistically
// gap (inference on the Pi can dip below 1 fps per camera under load); going
// stale means the worker actually died, not that it is merely slow.
constexpr uint64_t kDetFreshNs = 2000000000ull;

void print_usage(const char* prog) {
    std::fprintf(stderr,
                 "usage: %s [--dets NAME...] [--wav PATH [--seconds S]] [--adev NAME]\n"
                 "          [--master F] [--tone]\n"
                 "  --dets NAME...  detection ring names or file paths\n"
                 "                  (default: bat_det0 and, when present, bat_det1)\n"
                 "  --wav PATH      render to a WAV file instead of QNX audio\n"
                 "  --seconds S     stop after S seconds (default 10 with --wav, else run)\n"
                 "  --adev NAME     ALSA PCM device name (default: \"default\")\n"
                 "  --master F      master volume 0..1 (default 0.45)\n"
                 "  --tone          play a 1 s test tone and exit (audio bring-up)\n",
                 prog);
}

// One detection ring, opened lazily like the viewfinder's views: the depth
// worker may start later, and its ring may be a shm name or a plain file.
class DetSource {
public:
    DetSource(std::string name, int camera) : name_(std::move(name)), camera_(camera) {}
    ~DetSource() {
        if (open_) bat_ring_close(&ring_);
    }

    // Newest fresh detections (empty if none, stale, or the ring is not up).
    void poll(uint64_t now_ns, std::vector<fusion::Detection>* out) {
        if (!open_ && !try_open(now_ns)) return;
        bat_det_payload p;
        bat_slot_hdr meta;
        const int rc = bat_ring_read_latest(&ring_, reinterpret_cast<uint8_t*>(&p),
                                            sizeof(p), &meta);
        if (rc == 1 && meta.size == sizeof(p) &&
            (!have_frame_ || meta.frame_idx != last_frame_idx_)) {
            last_frame_idx_ = meta.frame_idx;
            have_frame_ = true;
            t_publish_ = meta.t_publish;
            dets_.clear();
            const uint32_t count = p.count < BAT_DET_NMAX ? p.count : BAT_DET_NMAX;
            for (uint32_t i = 0; i < count; ++i)
                dets_.push_back({p.rec[i].closeness, p.rec[i].azimuth_deg, camera_});
        }
        if (have_frame_ && now_ns >= t_publish_ && now_ns - t_publish_ <= kDetFreshNs)
            out->insert(out->end(), dets_.begin(), dets_.end());
    }

private:
    bool try_open(uint64_t now_ns) {
        if (now_ns < next_open_ns_) return false;
        next_open_ns_ = now_ns + 1000000000ull;
        // A plain file path first (host rings), then the shm name (device).
        if (bat_ring_open(&ring_, name_.c_str(), /*use_shm=*/0) != 0 &&
            bat_ring_open(&ring_, name_.c_str(), /*use_shm=*/1) != 0)
            return false;
        if (ring_.hdr->format != BAT_FMT_DET ||
            ring_.hdr->slot_size < sizeof(bat_det_payload)) {
            bat_ring_close(&ring_);
            return false;
        }
        open_ = true;
        return true;
    }

    std::string name_;
    int camera_;
    bat_ring ring_{};
    bool open_ = false;
    uint64_t next_open_ns_ = 0;
    uint64_t last_frame_idx_ = ~0ull;
    bool have_frame_ = false;
    uint64_t t_publish_ = 0;
    std::vector<fusion::Detection> dets_;
};

int run_tone(AudioSink& sink) {
    std::fprintf(stderr, "tone: 440 Hz for 1 s\n");
    std::vector<int16_t> block(kBlockFrames * 2);
    double phase = 0.0;
    for (int rendered = 0; rendered < synth::kSampleRate; rendered += kBlockFrames) {
        for (int f = 0; f < kBlockFrames; ++f) {
            const int16_t s = static_cast<int16_t>(
                std::lrint(std::sin(phase) * 0.25 * 32767.0));
            block[2 * f] = s;
            block[2 * f + 1] = s;
            phase += 2.0 * 3.14159265358979323846 * 440.0 / synth::kSampleRate;
        }
        if (!sink.write(block.data(), kBlockFrames)) return 1;
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    std::vector<std::string> det_names;
    std::string wav_path, adev;
    double seconds = -1.0;
    float master = 0.45f;
    bool tone = false;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--dets") == 0) {
            while (i + 1 < argc && argv[i + 1][0] != '-')
                det_names.emplace_back(argv[++i]);
        } else if (std::strcmp(argv[i], "--wav") == 0 && i + 1 < argc) {
            wav_path = argv[++i];
        } else if (std::strcmp(argv[i], "--seconds") == 0 && i + 1 < argc) {
            seconds = std::atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--adev") == 0 && i + 1 < argc) {
            adev = argv[++i];
        } else if (std::strcmp(argv[i], "--master") == 0 && i + 1 < argc) {
            master = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--tone") == 0) {
            tone = true;
        } else {
            print_usage(argv[0]);
            return 2;
        }
    }
    if (det_names.empty()) {
        det_names.emplace_back("/bat_det0");
        det_names.emplace_back("/bat_det1");
    }
    if (!wav_path.empty() && seconds < 0.0) seconds = 10.0;

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::unique_ptr<AudioSink> sink;
    if (!wav_path.empty())
        sink = std::make_unique<WavFileSink>(wav_path);
    else
        sink = std::make_unique<QsaAudioSink>(adev);
    if (!sink->start()) {
        std::fprintf(stderr,
                     "error: audio output failed to open%s\n",
                     wav_path.empty()
                         ? " — is io-audio up and the USB headset plugged in? "
                           "(off-device, use --wav)"
                         : "");
        return 1;
    }
    if (tone) {
        const int rc = run_tone(*sink);
        sink->stop();
        return rc;
    }

    std::vector<std::unique_ptr<DetSource>> sources;
    for (size_t i = 0; i < det_names.size(); ++i)
        sources.push_back(std::make_unique<DetSource>(det_names[i], static_cast<int>(i)));

    std::printf("bat_audio: %zu detection ring(s) -> %s\n", sources.size(),
                wav_path.empty() ? "QSA playback" : wav_path.c_str());

    fusion::Tracker tracker;
    fusion::ZoneQuantizer zone;
    bool have_voice = false;
    float voice_azimuth = 0.0f;
    synth::Engine engine(master);
    std::vector<int16_t> block(kBlockFrames * 2);
    std::vector<fusion::Detection> dets;
    uint64_t frames_rendered = 0;
    const uint64_t frame_limit =
        seconds > 0.0 ? static_cast<uint64_t>(seconds * synth::kSampleRate) : 0;

    while (g_run) {
        const uint64_t now = bat_ring_now_ns();
        dets.clear();
        for (auto& src : sources) src->poll(now, &dets);

        // Winner-take-all: exactly one object hums — the closest across both
        // cameras (sticky, so near-ties don't trade the voice) — and its ear
        // is decided by zone: hard left, both, or hard right.
        const std::vector<fusion::Track> tracks = tracker.update(dets, now);
        const fusion::Track* voice =
            fusion::pick_voice(tracks, have_voice, voice_azimuth);
        synth::VoiceTarget target{0.0f, 0.0f, false};
        int n = 0;
        if (voice) {
            voice_azimuth = voice->azimuth_deg;
            have_voice = true;
            target = {voice->closeness, zone.quantize(voice->azimuth_deg), true};
            n = 1;
        } else {
            have_voice = false;
        }
        engine.set_targets(&target, n);

        engine.render(block.data(), kBlockFrames);
        if (!sink->write(block.data(), kBlockFrames)) {
            std::fprintf(stderr, "error: audio write failed\n");
            break;
        }
        frames_rendered += kBlockFrames;
        if (frame_limit && frames_rendered >= frame_limit) break;
        // A file sink never blocks, so pace to wall-clock — live detections
        // (e.g. tools/detfeed.py) must line up with the rendered timeline.
        if (!wav_path.empty())
            usleep(kBlockFrames * 1000000ull / synth::kSampleRate);
    }

    sink->stop();
    std::printf("bat_audio: stopped after %.1f s\n",
                static_cast<double>(frames_rendered) / synth::kSampleRate);
    return 0;
}
