#include "qsasink.h"

#include <utility>

#ifdef __QNX__

// QNX 8 ships an ALSA-compatible libasound (snd_pcm_open/set_params/writei —
// confirmed against the image's exported symbols) but no development header,
// so the handful of declarations the sink needs are vendored here. Only
// opaque handles, ints, and longs cross this boundary — no struct layouts —
// which keeps the hand declarations ABI-safe.
extern "C" {
typedef struct _snd_pcm snd_pcm_t;
int snd_pcm_open(snd_pcm_t** pcm, const char* name, int stream, int mode);
int snd_pcm_set_params(snd_pcm_t* pcm, int format, int access,
                       unsigned channels, unsigned rate, int soft_resample,
                       unsigned latency_us);
long snd_pcm_writei(snd_pcm_t* pcm, const void* buffer, unsigned long frames);
int snd_pcm_recover(snd_pcm_t* pcm, int err, int silent);
int snd_pcm_drain(snd_pcm_t* pcm);
int snd_pcm_close(snd_pcm_t* pcm);
const char* snd_strerror(int errnum);
}

namespace {
constexpr int kStreamPlayback = 0;    // SND_PCM_STREAM_PLAYBACK
constexpr int kFormatS16LE = 2;       // SND_PCM_FORMAT_S16_LE
constexpr int kAccessRwInterleaved = 3;  // SND_PCM_ACCESS_RW_INTERLEAVED
constexpr unsigned kLatencyUs = 100000;  // 100 ms device buffer
}  // namespace

#include <cstdio>

#include "synth.h"

QsaAudioSink::QsaAudioSink(std::string device) : device_(std::move(device)) {}

QsaAudioSink::~QsaAudioSink() { stop(); }

bool QsaAudioSink::start() {
    snd_pcm_t* pcm = nullptr;
    const char* name = device_.empty() ? "default" : device_.c_str();
    int rc = snd_pcm_open(&pcm, name, kStreamPlayback, 0);
    if (rc < 0) {
        std::fprintf(stderr, "alsa: open '%s' failed: %s\n", name, snd_strerror(rc));
        return false;
    }
    rc = snd_pcm_set_params(pcm, kFormatS16LE, kAccessRwInterleaved,
                            /*channels=*/2, synth::kSampleRate,
                            /*soft_resample=*/1, kLatencyUs);
    if (rc < 0) {
        std::fprintf(stderr, "alsa: set_params failed: %s\n", snd_strerror(rc));
        snd_pcm_close(pcm);
        return false;
    }
    handle_ = pcm;
    return true;
}

bool QsaAudioSink::write(const int16_t* frames, int nframes) {
    if (!handle_) return false;
    snd_pcm_t* pcm = static_cast<snd_pcm_t*>(handle_);
    const int16_t* p = frames;
    long remaining = nframes;
    while (remaining > 0) {
        const long n = snd_pcm_writei(pcm, p, static_cast<unsigned long>(remaining));
        if (n < 0) {
            // Usually an underrun; recover() re-prepares and we resend.
            const int rc = snd_pcm_recover(pcm, static_cast<int>(n), /*silent=*/1);
            if (rc < 0) {
                std::fprintf(stderr, "alsa: write failed: %s\n", snd_strerror(rc));
                return false;
            }
            continue;
        }
        p += n * 2;
        remaining -= n;
    }
    return true;
}

void QsaAudioSink::stop() {
    if (!handle_) return;
    snd_pcm_t* pcm = static_cast<snd_pcm_t*>(handle_);
    snd_pcm_drain(pcm);
    snd_pcm_close(pcm);
    handle_ = nullptr;
}

#else  // !__QNX__: link-anywhere stub; dev machines render with --wav

QsaAudioSink::QsaAudioSink(std::string device) : device_(std::move(device)) {}

QsaAudioSink::~QsaAudioSink() = default;

bool QsaAudioSink::start() { return false; }

bool QsaAudioSink::write(const int16_t*, int) { return false; }

void QsaAudioSink::stop() {}

#endif
