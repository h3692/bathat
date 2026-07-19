#include "qsasink.h"

#include <utility>

#ifdef __QNX__

#include <sys/asoundlib.h>

#include <cstdio>
#include <cstring>

#include "synth.h"

QsaAudioSink::QsaAudioSink(std::string device) : device_(std::move(device)) {}

QsaAudioSink::~QsaAudioSink() { stop(); }

bool QsaAudioSink::start() {
    snd_pcm_t* pcm = nullptr;
    int rc;
    if (device_.empty()) {
        int card = -1, dev = 0;
        rc = snd_pcm_open_preferred(&pcm, &card, &dev, SND_PCM_OPEN_PLAYBACK);
    } else {
        rc = snd_pcm_open_name(&pcm, device_.c_str(), SND_PCM_OPEN_PLAYBACK);
    }
    if (rc < 0) {
        std::fprintf(stderr, "qsa: open failed: %s\n", snd_strerror(rc));
        return false;
    }

    snd_pcm_channel_params_t params;
    std::memset(&params, 0, sizeof(params));
    params.channel = SND_PCM_CHANNEL_PLAYBACK;
    params.mode = SND_PCM_MODE_BLOCK;
    params.start_mode = SND_PCM_START_FULL;
    params.stop_mode = SND_PCM_STOP_STOP;
    params.format.interleave = 1;
    params.format.format = SND_PCM_SFMT_S16_LE;
    params.format.rate = synth::kSampleRate;
    params.format.voices = 2;
    params.buf.block.frag_size = 4096;  // bytes: 1024 stereo S16 frames ~ 21 ms
    params.buf.block.frags_min = 1;
    params.buf.block.frags_max = 4;
    rc = snd_pcm_plugin_params(pcm, &params);
    if (rc < 0) {
        std::fprintf(stderr, "qsa: params failed: %s\n", snd_strerror(rc));
        snd_pcm_close(pcm);
        return false;
    }
    rc = snd_pcm_plugin_prepare(pcm, SND_PCM_CHANNEL_PLAYBACK);
    if (rc < 0) {
        std::fprintf(stderr, "qsa: prepare failed: %s\n", snd_strerror(rc));
        snd_pcm_close(pcm);
        return false;
    }
    handle_ = pcm;
    return true;
}

bool QsaAudioSink::write(const int16_t* frames, int nframes) {
    if (!handle_) return false;
    snd_pcm_t* pcm = static_cast<snd_pcm_t*>(handle_);
    const char* p = reinterpret_cast<const char*>(frames);
    int remaining = nframes * 2 * static_cast<int>(sizeof(int16_t));
    while (remaining > 0) {
        const int wrote = snd_pcm_plugin_write(pcm, p, remaining);
        if (wrote < remaining) {
            // Short write: usually an underrun; re-prepare and push the rest.
            snd_pcm_channel_status_t status;
            std::memset(&status, 0, sizeof(status));
            status.channel = SND_PCM_CHANNEL_PLAYBACK;
            if (snd_pcm_plugin_status(pcm, &status) < 0) return false;
            if (status.status == SND_PCM_STATUS_UNDERRUN ||
                status.status == SND_PCM_STATUS_READY) {
                if (snd_pcm_plugin_prepare(pcm, SND_PCM_CHANNEL_PLAYBACK) < 0)
                    return false;
            } else if (wrote <= 0) {
                return false;
            }
        }
        if (wrote > 0) {
            p += wrote;
            remaining -= wrote;
        }
    }
    return true;
}

void QsaAudioSink::stop() {
    if (!handle_) return;
    snd_pcm_t* pcm = static_cast<snd_pcm_t*>(handle_);
    snd_pcm_plugin_flush(pcm, SND_PCM_CHANNEL_PLAYBACK);
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
