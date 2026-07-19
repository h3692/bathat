#ifndef BATHAT_QSASINK_H
#define BATHAT_QSASINK_H

#include <string>

#include "audiosink.h"

// QNX audio AudioSink — the real output path to the USB headphones on the
// Pi. QNX 8's libasound exposes the ALSA API (not classic QSA): the sink
// opens "default" (or a named device via --adev, e.g. "hw:0,0"), configures
// interleaved S16 stereo at synth::kSampleRate in blocking mode, and recovers
// from underruns via snd_pcm_recover.
//
// On non-QNX builds this compiles to a stub whose start() returns false, so
// the audio process links everywhere and dev machines use --wav instead.
class QsaAudioSink final : public AudioSink {
public:
    // Empty `device` opens "default"; otherwise an ALSA device name.
    explicit QsaAudioSink(std::string device = "");
    ~QsaAudioSink() override;

    bool start() override;
    bool write(const int16_t* frames, int nframes) override;
    void stop() override;

private:
    std::string device_;
    void* handle_ = nullptr;  // snd_pcm_t*, void* keeps this header host-safe
};

#endif  // BATHAT_QSASINK_H
