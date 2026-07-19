#ifndef BATHAT_QSASINK_H
#define BATHAT_QSASINK_H

#include <string>

#include "audiosink.h"

// QNX io-audio (QSA) AudioSink — the real output path to the USB headphones
// on the Pi. Opens the preferred PCM playback device (or a named one via
// --adev), configures interleaved S16 stereo at synth::kSampleRate in blocking
// mode, and recovers from underruns by re-preparing the channel.
//
// On non-QNX builds this compiles to a stub whose start() returns false, so
// the audio process links everywhere and dev machines use --wav instead.
class QsaAudioSink final : public AudioSink {
public:
    // Empty `device` picks the system's preferred playback device; otherwise
    // a QSA device name (snd_pcm_open_name).
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
