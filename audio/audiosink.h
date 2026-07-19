#ifndef BATHAT_AUDIOSINK_H
#define BATHAT_AUDIOSINK_H

#include <cstdint>

// Where the synthesized hum goes: QNX USB headphones on the device
// (QsaAudioSink), a WAV file on the dev machine (WavFileSink). Everything is
// interleaved stereo S16 at synth::kSampleRate.
class AudioSink {
public:
    virtual ~AudioSink() = default;

    // Open the output. False = unusable (caller should bail with a message).
    virtual bool start() = 0;

    // Blocking write of `nframes` interleaved stereo frames; the block-rate
    // backpressure of the real device is what paces the render loop.
    virtual bool write(const int16_t* frames, int nframes) = 0;

    virtual void stop() = 0;
};

#endif  // BATHAT_AUDIOSINK_H
