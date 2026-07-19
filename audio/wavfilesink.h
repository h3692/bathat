#ifndef BATHAT_WAVFILESINK_H
#define BATHAT_WAVFILESINK_H

#include <cstdio>
#include <string>

#include "audiosink.h"

// Dev-machine AudioSink: writes the stream to a standard 16-bit stereo WAV
// file so the hum can be rendered and inspected without QNX hardware. The
// RIFF/data sizes are patched on stop().
class WavFileSink final : public AudioSink {
public:
    explicit WavFileSink(std::string path);
    ~WavFileSink() override;

    bool start() override;
    bool write(const int16_t* frames, int nframes) override;
    void stop() override;

private:
    std::string path_;
    FILE* file_ = nullptr;
    uint32_t data_bytes_ = 0;
};

#endif  // BATHAT_WAVFILESINK_H
