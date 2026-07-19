#ifndef AUDIO_OUTPUT_HPP
#define AUDIO_OUTPUT_HPP

#include <string>

class AudioOutput {
public:
    virtual ~AudioOutput() = default;

    virtual void speakObject(
        const std::string& label,
        float pan
    ) = 0;

    virtual void playPing(
        float pan,
        int urgency
    ) = 0;

    virtual void stopAll() = 0;
};

#endif
