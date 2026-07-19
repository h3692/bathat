#ifndef WAV_AUDIO_OUTPUT_HPP
#define WAV_AUDIO_OUTPUT_HPP

#include "AudioOutput.hpp"

#include <string>

class WavAudioOutput final : public AudioOutput {
public:
    // soundsDirectory will usually be "sounds".
    explicit WavAudioOutput(
        const std::string& soundsDirectory
    );

    // Selects and eventually plays an object-name WAV file.
    void speakObject(
        const std::string& label,
        float pan
    ) override;

    // Selects and eventually plays the radar ping.
    void playPing(
        float pan,
        int urgency
    ) override;

    // Stops any ongoing audio.
    void stopAll() override;

private:
    std::string soundsDirectory;

    // Turns a YOLO label into a safe filename.
    std::string labelToFilename(
        const std::string& label
    ) const;

    // Returns the correct sound path or obstacle.wav
    // when there is no matching object file.
    std::string getObjectSoundPath(
        const std::string& label
    ) const;
};

#endif