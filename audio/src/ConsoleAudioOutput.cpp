#include "AudioOutput.hpp"

#include <iomanip>
#include <iostream>
#include <string>

class ConsoleAudioOutput final : public AudioOutput {
public:
    void speakObject(
        const std::string& label,
        float pan
    ) override {
        std::cout
            << "[SPEECH] \""
            << label
            << "\" pan="
            << std::fixed
            << std::setprecision(2)
            << pan
            << '\n';
    }

    void playPing(
        float pan,
        int urgency
    ) override {
        std::cout
            << "[PING] pan="
            << std::fixed
            << std::setprecision(2)
            << pan
            << ", urgency="
            << urgency
            << '\n';
    }

    void stopAll() override {
        std::cout
            << "[AUDIO] No active obstacle. Stopping."
            << '\n';
    }
};

AudioOutput* createConsoleAudioOutput() {
    return new ConsoleAudioOutput();
}