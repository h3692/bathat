#include "WavAudioOutput.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>

WavAudioOutput::WavAudioOutput(
    const std::string& soundsDirectory
)
    : soundsDirectory(soundsDirectory) {
}

std::string WavAudioOutput::labelToFilename(
    const std::string& label
) const {
    std::string filename = label;

    for (char& character : filename) {
        const unsigned char safeCharacter =
            static_cast<unsigned char>(character);

        if (
            character == ' ' ||
            character == '-' ||
            character == '/'
        ) {
            character = '_';
        } else {
            character = static_cast<char>(
                std::tolower(safeCharacter)
            );
        }
    }

    return filename + ".wav";
}

std::string WavAudioOutput::getObjectSoundPath(
    const std::string& label
) const {
    const std::string filename =
        labelToFilename(label);

    const std::string requestedPath =
        soundsDirectory + "/" + filename;

    if (std::filesystem::exists(requestedPath)) {
        return requestedPath;
    }

    // Use a generic announcement if the exact
    // YOLO label has no prerecorded file.
    return soundsDirectory + "/obstacle.wav";
}

void WavAudioOutput::speakObject(
    const std::string& label,
    float pan
) {
    const std::string soundPath =
        getObjectSoundPath(label);

    std::cout
        << "[OBJECT VOICE] File: "
        << soundPath
        << ", pan: "
        << pan
        << '\n';

    /*
     * This is currently a test implementation.
     *
     * Later, this function will:
     * 1. load the WAV file;
     * 2. turn the mono audio into stereo;
     * 3. adjust left/right volume using pan;
     * 4. send the samples to QNX USB audio.
     */
}

void WavAudioOutput::playPing(
    float pan,
    int urgency
) {
    const std::string pingPath =
        soundsDirectory + "/ping.wav";

    std::cout
        << "[RADAR PING] File: "
        << pingPath
        << ", pan: "
        << pan
        << ", urgency: "
        << urgency
        << '\n';
}

void WavAudioOutput::stopAll() {
    std::cout
        << "[AUDIO STOP] No active obstacle."
        << '\n';
}