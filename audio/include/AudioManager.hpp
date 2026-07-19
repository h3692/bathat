#ifndef AUDIO_MANAGER_HPP
#define AUDIO_MANAGER_HPP

#include "AudioObstacle.hpp"
#include "AudioOutput.hpp"

#include <chrono>
#include <optional>
#include <string>
#include <vector>

class AudioManager {
public:
    explicit AudioManager(AudioOutput& output);

    void update(
        const std::vector<AudioObstacle>& obstacles
    );

    void reset();

private:
    AudioOutput& output;

    std::optional<AudioObstacle> activeObstacle;
    std::string activeObstacleKey;

    std::chrono::steady_clock::time_point lastPingTime;

    static bool isValid(
        const AudioObstacle& obstacle
    );

    static std::string zoneFromPan(
        float pan
    );

    static std::string makeObstacleKey(
        const AudioObstacle& obstacle
    );

    static float calculatePriority(
        const AudioObstacle& obstacle
    );

    static int pingIntervalMilliseconds(
        int urgency
    );

    static const AudioObstacle* chooseHighestPriority(
        const std::vector<AudioObstacle>& obstacles
    );
};

#endif