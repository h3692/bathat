#include "AudioManager.hpp"

#include <cmath>
#include <limits>

AudioManager::AudioManager(AudioOutput& output)
    : output(output),
      activeObstacle(std::nullopt),
      activeObstacleKey(""),
      lastPingTime(std::chrono::steady_clock::now()) {
}

bool AudioManager::isValid(
    const AudioObstacle& obstacle
) {
    return (
        !obstacle.label.empty()
        && obstacle.pan >= -1.0f
        && obstacle.pan <= 1.0f
        && obstacle.urgency >= 0
        && obstacle.urgency <= 3
        && obstacle.confidence >= 0.0f
        && obstacle.confidence <= 1.0f
    );
}

std::string AudioManager::zoneFromPan(
    float pan
) {
    if (pan < -0.35f) {
        return "left";
    }

    if (pan > 0.35f) {
        return "right";
    }

    return "center";
}

std::string AudioManager::makeObstacleKey(
    const AudioObstacle& obstacle
) {
    return obstacle.label + ":" + zoneFromPan(obstacle.pan);
}

float AudioManager::calculatePriority(
    const AudioObstacle& obstacle
) {
    float score = 0.0f;

    score += static_cast<float>(obstacle.urgency) * 100.0f;

    // Gives some extra priority to objects in front.
    score += (1.0f - std::abs(obstacle.pan)) * 20.0f;

    score += obstacle.confidence * 10.0f;
    score += obstacle.closeness * 5.0f;

    return score;
}

int AudioManager::pingIntervalMilliseconds(
    int urgency
) {
    switch (urgency) {
        case 3:
            return 140;

        case 2:
            return 450;

        case 1:
            return 1000;

        default:
            return -1;
    }
}

const AudioObstacle* AudioManager::chooseHighestPriority(
    const std::vector<AudioObstacle>& obstacles
) {
    const AudioObstacle* bestObstacle = nullptr;

    float bestScore =
        -std::numeric_limits<float>::infinity();

    for (const AudioObstacle& obstacle : obstacles) {
        if (!isValid(obstacle)) {
            continue;
        }

        if (obstacle.urgency == 0) {
            continue;
        }

        if (obstacle.confidence < 0.35f) {
            continue;
        }

        const float score =
            calculatePriority(obstacle);

        if (score > bestScore) {
            bestScore = score;
            bestObstacle = &obstacle;
        }
    }

    return bestObstacle;
}

void AudioManager::update(
    const std::vector<AudioObstacle>& obstacles
) {
    const AudioObstacle* selected =
        chooseHighestPriority(obstacles);

    if (selected == nullptr) {
        if (activeObstacle.has_value()) {
            output.stopAll();
        }

        activeObstacle.reset();
        activeObstacleKey.clear();

        return;
    }

    const std::string newKey =
        makeObstacleKey(*selected);

    // Speak only when the active object or zone changes.
    if (
        !activeObstacle.has_value()
        || newKey != activeObstacleKey
    ) {
        output.speakObject(
            selected->label,
            selected->pan
        );

        activeObstacleKey = newKey;

        const int interval =
            pingIntervalMilliseconds(
                selected->urgency
            );

        lastPingTime =
            std::chrono::steady_clock::now()
            - std::chrono::milliseconds(
                interval > 0 ? interval : 0
            );
    }

    activeObstacle = *selected;

    const int interval =
        pingIntervalMilliseconds(
            selected->urgency
        );

    if (interval < 0) {
        return;
    }

    const auto now =
        std::chrono::steady_clock::now();

    const auto elapsed =
        std::chrono::duration_cast<
            std::chrono::milliseconds
        >(now - lastPingTime);

    if (elapsed.count() >= interval) {
        output.playPing(
            selected->pan,
            selected->urgency
        );

        lastPingTime = now;
    }
}

void AudioManager::reset() {
    activeObstacle.reset();
    activeObstacleKey.clear();

    output.stopAll();
}