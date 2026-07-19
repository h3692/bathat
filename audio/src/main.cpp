#include "AudioManager.hpp"
#include "AudioObstacle.hpp"
#include "AudioOutput.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

AudioOutput* createConsoleAudioOutput();

void runState(
    AudioManager& manager,
    const std::vector<AudioObstacle>& obstacles,
    int durationMilliseconds
) {
    const int updateIntervalMs = 50;
    int elapsed = 0;

    while (elapsed < durationMilliseconds) {
        manager.update(obstacles);

        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                updateIntervalMs
            )
        );

        elapsed += updateIntervalMs;
    }
}

int main() {
    std::unique_ptr<AudioOutput> audioOutput(
        createConsoleAudioOutput()
    );

    AudioManager audioManager(*audioOutput);

    std::cout
        << "\nTEST 1: Chair appears on left\n";

    runState(
        audioManager,
        {
            {
                "chair",
                -0.75f,
                0.70f,
                2,
                0.95f
            }
        },
        3000
    );

    std::cout
        << "\nTEST 2: Chair gets closer\n";

    runState(
        audioManager,
        {
            {
                "chair",
                -0.75f,
                0.95f,
                3,
                0.95f
            }
        },
        2000
    );

    std::cout
        << "\nTEST 3: Chair leaves\n";

    runState(
        audioManager,
        {},
        1000
    );

    std::cout
        << "\nTEST 4: Multiple obstacles\n";

    runState(
        audioManager,
        {
            {
                "table",
                0.75f,
                0.65f,
                2,
                0.92f
            },
            {
                "person",
                0.05f,
                0.55f,
                2,
                0.90f
            },
            {
                "chair",
                -0.80f,
                0.90f,
                3,
                0.96f
            }
        },
        3000
    );

    audioManager.reset();

    return 0;
}