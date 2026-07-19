#include "AudioManager.hpp"
#include "AudioObstacle.hpp"
#include "WavAudioOutput.hpp"

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

void runState(
    AudioManager& manager,
    const std::vector<AudioObstacle>& obstacles,
    int durationMilliseconds
) {
    const int updateIntervalMilliseconds = 50;
    int elapsedMilliseconds = 0;

    while (
        elapsedMilliseconds <
        durationMilliseconds
    ) {
        manager.update(obstacles);

        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                updateIntervalMilliseconds
            )
        );

        elapsedMilliseconds +=
            updateIntervalMilliseconds;
    }
}

int main() {
    /*
     * Run the finished program from inside the
     * audio directory so that "sounds" points to:
     *
     * audio/sounds/
     */
    WavAudioOutput audioOutput("sounds");

    AudioManager audioManager(audioOutput);

    std::cout
        << "\nTEST 1: Chair appears on the left\n";

    runState(
        audioManager,
        {
            {
                "chair", // YOLO label
                -0.75f,  // left side
                0.70f,   // closeness score
                2,       // near
                0.95f    // confidence
            }
        },
        3000
    );

    std::cout
        << "\nTEST 2: Chair becomes dangerous\n";

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
        << "\nTEST 3: Chair leaves the area\n";

    runState(
        audioManager,
        {},
        1000
    );

    std::cout
        << "\nTEST 4: Unknown YOLO label\n";

    runState(
        audioManager,
        {
            {
                "fire hydrant",
                0.60f,
                0.80f,
                2,
                0.90f
            }
        },
        2000
    );

    audioManager.reset();

    return 0;
}