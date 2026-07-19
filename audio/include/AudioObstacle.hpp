#ifndef AUDIO_OBSTACLE_HPP
#define AUDIO_OBSTACLE_HPP

#include <string>

struct AudioObstacle {
    std::string label;

    // -1.0 = far left
    //  0.0 = directly ahead
    // +1.0 = far right
    float pan;

    // Temporary score indicating how close the obstacle is.
    float closeness;

    // 0 = clear, 1 = far, 2 = near, 3 = danger
    int urgency;

    // YOLO confidence from 0.0 to 1.0
    float confidence;
};

#endif
