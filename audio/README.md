# Audio System

This module will convert YOLO and MiDaS obstacle data into spoken object labels and repeating spatial pings.

# Audio Radar Module

This module converts fused YOLO and MiDaS results into
spoken object announcements and repeating spatial warning
pings.

## Input

The sensing pipeline provides:

```cpp
struct AudioObstacle {
    std::string label;
    float pan;
    float closeness;
    int urgency;
    float confidence;
};