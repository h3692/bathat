#ifndef BATHAT_CAMERA_H
#define BATHAT_CAMERA_H

#include <camera/camera_api.h>

#include "frameslot.h"

// Print every camera the framework reports, plus each one's supported
// viewfinder frame types, resolutions, and EV-offset support. Run this first
// on the Pi (`bathat --probe`) to turn the hardware unknowns into printed
// facts. Returns 0 on success, non-zero if enumeration failed.
int camera_probe_all();

// One running camera that streams NV12 viewfinder frames into a FrameSlot.
class CameraStream {
public:
    // Open `unit`, configure an NV12 viewfinder at width x height, set
    // auto-exposure with a positive EV bias (clamped to what the sensor
    // supports), and start streaming into `slot`. Returns true on success.
    bool start(camera_unit_t unit, int width, int height, double ev_bias, FrameSlot& slot);

    // Stop the viewfinder and close the camera. Safe to call more than once.
    void stop();

    camera_unit_t unit() const { return unit_; }

    ~CameraStream();

private:
    camera_handle_t handle_ = CAMERA_HANDLE_INVALID;
    camera_unit_t unit_ = CAMERA_UNIT_NONE;
    bool streaming_ = false;
};

#endif  // BATHAT_CAMERA_H
