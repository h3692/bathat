#ifndef BATHAT_CAMERA_H
#define BATHAT_CAMERA_H

#include <camera/camera_api.h>

#include "bat_ring.h"
#include "frameslot.h"

// Print every camera the framework reports, plus each one's supported
// viewfinder frame types, resolutions, and EV-offset support. Run this first
// on the Pi (`bathat --probe`) to turn the hardware unknowns into printed
// facts. Returns 0 on success, non-zero if enumeration failed.
int camera_probe_all();

// Where a camera's frames go. Either destination may be null:
//  - slot feeds the on-screen viewfinder (skipped with --no-display),
//  - ring publishes to the shared-memory ring the depth worker reads.
struct FrameSink {
    FrameSlot* slot = nullptr;
    bat_ring* ring = nullptr;
};

// One running camera that streams NV12 viewfinder frames into a FrameSink.
class CameraStream {
public:
    // Open `unit`, configure an NV12 viewfinder at width x height, enable auto
    // white balance, set manual exposure (ISO + shutter seconds, each clamped to
    // the sensor's supported range), and start streaming into `sink`.
    bool start(camera_unit_t unit, int width, int height, unsigned iso, double shutter,
               FrameSink& sink);

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
