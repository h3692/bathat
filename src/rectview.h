#ifndef BATHAT_RECTVIEW_H
#define BATHAT_RECTVIEW_H

#include <cstdint>
#include <string>
#include <vector>

#include "bat_ring.h"
#include "composite.h"

// Turns a BGR ring (/bat_rect0, written by undistort/undistort_worker.py) into
// an NV12 tile the viewfinder can composite as a camera panel -- the rectified
// counterpart of DepthView.
//
// The undistort worker starts after the capture daemon, so the ring may not
// exist yet: poll() keeps retrying the open (throttled) until it appears. The
// source frame is nearest-neighbor scaled into the requested tile size and
// converted BGR -> NV12 (same BT.601 math as DepthView).
//
// Not thread-safe: poll() and view() are both meant for the render thread.
class RectView {
public:
    // `name` is the ring name ("/bat_rect0") or, with use_shm=0, a plain file
    // path (host tests). The tile is tile_w x tile_h pixels (even).
    RectView(std::string name, int use_shm, int tile_w, int tile_h);
    ~RectView();

    // Read the newest rectified frame if there is one and refresh the tile.
    // Returns true if the tile changed. Cheap when there is nothing new.
    bool poll();

    // Valid only after poll() returned true at least once.
    bool valid() const { return valid_; }
    Nv12Frame view() const;

private:
    bool try_open(uint64_t now_ns);
    void convert(const uint8_t* bgr, int w, int h);

    std::string name_;
    int use_shm_;
    int tile_w_, tile_h_;

    bat_ring ring_{};
    bool ring_open_ = false;
    uint64_t next_open_ns_ = 0;      // don't hammer shm_open; retry ~1/s
    uint64_t last_frame_idx_ = ~0ull;
    bool have_frame_ = false;

    std::vector<uint8_t> payload_;   // raw BGR frame from the ring
    std::vector<uint8_t> y_, uv_;    // converted tile, tight NV12
    bool valid_ = false;
};

#endif  // BATHAT_RECTVIEW_H
