#ifndef BATHAT_DEPTHVIEW_H
#define BATHAT_DEPTHVIEW_H

#include <cstdint>
#include <string>
#include <vector>

#include "bat_ring.h"
#include "composite.h"

// Turns a float32 depth ring (/bat_depth0, written by depth/depth_worker.py)
// into an NV12 tile the viewfinder can composite next to the camera feed.
//
// The depth worker starts after the capture daemon, so the ring may not exist
// yet: poll() keeps retrying the open (throttled) until it appears. Once a
// frame arrives it is colorized with the inferno colormap (bright/warm = near,
// matching tools/ringdump.py) and nearest-neighbor upscaled into a tight NV12
// buffer of the requested tile size.
//
// MiDaS depth is relative with an arbitrary per-frame scale, so poll()
// normalizes against an exponential rolling min/max instead of each frame's
// own extremes -- per-frame normalization makes the whole tile pulse.
//
// Not thread-safe: poll() and view() are both meant for the render thread.
class DepthView {
public:
    // `name` is the ring name ("/bat_depth0") or, with use_shm=0, a plain
    // file path (host tests). The tile is tile_w x tile_h pixels (even).
    DepthView(std::string name, int use_shm, int tile_w, int tile_h);
    ~DepthView();

    // Read the newest depth frame if there is one and refresh the tile.
    // Returns true if the tile changed. Cheap when there is nothing new.
    bool poll();

    // Valid only after poll() returned true at least once.
    bool valid() const { return valid_; }
    Nv12Frame view() const;

    // Total frames the worker has published (0 until the ring opens) -- lets
    // the main loop derive a depth fps from successive samples.
    uint64_t wr_count() const { return ring_open_ ? ring_.hdr->wr_count : 0; }

private:
    bool try_open(uint64_t now_ns);
    void colorize(const float* depth, int w, int h);

    std::string name_;
    int use_shm_;
    int tile_w_, tile_h_;

    bat_ring ring_{};
    bool ring_open_ = false;
    uint64_t next_open_ns_ = 0;      // don't hammer shm_open; retry ~1/s
    uint64_t last_frame_idx_ = ~0ull;
    bool have_frame_ = false;        // last_frame_idx_ holds a real index

    std::vector<uint8_t> payload_;   // raw float32 frame from the ring
    std::vector<uint8_t> y_, uv_;    // colorized tile, tight NV12
    bool valid_ = false;

    // Rolling normalization state (seeded by the first frame).
    bool norm_init_ = false;
    float lo_ = 0.0f, hi_ = 1.0f;

    // Colormap LUTs: depth index -> Y, and -> interleaved U,V.
    uint8_t lut_y_[256];
    uint8_t lut_u_[256];
    uint8_t lut_v_[256];
};

#endif  // BATHAT_DEPTHVIEW_H
