#ifndef BATHAT_DETVIEW_H
#define BATHAT_DETVIEW_H

#include <cstdint>
#include <string>
#include <vector>

#include "bat_ring.h"
#include "composite.h"

// Reads a detections ring (/bat_det0, written by depth/depth_worker.py) so the
// viewfinder can circle the close blobs on its tiles.
//
// Same lifecycle as DepthView: the worker starts after the capture daemon, so
// poll() keeps retrying the open (throttled) until the ring appears, and holds
// on to the newest record set. Detections describe positions normalized to the
// depth map, so drawing scales them to whatever tile they are overlaid on.
//
// Not thread-safe: poll() and dets() are both meant for the render thread.
class DetView {
public:
    // `name` is the ring name ("/bat_det0") or, with use_shm=0, a plain file
    // path (host tests).
    DetView(std::string name, int use_shm);
    ~DetView();

    // Read the newest detection set if there is one. Returns true if it changed.
    bool poll();

    // Newest records (empty until the first frame arrives).
    const std::vector<bat_det_record>& dets() const { return dets_; }

    // False once the depth worker stops publishing, so stale circles are not
    // drawn over live camera frames.
    bool fresh(uint64_t now_ns, uint64_t max_age_ns = 600000000ull) const {
        return have_frame_ && now_ns >= t_publish_ && now_ns - t_publish_ <= max_age_ns;
    }

private:
    bool try_open(uint64_t now_ns);

    std::string name_;
    int use_shm_;

    bat_ring ring_{};
    bool ring_open_ = false;
    uint64_t next_open_ns_ = 0;      // don't hammer shm_open; retry ~1/s
    uint64_t last_frame_idx_ = ~0ull;
    bool have_frame_ = false;        // last_frame_idx_ holds a real index
    uint64_t t_publish_ = 0;

    bat_det_payload payload_{};
    std::vector<bat_det_record> dets_;
};

// Ring a circle around each detection: a white circle with a black circle just
// inside, visible on any background (no chroma writes). The detections'
// normalized coordinates are mapped onto the tile at (ox, oy) of size
// tile_w x tile_h inside `dst`; drawing clips to both the tile and `dst`.
void det_draw_circles(const std::vector<bat_det_record>& dets, Nv12Dest& dst,
                      int ox, int oy, int tile_w, int tile_h);

#endif  // BATHAT_DETVIEW_H
