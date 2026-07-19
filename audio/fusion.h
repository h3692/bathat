#ifndef BATHAT_FUSION_H
#define BATHAT_FUSION_H

#include <cstdint>
#include <vector>

// Turns the two cameras' per-frame blob detections into a stable set of at
// most three tracked objects for the synthesizer.
//
// The cameras' fields overlap in the middle of the 180-degree arc, so a blob
// seen by both appears twice at nearly the same world azimuth: detections from
// *different* cameras within the merge gate collapse into one object (azimuth =
// closeness-weighted mean, closeness = max — geometry is trusted, the
// cross-camera closeness comparison is not load-bearing).
//
// Tracks bridge the slow, alternating detection cadence (~4 fps per camera):
// a detection near an existing track updates it (EMA-smoothed), a track that
// misses updates is held for a grace period before it drops, and a full table
// only trades its weakest track for a strictly closer object. The result is
// ranked nearest-first, which is exactly the synthesizer's voice order.
namespace fusion {

// One blob from one camera, already converted to world azimuth.
struct Detection {
    float closeness;    // [0,1], 1 = nearest (per-camera rolling-normalized)
    float azimuth_deg;  // 0 = ahead, positive = the user's right
    int camera;         // 0 or 1
};

struct Track {
    float closeness;    // EMA-smoothed
    float azimuth_deg;  // EMA-smoothed
};

class Tracker {
public:
    static constexpr int kMaxTracks = 3;
    // Wide gates on purpose: the mounting yaws are approximate, so the two
    // cameras can disagree noticeably about one object in the overlap; if the
    // gates are tighter than that disagreement, the object splits into two
    // alternating tracks and the pan ping-pongs around the middle.
    static constexpr float kMergeGateDeg = 20.0f;  // cross-camera same-object gate
    static constexpr float kTrackGateDeg = 22.0f;  // detection-to-track gate
    static constexpr float kEma = 0.4f;            // per-update smoothing
    static constexpr uint64_t kHoldNs = 300000000ull;  // 300 ms miss grace

    // Feed the newest detections from both cameras (concatenated) and the
    // current CLOCK_MONOTONIC time; returns active tracks, nearest first.
    std::vector<Track> update(const std::vector<Detection>& dets, uint64_t now_ns);

private:
    struct Slot {
        bool used = false;
        float closeness = 0.0f;
        float azimuth_deg = 0.0f;
        uint64_t last_hit_ns = 0;
    };
    Slot slots_[kMaxTracks];
};

}  // namespace fusion

#endif  // BATHAT_FUSION_H
