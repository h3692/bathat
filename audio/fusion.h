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
    uint32_t id = 0;    // stable while the track lives; fresh per new object,
                        // never reused — the announcer dedupes on it
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
    // Smoothing and hold are sized for the *actual* detection cadence, which
    // can dip below 1 fps per camera on the Pi: heavy smoothing or a short
    // hold at that rate means seconds of reaction lag and a hum that dies and
    // re-attacks between inferences.
    static constexpr float kEma = 0.6f;            // per-update smoothing
    static constexpr uint64_t kHoldNs = 1200000000ull;  // 1.2 s miss grace

    // Feed the newest detections from both cameras (concatenated) and the
    // current CLOCK_MONOTONIC time; returns active tracks, nearest first.
    std::vector<Track> update(const std::vector<Detection>& dets, uint64_t now_ns);

private:
    struct Slot {
        bool used = false;
        float closeness = 0.0f;
        float azimuth_deg = 0.0f;
        uint64_t last_hit_ns = 0;
        uint32_t id = 0;
    };
    Slot slots_[kMaxTracks];
    uint32_t next_id_ = 1;
};

// Winner-take-all voice selection: only one object hums at a time. The
// nearest track wins, but the current holder (identified by its previous
// azimuth) keeps the voice unless a challenger beats it by a clear closeness
// margin — otherwise two similar objects would trade the voice every frame.
// Returns a pointer into `tracks`, or nullptr when there is nothing to voice.
constexpr float kVoiceStickyGateDeg = 25.0f;
constexpr float kVoiceStickyMargin = 0.08f;
const Track* pick_voice(const std::vector<Track>& tracks, bool have_holder,
                        float holder_azimuth_deg);

// Deterministic ear assignment: azimuths quantize to hard left (-90), both
// ears (0), or hard right (+90), with a hysteresis band so a borderline
// object cannot flutter between zones. The synth's pan slew then glides the
// (rare) zone changes instead of snapping.
class ZoneQuantizer {
public:
    static constexpr float kEdgeDeg = 18.0f;  // |azimuth| beyond this = side zone
    static constexpr float kHystDeg = 6.0f;   // come this far back to leave it

    // Feed the voiced object's azimuth; returns -90, 0, or +90.
    float quantize(float azimuth_deg);

private:
    enum class Zone { Left, Center, Right };
    Zone zone_ = Zone::Center;
};

}  // namespace fusion

#endif  // BATHAT_FUSION_H
