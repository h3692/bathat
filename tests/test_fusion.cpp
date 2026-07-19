// Host unit test for cross-camera fusion + tracking: overlap merging, top-3
// selection, azimuth association with EMA smoothing, and the hold timer that
// bridges the alternating-camera update gaps. Build and run with
//   make -C tests
#include "fusion.h"

#include <cmath>
#include <cstdio>
#include <vector>

static int failures = 0;

#define CHECK(cond, msg)                                                        \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::printf("FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__);       \
            ++failures;                                                         \
        }                                                                       \
    } while (0)

static bool close_to(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

static constexpr uint64_t kMs = 1000000ull;  // ns per ms

int main() {
    using fusion::Detection;
    using fusion::Track;
    using fusion::Tracker;

    // Two cameras seeing the same object in the overlap: one merged track,
    // closeness = max, azimuth = closeness-weighted mean.
    {
        Tracker tracker;
        const std::vector<Detection> dets = {
            {0.8f, 40.0f, 0}, {0.6f, 44.0f, 1}};
        const std::vector<Track> tracks = tracker.update(dets, kMs);
        CHECK(tracks.size() == 1, "overlap pair merges to one track");
        CHECK(close_to(tracks[0].closeness, 0.8f, 1e-4f), "merged closeness is the max");
        const float want_az = (40.0f * 0.8f + 44.0f * 0.6f) / 1.4f;
        CHECK(close_to(tracks[0].azimuth_deg, want_az, 0.01f),
              "merged azimuth is the closeness-weighted mean");
    }

    // Same two azimuths from the SAME camera stay separate objects.
    {
        Tracker tracker;
        const std::vector<Detection> dets = {
            {0.8f, 40.0f, 0}, {0.6f, 44.0f, 0}};
        CHECK(tracker.update(dets, kMs).size() == 2, "same-camera pair never merges");
    }

    // Azimuths beyond the merge gate are two objects.
    {
        Tracker tracker;
        const std::vector<Detection> dets = {
            {0.8f, -30.0f, 0}, {0.6f, 30.0f, 1}};
        const std::vector<Track> tracks = tracker.update(dets, kMs);
        CHECK(tracks.size() == 2, "distinct azimuths make two tracks");
        CHECK(tracks[0].closeness > tracks[1].closeness, "tracks rank nearest-first");
    }

    // More objects than voices: only the top 3 by closeness survive.
    {
        Tracker tracker;
        const std::vector<Detection> dets = {
            {0.9f, -80.0f, 0}, {0.7f, -40.0f, 0}, {0.5f, 0.0f, 0},
            {0.3f, 40.0f, 1}, {0.2f, 80.0f, 1}};
        const std::vector<Track> tracks = tracker.update(dets, kMs);
        CHECK(tracks.size() == 3, "capped at three tracks");
        CHECK(close_to(tracks[2].closeness, 0.5f, 1e-4f), "weakest survivor is the 3rd nearest");
    }

    // Association: a moving object updates its track (EMA), not a new one.
    {
        Tracker tracker;
        tracker.update({{1.0f, 0.0f, 0}}, kMs);
        const std::vector<Track> tracks = tracker.update({{1.0f, 10.0f, 0}}, 2 * kMs);
        CHECK(tracks.size() == 1, "moving object keeps one track");
        CHECK(close_to(tracks[0].azimuth_deg, 4.0f, 0.01f),
              "azimuth eases toward the new detection (EMA 0.4)");
    }

    // The hold timer: a missed update inside 300 ms keeps the track alive
    // (bridging the alternating-camera cadence), a longer gap drops it.
    {
        Tracker tracker;
        tracker.update({{1.0f, 20.0f, 0}}, kMs);
        CHECK(tracker.update({}, kMs + 200 * kMs).size() == 1,
              "missed update inside the hold window keeps the track");
        CHECK(tracker.update({}, kMs + 400 * kMs).empty(),
              "track drops after the hold window");
    }

    // A full table only trades its weakest track for a strictly closer object.
    {
        Tracker tracker;
        tracker.update({{0.9f, -80.0f, 0}, {0.7f, 0.0f, 0}, {0.5f, 80.0f, 1}}, kMs);
        // A new 0.6-closeness object: closer than the weakest track -> evicts it.
        std::vector<Track> tracks =
            tracker.update({{0.9f, -80.0f, 0}, {0.7f, 0.0f, 0}, {0.6f, 40.0f, 1}},
                           2 * kMs);
        CHECK(tracks.size() == 3, "full table stays at three");
        bool has_40 = false, has_80 = false;
        for (const Track& t : tracks) {
            has_40 |= close_to(t.azimuth_deg, 40.0f, 1.0f);
            has_80 |= close_to(t.azimuth_deg, 80.0f, 1.0f);
        }
        CHECK(has_40 && !has_80, "closer new object evicts the weakest track");
        // A farther new object (0.3 < every track) changes nothing.
        tracks = tracker.update({{0.9f, -80.0f, 0}, {0.7f, 0.0f, 0},
                                 {0.6f, 40.0f, 1}, {0.3f, 80.0f, 1}}, 3 * kMs);
        has_80 = false;
        for (const Track& t : tracks) has_80 |= close_to(t.azimuth_deg, 80.0f, 1.0f);
        CHECK(tracks.size() == 3 && !has_80,
              "farther object does not evict a nearer track");
    }

    if (failures == 0) {
        std::printf("PASS: fusion (all checks)\n");
        return 0;
    }
    std::printf("%d check(s) FAILED\n", failures);
    return 1;
}
