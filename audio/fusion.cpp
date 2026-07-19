#include "fusion.h"

#include <algorithm>
#include <cmath>

namespace fusion {

namespace {

struct Candidate {
    float closeness;
    float azimuth_deg;
};

}  // namespace

std::vector<Track> Tracker::update(const std::vector<Detection>& dets,
                                   uint64_t now_ns) {
    // 1. Merge the cameras' overlap: greedily pair each camera-0 detection
    // with the nearest camera-1 detection inside the gate. Same-camera blobs
    // are distinct objects by construction (connected components already
    // separated them), so they never merge.
    std::vector<const Detection*> cam0, cam1;
    for (const Detection& d : dets)
        (d.camera == 0 ? cam0 : cam1).push_back(&d);

    std::vector<Candidate> cands;
    std::vector<bool> taken(cam1.size(), false);
    for (const Detection* a : cam0) {
        int best = -1;
        float best_gap = kMergeGateDeg;
        for (size_t j = 0; j < cam1.size(); ++j) {
            if (taken[j]) continue;
            const float gap = std::fabs(a->azimuth_deg - cam1[j]->azimuth_deg);
            if (gap <= best_gap) {
                best_gap = gap;
                best = static_cast<int>(j);
            }
        }
        if (best >= 0) {
            const Detection* b = cam1[best];
            taken[best] = true;
            const float wsum = a->closeness + b->closeness;
            cands.push_back({std::max(a->closeness, b->closeness),
                             wsum > 0.0f
                                 ? (a->azimuth_deg * a->closeness +
                                    b->azimuth_deg * b->closeness) / wsum
                                 : 0.5f * (a->azimuth_deg + b->azimuth_deg)});
        } else {
            cands.push_back({a->closeness, a->azimuth_deg});
        }
    }
    for (size_t j = 0; j < cam1.size(); ++j)
        if (!taken[j]) cands.push_back({cam1[j]->closeness, cam1[j]->azimuth_deg});

    // 2. Only the nearest few can ever make a sound.
    std::sort(cands.begin(), cands.end(),
              [](const Candidate& x, const Candidate& y) {
                  return x.closeness > y.closeness;
              });
    if (cands.size() > kMaxTracks) cands.resize(kMaxTracks);

    // 3. Associate candidates to tracks by nearest azimuth; new objects take a
    // free slot, or evict the weakest unmatched track if strictly closer.
    bool matched[kMaxTracks] = {false, false, false};
    for (const Candidate& c : cands) {
        int best = -1;
        float best_gap = kTrackGateDeg;
        for (int i = 0; i < kMaxTracks; ++i) {
            if (!slots_[i].used || matched[i]) continue;
            const float gap = std::fabs(c.azimuth_deg - slots_[i].azimuth_deg);
            if (gap <= best_gap) {
                best_gap = gap;
                best = i;
            }
        }
        if (best < 0) {
            for (int i = 0; i < kMaxTracks; ++i) {
                if (!slots_[i].used) {
                    best = i;
                    break;
                }
            }
            if (best >= 0) {
                slots_[best] = Slot{true, c.closeness, c.azimuth_deg, now_ns,
                                    next_id_++};
                matched[best] = true;
                continue;
            }
            int weakest = -1;
            for (int i = 0; i < kMaxTracks; ++i) {
                if (matched[i]) continue;
                if (weakest < 0 || slots_[i].closeness < slots_[weakest].closeness)
                    weakest = i;
            }
            if (weakest >= 0 && c.closeness > slots_[weakest].closeness) {
                slots_[weakest] = Slot{true, c.closeness, c.azimuth_deg, now_ns,
                                       next_id_++};
                matched[weakest] = true;
            }
            continue;
        }
        Slot& s = slots_[best];
        s.azimuth_deg += kEma * (c.azimuth_deg - s.azimuth_deg);
        s.closeness += kEma * (c.closeness - s.closeness);
        s.last_hit_ns = now_ns;
        matched[best] = true;
    }

    // 4. Tracks that missed this round survive the hold window (the cameras
    // alternate, so every object legitimately skips updates), then drop.
    for (int i = 0; i < kMaxTracks; ++i)
        if (slots_[i].used && !matched[i] &&
            now_ns - slots_[i].last_hit_ns > kHoldNs)
            slots_[i].used = false;

    // 5. Rank nearest-first: index = the synthesizer's voice rank.
    std::vector<Track> tracks;
    for (const Slot& s : slots_)
        if (s.used) tracks.push_back({s.closeness, s.azimuth_deg, s.id});
    std::sort(tracks.begin(), tracks.end(),
              [](const Track& x, const Track& y) {
                  return x.closeness > y.closeness;
              });
    return tracks;
}

const Track* pick_voice(const std::vector<Track>& tracks, bool have_holder,
                        float holder_azimuth_deg) {
    if (tracks.empty()) return nullptr;
    const Track* best = &tracks[0];
    if (!have_holder) return best;
    for (const Track& t : tracks) {
        if (std::fabs(t.azimuth_deg - holder_azimuth_deg) <= kVoiceStickyGateDeg &&
            best->closeness - t.closeness <= kVoiceStickyMargin)
            return &t;
    }
    return best;
}

float ZoneQuantizer::quantize(float azimuth_deg) {
    switch (zone_) {
        case Zone::Left:
            if (azimuth_deg > -(kEdgeDeg - kHystDeg))
                zone_ = azimuth_deg > kEdgeDeg ? Zone::Right : Zone::Center;
            break;
        case Zone::Right:
            if (azimuth_deg < kEdgeDeg - kHystDeg)
                zone_ = azimuth_deg < -kEdgeDeg ? Zone::Left : Zone::Center;
            break;
        case Zone::Center:
            if (azimuth_deg < -kEdgeDeg) zone_ = Zone::Left;
            else if (azimuth_deg > kEdgeDeg) zone_ = Zone::Right;
            break;
    }
    return zone_ == Zone::Left ? -90.0f : zone_ == Zone::Right ? 90.0f : 0.0f;
}

}  // namespace fusion
