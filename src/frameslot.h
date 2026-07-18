#ifndef BATHAT_FRAMESLOT_H
#define BATHAT_FRAMESLOT_H

#include <cstdint>
#include <mutex>
#include <vector>

#include "composite.h"

// Thread-safe holder for the most recent NV12 frame from one camera.
//
// The QNX camera framework guarantees a viewfinder buffer's pixel data is
// valid ONLY for the duration of the callback, so write() copies the data into
// owned storage (compacted to a tight stride == width layout).
//
// The render loop reads by locking the slot's mutex, checking valid(), and
// using view() while the lock is held. Lock two slots together with
// std::scoped_lock to avoid deadlock.
class FrameSlot {
public:
    // Copy an NV12 frame into the slot. Called from the camera callback.
    void write(const Nv12Frame& frame);

    std::mutex& mutex() { return m_; }

    // The following are valid only while mutex() is held.
    bool valid() const { return valid_; }
    Nv12Frame view() const;

private:
    std::mutex m_;
    std::vector<uint8_t> y_, uv_;
    int width_ = 0, height_ = 0;
    bool valid_ = false;
};

#endif  // BATHAT_FRAMESLOT_H
