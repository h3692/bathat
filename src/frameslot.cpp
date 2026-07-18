#include "frameslot.h"

#include <cstring>

void FrameSlot::write(const Nv12Frame& f) {
    if (!f.y || !f.uv || f.width <= 0 || f.height <= 0) return;

    std::lock_guard<std::mutex> lk(m_);

    // Store as a tight NV12 buffer (stride == width) so view() is trivial.
    y_.resize(static_cast<size_t>(f.width) * f.height);
    uv_.resize(static_cast<size_t>(f.width) * (f.height / 2));

    for (int r = 0; r < f.height; ++r)
        std::memcpy(y_.data() + static_cast<size_t>(r) * f.width,
                    f.y + static_cast<size_t>(r) * f.y_stride,
                    static_cast<size_t>(f.width));

    for (int r = 0; r < f.height / 2; ++r)
        std::memcpy(uv_.data() + static_cast<size_t>(r) * f.width,
                    f.uv + static_cast<size_t>(r) * f.uv_stride,
                    static_cast<size_t>(f.width));

    width_ = f.width;
    height_ = f.height;
    valid_ = true;
}

Nv12Frame FrameSlot::view() const {
    Nv12Frame v;
    v.y = y_.data();
    v.uv = uv_.data();
    v.width = width_;
    v.height = height_;
    v.y_stride = width_;   // compacted on write
    v.uv_stride = width_;
    return v;
}
