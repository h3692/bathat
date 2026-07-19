#include "rectview.h"

#include <algorithm>
#include <utility>
#include <vector>

namespace {

// BT.601 full-range RGB -> YUV, matching DepthView so camera panels and depth
// tiles share the same colour space.
uint8_t clamp_u8(float v) {
    return static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, v + 0.5f)));
}

}  // namespace

RectView::RectView(std::string name, int use_shm, int tile_w, int tile_h)
    : name_(std::move(name)), use_shm_(use_shm),
      tile_w_(tile_w & ~1), tile_h_(tile_h & ~1) {
    y_.assign(static_cast<size_t>(tile_w_) * tile_h_, 0);            // black...
    uv_.assign(static_cast<size_t>(tile_w_) * (tile_h_ / 2), 0x80);  // ...not green
}

RectView::~RectView() {
    if (ring_open_) bat_ring_close(&ring_);
}

bool RectView::try_open(uint64_t now_ns) {
    if (now_ns < next_open_ns_) return false;
    next_open_ns_ = now_ns + 1000000000ull;  // retry ~1/s until the worker is up
    if (bat_ring_open(&ring_, name_.c_str(), use_shm_) != 0) return false;
    if (ring_.hdr->format != BAT_FMT_BGR8 ||
        ring_.hdr->width == 0 || ring_.hdr->height == 0) {
        bat_ring_close(&ring_);
        return false;
    }
    payload_.resize(ring_.hdr->slot_size);
    ring_open_ = true;
    return true;
}

bool RectView::poll() {
    if (!ring_open_ && !try_open(bat_ring_now_ns())) return false;

    bat_slot_hdr meta;
    if (bat_ring_read_latest(&ring_, payload_.data(), payload_.size(), &meta) != 1)
        return false;
    if (have_frame_ && meta.frame_idx == last_frame_idx_) return false;

    const uint32_t w = ring_.hdr->width, h = ring_.hdr->height;
    if (meta.size != static_cast<uint64_t>(w) * h * 3) return false;

    convert(payload_.data(), static_cast<int>(w), static_cast<int>(h));
    last_frame_idx_ = meta.frame_idx;
    have_frame_ = true;
    valid_ = true;
    return true;
}

void RectView::convert(const uint8_t* bgr, int w, int h) {
    // Source column for each tile column (nearest-neighbour scale).
    std::vector<int> col_of(tile_w_);
    for (int tx = 0; tx < tile_w_; ++tx)
        col_of[tx] = std::min(tx * w / tile_w_, w - 1);

    // Y plane: full tile resolution. Payload is interleaved B,G,R per pixel.
    for (int ty = 0; ty < tile_h_; ++ty) {
        const int sy = std::min(ty * h / tile_h_, h - 1);
        const uint8_t* row = bgr + static_cast<size_t>(sy) * w * 3;
        uint8_t* out = y_.data() + static_cast<size_t>(ty) * tile_w_;
        for (int tx = 0; tx < tile_w_; ++tx) {
            const uint8_t* px = row + static_cast<size_t>(col_of[tx]) * 3;
            const float b = px[0], g = px[1], r = px[2];
            out[tx] = clamp_u8(0.299f * r + 0.587f * g + 0.114f * b);
        }
    }

    // Chroma at half resolution: sample the top-left pixel of each 2x2 block.
    for (int ty = 0; ty < tile_h_ / 2; ++ty) {
        const int sy = std::min(2 * ty * h / tile_h_, h - 1);
        const uint8_t* row = bgr + static_cast<size_t>(sy) * w * 3;
        uint8_t* out = uv_.data() + static_cast<size_t>(ty) * tile_w_;
        for (int tx = 0; tx < tile_w_ / 2; ++tx) {
            const uint8_t* px = row + static_cast<size_t>(col_of[2 * tx]) * 3;
            const float b = px[0], g = px[1], r = px[2];
            out[2 * tx] = clamp_u8(128.0f - 0.168736f * r - 0.331264f * g + 0.5f * b);
            out[2 * tx + 1] = clamp_u8(128.0f + 0.5f * r - 0.418688f * g - 0.081312f * b);
        }
    }
}

Nv12Frame RectView::view() const {
    Nv12Frame v;
    v.y = y_.data();
    v.uv = uv_.data();
    v.width = tile_w_;
    v.height = tile_h_;
    v.y_stride = tile_w_;
    v.uv_stride = tile_w_;
    return v;
}
