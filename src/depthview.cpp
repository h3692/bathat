#include "depthview.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "inferno_lut.h"

namespace {

// BT.601 full-range RGB -> YUV, good enough for a debug visualization.
uint8_t clamp_u8(float v) {
    return static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, v + 0.5f)));
}

}  // namespace

DepthView::DepthView(std::string name, int use_shm, int tile_w, int tile_h)
    : name_(std::move(name)), use_shm_(use_shm),
      tile_w_(tile_w & ~1), tile_h_(tile_h & ~1) {
    for (int i = 0; i < 256; ++i) {
        const float r = kInfernoRgb[i][0], g = kInfernoRgb[i][1], b = kInfernoRgb[i][2];
        lut_y_[i] = clamp_u8(0.299f * r + 0.587f * g + 0.114f * b);
        lut_u_[i] = clamp_u8(128.0f - 0.168736f * r - 0.331264f * g + 0.5f * b);
        lut_v_[i] = clamp_u8(128.0f + 0.5f * r - 0.418688f * g - 0.081312f * b);
    }
    y_.assign(static_cast<size_t>(tile_w_) * tile_h_, 0);          // black...
    uv_.assign(static_cast<size_t>(tile_w_) * (tile_h_ / 2), 0x80);  // ...not green
}

DepthView::~DepthView() {
    if (ring_open_) bat_ring_close(&ring_);
}

bool DepthView::try_open(uint64_t now_ns) {
    if (now_ns < next_open_ns_) return false;
    next_open_ns_ = now_ns + 1000000000ull;  // retry ~1/s until the worker is up
    if (bat_ring_open(&ring_, name_.c_str(), use_shm_) != 0) return false;
    if (ring_.hdr->format != BAT_FMT_F32 ||
        ring_.hdr->width == 0 || ring_.hdr->height == 0) {
        bat_ring_close(&ring_);
        return false;
    }
    // Sized once from the header we validated; a restarted worker declaring a
    // bigger ring then fails read_latest's bounds check instead of overflowing.
    payload_.resize(ring_.hdr->slot_size);
    ring_open_ = true;
    return true;
}

bool DepthView::poll() {
    if (!ring_open_ && !try_open(bat_ring_now_ns())) return false;

    bat_slot_hdr meta;
    if (bat_ring_read_latest(&ring_, payload_.data(), payload_.size(), &meta) != 1)
        return false;
    if (have_frame_ && meta.frame_idx == last_frame_idx_) return false;

    const uint32_t w = ring_.hdr->width, h = ring_.hdr->height;
    if (meta.size != static_cast<uint64_t>(w) * h * 4) return false;

    colorize(reinterpret_cast<const float*>(payload_.data()),
             static_cast<int>(w), static_cast<int>(h));
    last_frame_idx_ = meta.frame_idx;
    have_frame_ = true;
    valid_ = true;
    return true;
}

void DepthView::colorize(const float* depth, int w, int h) {
    // MiDaS gives relative depth (bigger = nearer) on an arbitrary per-frame
    // scale; normalize against a rolling min/max so brightness doesn't pulse.
    float mn = depth[0], mx = depth[0];
    const size_t n = static_cast<size_t>(w) * h;
    for (size_t i = 1; i < n; ++i) {
        const float d = depth[i];
        if (d < mn) mn = d;
        if (d > mx) mx = d;
    }
    if (!std::isfinite(mn) || !std::isfinite(mx)) return;
    if (!norm_init_) {
        lo_ = mn;
        hi_ = mx;
        norm_init_ = true;
    } else {
        constexpr float alpha = 0.15f;
        lo_ += alpha * (mn - lo_);
        hi_ += alpha * (mx - hi_);
    }
    const float scale = 255.0f / std::max(hi_ - lo_, 1e-6f);

    // Source pixel -> colormap index, then nearest-neighbor upscale into the
    // tile (index map first so each source pixel is quantized exactly once).
    std::vector<uint8_t> idx(n);
    for (size_t i = 0; i < n; ++i) {
        const float v = (depth[i] - lo_) * scale;
        idx[i] = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, v)));
    }

    std::vector<int> col_of(tile_w_);
    for (int tx = 0; tx < tile_w_; ++tx)
        col_of[tx] = std::min(tx * w / tile_w_, w - 1);

    for (int ty = 0; ty < tile_h_; ++ty) {
        const int sy = std::min(ty * h / tile_h_, h - 1);
        const uint8_t* src = idx.data() + static_cast<size_t>(sy) * w;
        uint8_t* out = y_.data() + static_cast<size_t>(ty) * tile_w_;
        for (int tx = 0; tx < tile_w_; ++tx) out[tx] = lut_y_[src[col_of[tx]]];
    }

    // Chroma at half resolution: sample the top-left pixel of each 2x2 block.
    for (int ty = 0; ty < tile_h_ / 2; ++ty) {
        const int sy = std::min(2 * ty * h / tile_h_, h - 1);
        const uint8_t* src = idx.data() + static_cast<size_t>(sy) * w;
        uint8_t* out = uv_.data() + static_cast<size_t>(ty) * tile_w_;
        for (int tx = 0; tx < tile_w_ / 2; ++tx) {
            const uint8_t k = src[col_of[2 * tx]];
            out[2 * tx] = lut_u_[k];
            out[2 * tx + 1] = lut_v_[k];
        }
    }
}

Nv12Frame DepthView::view() const {
    Nv12Frame v;
    v.y = y_.data();
    v.uv = uv_.data();
    v.width = tile_w_;
    v.height = tile_h_;
    v.y_stride = tile_w_;
    v.uv_stride = tile_w_;
    return v;
}
