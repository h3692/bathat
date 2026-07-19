#include "detview.h"

#include <algorithm>
#include <utility>

DetView::DetView(std::string name, int use_shm)
    : name_(std::move(name)), use_shm_(use_shm) {}

DetView::~DetView() {
    if (ring_open_) bat_ring_close(&ring_);
}

bool DetView::try_open(uint64_t now_ns) {
    if (now_ns < next_open_ns_) return false;
    next_open_ns_ = now_ns + 1000000000ull;  // retry ~1/s until the worker is up
    if (bat_ring_open(&ring_, name_.c_str(), use_shm_) != 0) return false;
    if (ring_.hdr->format != BAT_FMT_DET ||
        ring_.hdr->slot_size < sizeof(bat_det_payload)) {
        bat_ring_close(&ring_);
        return false;
    }
    ring_open_ = true;
    return true;
}

bool DetView::poll() {
    if (!ring_open_ && !try_open(bat_ring_now_ns())) return false;

    bat_slot_hdr meta;
    bat_det_payload p;
    if (bat_ring_read_latest(&ring_, reinterpret_cast<uint8_t*>(&p),
                             sizeof(p), &meta) != 1)
        return false;
    if (have_frame_ && meta.frame_idx == last_frame_idx_) return false;
    if (meta.size != sizeof(bat_det_payload)) return false;

    payload_ = p;
    const uint32_t count = std::min(payload_.count, BAT_DET_NMAX);
    dets_.assign(payload_.rec, payload_.rec + count);
    last_frame_idx_ = meta.frame_idx;
    t_publish_ = meta.t_publish;
    have_frame_ = true;
    return true;
}

namespace {

// One 1-pixel circle via the midpoint algorithm, clipped to the tile at
// (ox, oy) intersected with `dst`.
void draw_circle(Nv12Dest& dst, int ox, int oy, int tile_w, int tile_h,
                 int cx, int cy, int r, uint8_t value) {
    const int x0 = std::max(ox, 0), x1 = std::min(ox + tile_w, dst.width);
    const int y0 = std::max(oy, 0), y1 = std::min(oy + tile_h, dst.height);
    auto plot = [&](int x, int y) {
        if (x >= x0 && x < x1 && y >= y0 && y < y1)
            dst.y[static_cast<size_t>(y) * dst.y_stride + x] = value;
    };
    int x = 0, y = r, d = 1 - r;
    while (x <= y) {
        plot(cx + x, cy + y); plot(cx - x, cy + y);
        plot(cx + x, cy - y); plot(cx - x, cy - y);
        plot(cx + y, cy + x); plot(cx - y, cy + x);
        plot(cx + y, cy - x); plot(cx - y, cy - x);
        if (d < 0) {
            d += 2 * x + 3;
        } else {
            d += 2 * (x - y) + 5;
            --y;
        }
        ++x;
    }
}

}  // namespace

void det_draw_circles(const std::vector<bat_det_record>& dets, Nv12Dest& dst,
                      int ox, int oy, int tile_w, int tile_h) {
    for (const bat_det_record& det : dets) {
        const int cx = ox + static_cast<int>(det.cx * tile_w + 0.5f);
        const int cy = oy + static_cast<int>(det.cy * tile_h + 0.5f);
        const int r = static_cast<int>(det.radius * tile_w + 0.5f);
        if (r < 2) continue;
        draw_circle(dst, ox, oy, tile_w, tile_h, cx, cy, r, 255);
        draw_circle(dst, ox, oy, tile_w, tile_h, cx, cy, r - 1, 0);
    }
}
