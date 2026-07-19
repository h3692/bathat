// Host unit test for DepthView: float32 depth ring -> colorized NV12 tile.
// Uses a file-backed ring (no shared memory, no QNX). Build and run with
//   make -C tests
#include "depthview.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <vector>

#include "bat_ring.h"
#include "inferno_lut.h"

static int failures = 0;

#define CHECK(cond, msg)                                                        \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::printf("FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__);       \
            ++failures;                                                         \
        }                                                                       \
    } while (0)

// Source depth frame and destination tile: a clean 2x upscale, so every
// source pixel maps to one 2x2 tile block.
static const int W = 8, H = 4, TW = 16, TH = 8;

// Same RGB->YUV math as DepthView, applied to the shared inferno table, so
// the test can predict tile colors from a colormap index (+-1 for rounding).
static uint8_t expect_y(int idx) {
    const float r = kInfernoRgb[idx][0], g = kInfernoRgb[idx][1], b = kInfernoRgb[idx][2];
    return static_cast<uint8_t>(0.299f * r + 0.587f * g + 0.114f * b + 0.5f);
}

static void write_frame(bat_ring* ring, const float* vals, uint32_t size) {
    uint8_t* dst = bat_ring_write_begin(ring);
    std::memcpy(dst, vals, size);
    bat_ring_write_end(ring, size, bat_ring_now_ns());
}

static uint8_t tile_y(const Nv12Frame& t, int x, int y) {
    return t.y[static_cast<size_t>(y) * t.y_stride + x];
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::printf("usage: %s <ring-file>\n", argv[0]);
        return 2;
    }
    const char* path = argv[1];
    unlink(path);  // the ring must not exist yet: DepthView opens lazily

    DepthView view(path, /*use_shm=*/0, TW, TH);
    CHECK(!view.poll(), "poll() is false before the ring exists");
    CHECK(!view.valid(), "no tile before the ring exists");
    CHECK(view.wr_count() == 0, "wr_count 0 before the ring exists");

    bat_ring ring;
    CHECK(bat_ring_create(&ring, path, /*use_shm=*/0, BAT_FMT_F32, W, H,
                          W * H * sizeof(float)) == 0, "create ring");

    // Frame 1: row-major gradient 0..31, so bigger (= nearer) is later.
    float grad[W * H];
    for (int i = 0; i < W * H; ++i) grad[i] = static_cast<float>(i);
    write_frame(&ring, grad, sizeof(grad));

    // The failed open above armed the ~1 s retry throttle; wait it out.
    CHECK(!view.poll(), "open retry is throttled");
    usleep(1100000);
    CHECK(view.poll(), "poll() picks up frame 1 after the ring appears");
    CHECK(view.valid(), "tile valid after frame 1");
    CHECK(view.wr_count() == 1, "wr_count follows the writer");

    Nv12Frame t = view.view();
    CHECK(t.width == TW && t.height == TH, "tile has the requested size");
    CHECK(t.y_stride == TW && t.uv_stride == TW, "tile is tight NV12");

    // First frame is normalized to its own min/max: far end maps to colormap
    // index 0 (near-black), near end to 255 (bright yellow-white).
    CHECK(tile_y(t, 0, 0) <= expect_y(0) + 1, "far pixel is dark");
    CHECK(tile_y(t, TW - 1, TH - 1) + 1 >= expect_y(255), "near pixel is bright");
    // Nearest-neighbor 2x upscale: each 2x2 block is uniform.
    for (int by = 0; by < 2; ++by)
        for (int bx = 0; bx < 2; ++bx) {
            const uint8_t v = tile_y(t, 4 + bx * 2, 2 + by * 2);
            CHECK(tile_y(t, 4 + bx * 2 + 1, 2 + by * 2) == v &&
                  tile_y(t, 4 + bx * 2, 2 + by * 2 + 1) == v,
                  "2x2 tile block maps to one source pixel");
        }
    CHECK(!view.poll(), "no new frame -> poll() is false");

    // Frame 2: same shape, 10x the scale. Rolling normalization only tracks
    // 15% of the jump, so everything beyond the smoothed max clamps to the
    // same "near" color -- with per-frame normalization these would differ.
    float grad10[W * H];
    for (int i = 0; i < W * H; ++i) grad10[i] = 10.0f * i;
    write_frame(&ring, grad10, sizeof(grad10));
    CHECK(view.poll(), "poll() picks up frame 2");
    t = view.view();
    // hi_ = 31 + 0.15*(310-31) ~= 72.9, so source values 80..310 all clamp.
    const uint8_t near_y = tile_y(t, TW - 1, TH - 1);        // source i=31, v=310
    CHECK(tile_y(t, 0, 2) == near_y, "mid value clamps like max (rolling norm)");  // i=8, v=80
    CHECK(near_y + 1 >= expect_y(255), "clamped pixels are the near color");
    CHECK(tile_y(t, 0, 0) <= expect_y(0) + 1, "far pixel still dark");

    // Frame 3: wrong payload size (not W*H floats) must be skipped whole.
    std::vector<uint8_t> before(t.y, t.y + static_cast<size_t>(TW) * TH);
    write_frame(&ring, grad, sizeof(grad) / 2);
    CHECK(!view.poll(), "undersized frame is rejected");
    CHECK(view.valid(), "tile stays valid after a bad frame");
    t = view.view();
    CHECK(std::memcmp(before.data(), t.y, before.size()) == 0,
          "tile unchanged by a bad frame");

    bat_ring_close(&ring);

    if (failures == 0) {
        std::printf("PASS: depthview (all checks)\n");
        return 0;
    }
    std::printf("%d check(s) FAILED\n", failures);
    return 1;
}
