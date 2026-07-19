// Host unit test for RectView: BGR ring -> NV12 tile. Uses a file-backed ring
// (no shared memory, no QNX). Build and run with:  make -C tests
#include "rectview.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <vector>

#include "bat_ring.h"

static int failures = 0;

#define CHECK(cond, msg)                                                        \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::printf("FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__);       \
            ++failures;                                                         \
        }                                                                       \
    } while (0)

// 8x4 source, 16x8 tile -> a clean 2x upscale.
static const int W = 8, H = 4, TW = 16, TH = 8;

// Same BT.601 luma as RectView, so the test can predict tile Y from BGR.
static uint8_t luma(uint8_t b, uint8_t g, uint8_t r) {
    const float y = 0.299f * r + 0.587f * g + 0.114f * b + 0.5f;
    return static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, y)));
}

static void write_bgr(bat_ring* ring, const uint8_t* px, uint32_t size) {
    uint8_t* dst = bat_ring_write_begin(ring);
    std::memcpy(dst, px, size);
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
    unlink(path);  // the ring must not exist yet: RectView opens lazily

    RectView view(path, /*use_shm=*/0, TW, TH);
    CHECK(!view.poll(), "poll() is false before the ring exists");
    CHECK(!view.valid(), "no tile before the ring exists");

    bat_ring ring;
    CHECK(bat_ring_create(&ring, path, /*use_shm=*/0, BAT_FMT_BGR8, W, H, W * H * 3) == 0,
          "create ring");

    // Mid-grey everywhere, one bright-red pixel at source (0,0). BGR order.
    std::vector<uint8_t> frame(static_cast<size_t>(W) * H * 3, 100);  // B=G=R=100
    frame[0] = 0; frame[1] = 0; frame[2] = 255;                       // (0,0) = red
    write_bgr(&ring, frame.data(), static_cast<uint32_t>(frame.size()));

    // The failed open above armed the ~1 s retry throttle; wait it out.
    CHECK(!view.poll(), "open retry is throttled");
    usleep(1100000);
    CHECK(view.poll(), "poll() picks up the frame after the ring appears");
    CHECK(view.valid(), "tile valid after a frame");

    Nv12Frame t = view.view();
    CHECK(t.width == TW && t.height == TH, "tile has the requested size");
    CHECK(t.y_stride == TW && t.uv_stride == TW, "tile is tight NV12");

    // Grey area -> luma(100,100,100); the top-left 2x2 block -> the red pixel.
    CHECK(tile_y(t, TW - 1, TH - 1) == luma(100, 100, 100), "grey area luma");
    CHECK(tile_y(t, 0, 0) == luma(0, 0, 255), "red pixel luma (top-left)");
    CHECK(tile_y(t, 1, 0) == tile_y(t, 0, 0) && tile_y(t, 0, 1) == tile_y(t, 0, 0),
          "2x2 tile block maps to one source pixel");
    CHECK(!view.poll(), "no new frame -> poll() is false");

    // Wrong payload size (not W*H*3) must be skipped whole.
    std::vector<uint8_t> before(t.y, t.y + static_cast<size_t>(TW) * TH);
    write_bgr(&ring, frame.data(), static_cast<uint32_t>(frame.size()) / 2);
    CHECK(!view.poll(), "undersized frame is rejected");
    CHECK(view.valid(), "tile stays valid after a bad frame");
    t = view.view();
    CHECK(std::memcmp(before.data(), t.y, before.size()) == 0,
          "tile unchanged by a bad frame");

    bat_ring_close(&ring);

    if (failures == 0) {
        std::printf("PASS: rectview (all checks)\n");
        return 0;
    }
    std::printf("%d check(s) FAILED\n", failures);
    return 1;
}
