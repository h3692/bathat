// Host unit test for DetView: detections ring -> circle overlay on NV12.
// Uses a file-backed ring (no shared memory, no QNX). Build and run with
//   make -C tests
#include "detview.h"

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

static void write_payload(bat_ring* ring, const bat_det_payload& p, uint32_t size) {
    uint8_t* dst = bat_ring_write_begin(ring);
    std::memcpy(dst, &p, size);
    bat_ring_write_end(ring, size, bat_ring_now_ns());
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::printf("usage: %s <ring-file>\n", argv[0]);
        return 2;
    }
    const char* path = argv[1];
    unlink(path);  // the ring must not exist yet: DetView opens lazily

    DetView view(path, /*use_shm=*/0);
    CHECK(!view.poll(), "poll() is false before the ring exists");
    CHECK(view.dets().empty(), "no detections before the ring exists");

    bat_ring ring;
    CHECK(bat_ring_create(&ring, path, /*use_shm=*/0, BAT_FMT_DET, 256, 256,
                          sizeof(bat_det_payload)) == 0, "create ring");

    bat_det_payload p{};
    p.count = 1;
    p.rec[0] = bat_det_record{0.5f, 0.5f, 0.125f, 0.9f, -10.0f, 0.05f};
    write_payload(&ring, p, sizeof(p));

    // The failed open above armed the ~1 s retry throttle; wait it out.
    CHECK(!view.poll(), "open retry is throttled");
    usleep(1100000);
    CHECK(view.poll(), "poll() picks up the first frame");
    CHECK(view.dets().size() == 1, "one detection");
    CHECK(view.fresh(bat_ring_now_ns()), "fresh right after publishing");
    CHECK(view.fresh(bat_ring_now_ns() + 700ull * 1000 * 1000),
          "still fresh across a slow inference gap");
    CHECK(!view.fresh(bat_ring_now_ns() + 2500ull * 1000 * 1000),
          "stale after 2.5 s");
    CHECK(!view.poll(), "no new frame -> poll() is false");

    // Overlay: a 64x32 tile at origin (16, 8) of an 96x48 destination.
    // Centre lands at (16+32, 8+16), radius = 0.125 * 64 = 8 pixels.
    const int DW = 96, DH = 48;
    std::vector<uint8_t> y(static_cast<size_t>(DW) * DH, 7);
    std::vector<uint8_t> uv(static_cast<size_t>(DW) * (DH / 2), 0x80);
    Nv12Dest dst;
    dst.y = y.data();
    dst.uv = uv.data();
    dst.width = DW;
    dst.height = DH;
    dst.y_stride = DW;
    dst.uv_stride = DW;
    det_draw_circles(view.dets(), dst, 16, 8, 64, 32);

    const int cx = 16 + 32, cy = 8 + 16, r = 8;
    CHECK(y[static_cast<size_t>(cy) * DW + cx + r] == 255, "outer ring is white (right)");
    CHECK(y[static_cast<size_t>(cy) * DW + cx - r] == 255, "outer ring is white (left)");
    CHECK(y[static_cast<size_t>(cy - r) * DW + cx] == 255, "outer ring is white (top)");
    CHECK(y[static_cast<size_t>(cy) * DW + cx + r - 1] == 0, "inner ring is black");
    CHECK(y[static_cast<size_t>(cy) * DW + cx] == 7, "centre pixel untouched");
    CHECK(y[static_cast<size_t>(8) * DW + 16] == 7, "tile corner untouched");

    // A circle poking past the tile edge must clip, not scribble or crash.
    p.rec[0] = bat_det_record{0.02f, 0.5f, 0.25f, 0.9f, -40.0f, 0.05f};
    write_payload(&ring, p, sizeof(p));
    CHECK(view.poll(), "poll() picks up the edge blob");
    det_draw_circles(view.dets(), dst, 16, 8, 64, 32);
    for (int row = 0; row < DH; ++row)
        for (int col = 0; col < 16; ++col)
            if (y[static_cast<size_t>(row) * DW + col] != 7) {
                CHECK(false, "clipped circle leaked left of the tile");
                row = DH;
                break;
            }

    // A count beyond BAT_DET_NMAX is clamped, an undersized payload skipped.
    p.count = 99;
    write_payload(&ring, p, sizeof(p));
    CHECK(view.poll(), "poll() picks up the overfull frame");
    CHECK(view.dets().size() == BAT_DET_NMAX, "count clamps to BAT_DET_NMAX");
    write_payload(&ring, p, sizeof(p) / 2);
    CHECK(!view.poll(), "undersized payload is rejected");
    CHECK(view.dets().size() == BAT_DET_NMAX, "detections unchanged by a bad frame");

    bat_ring_close(&ring);

    if (failures == 0) {
        std::printf("PASS: detview (all checks)\n");
        return 0;
    }
    std::printf("%d check(s) FAILED\n", failures);
    return 1;
}
