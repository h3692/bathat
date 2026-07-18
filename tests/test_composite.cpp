// Host unit test for the pure NV12 side-by-side composite logic.
// No QNX or hardware dependencies: build and run on your dev machine with
//   make -C tests
#include "composite.h"

#include <cstdint>
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

// An NV12 frame backed by owned storage, with deliberately padded strides so
// the test catches stride-handling bugs.
struct OwnedFrame {
    std::vector<uint8_t> ybuf, uvbuf;
    Nv12Frame view;
};

static OwnedFrame make_frame(int w, int h, int y_stride, int uv_stride,
                             uint8_t y_val, uint8_t uv_val) {
    OwnedFrame f;
    f.ybuf.assign(static_cast<size_t>(y_stride) * h, y_val);
    f.uvbuf.assign(static_cast<size_t>(uv_stride) * (h / 2), uv_val);
    f.view.y = f.ybuf.data();
    f.view.uv = f.uvbuf.data();
    f.view.width = w;
    f.view.height = h;
    f.view.y_stride = y_stride;
    f.view.uv_stride = uv_stride;
    return f;
}

int main() {
    const int W = 8, H = 4;  // small, even dimensions

    // Left has padded strides; right has tight strides. Different fill values
    // per plane so we can tell the halves apart.
    OwnedFrame left  = make_frame(W, H, W + 4, W + 4, 0x10, 0x80);
    OwnedFrame right = make_frame(W, H, W,     W,     0x20, 0x90);

    // Destination is 2*W wide with padded strides.
    const int DW = 2 * W, DYS = 2 * W + 6, DUVS = 2 * W + 6;
    std::vector<uint8_t> dy(static_cast<size_t>(DYS) * H, 0);
    std::vector<uint8_t> duv(static_cast<size_t>(DUVS) * (H / 2), 0);

    Nv12Dest dst;
    dst.y = dy.data();
    dst.uv = duv.data();
    dst.width = DW;
    dst.height = H;
    dst.y_stride = DYS;
    dst.uv_stride = DUVS;

    composite_side_by_side(left.view, right.view, dst);

    // Y plane: left half is 0x10, right half is 0x20, on every row.
    for (int r = 0; r < H; ++r) {
        for (int c = 0; c < W; ++c) CHECK(dy[r * DYS + c] == 0x10, "left Y");
        for (int c = 0; c < W; ++c) CHECK(dy[r * DYS + W + c] == 0x20, "right Y");
    }

    // UV plane: half height, same left/right split.
    for (int r = 0; r < H / 2; ++r) {
        for (int c = 0; c < W; ++c) CHECK(duv[r * DUVS + c] == 0x80, "left UV");
        for (int c = 0; c < W; ++c) CHECK(duv[r * DUVS + W + c] == 0x90, "right UV");
    }

    // Stride padding beyond the composited width must be left untouched.
    for (int r = 0; r < H; ++r)
        for (int c = DW; c < DYS; ++c)
            CHECK(dy[r * DYS + c] == 0, "Y stride padding untouched");

    if (failures == 0) {
        std::printf("PASS: composite_side_by_side (all checks)\n");
        return 0;
    }
    std::printf("%d check(s) FAILED\n", failures);
    return 1;
}
