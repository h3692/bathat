#include "composite.h"

#include <algorithm>
#include <cstring>

void composite_place(const Nv12Frame& src, Nv12Dest& dst, int x) {
    if (!src.y || !src.uv || !dst.y || !dst.uv) return;
    x &= ~1;                            // even offset keeps U,V pairs aligned
    if (x < 0 || x >= dst.width) return;

    const int w = std::min(src.width, dst.width - x);
    const int h = std::min(src.height, dst.height);
    if (w <= 0 || h <= 0) return;

    // Y plane: h rows of w bytes.
    for (int row = 0; row < h; ++row) {
        std::memcpy(dst.y + static_cast<size_t>(row) * dst.y_stride + x,
                    src.y + static_cast<size_t>(row) * src.y_stride,
                    static_cast<size_t>(w));
    }

    // UV plane: h/2 rows. NV12 chroma is half-height but full-width in bytes
    // (each row holds w/2 interleaved U,V pairs = w bytes), so the horizontal
    // offset and copy width match the Y plane.
    const int uv_rows = h / 2;
    for (int row = 0; row < uv_rows; ++row) {
        std::memcpy(dst.uv + static_cast<size_t>(row) * dst.uv_stride + x,
                    src.uv + static_cast<size_t>(row) * src.uv_stride,
                    static_cast<size_t>(w));
    }
}

void composite_side_by_side(const Nv12Frame& left, const Nv12Frame& right, Nv12Dest& dst) {
    composite_place(left, dst, 0);
    composite_place(right, dst, left.width);
}
