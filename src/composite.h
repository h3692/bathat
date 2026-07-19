#ifndef BATHAT_COMPOSITE_H
#define BATHAT_COMPOSITE_H

#include <cstdint>

// A read-only view of an NV12 image: a full-resolution Y (luma) plane of
// `height` rows, followed by a half-height interleaved UV (chroma) plane of
// `height/2` rows. `width` and `height` must be even. Strides are the number
// of bytes between successive rows and may exceed `width` (padding).
struct Nv12Frame {
    const uint8_t* y   = nullptr;   // Y plane
    const uint8_t* uv  = nullptr;   // interleaved U,V plane
    int width          = 0;
    int height         = 0;
    int y_stride       = 0;         // bytes per Y row  (>= width)
    int uv_stride      = 0;         // bytes per UV row (>= width)
};

// A writable NV12 destination, e.g. a Screen back buffer.
struct Nv12Dest {
    uint8_t* y    = nullptr;
    uint8_t* uv   = nullptr;
    int width     = 0;              // full destination width
    int height    = 0;
    int y_stride  = 0;
    int uv_stride = 0;
};

// Copy one NV12 frame into `dst` at pixel offset (`x`, `y`), each forced even
// for chroma alignment. The copy is clipped to what fits, so a size mismatch
// truncates rather than overflowing the destination.
void composite_place(const Nv12Frame& src, Nv12Dest& dst, int x, int y = 0);

// Place `left` at x=0 and `right` immediately to its right (x = left.width).
void composite_side_by_side(const Nv12Frame& left, const Nv12Frame& right, Nv12Dest& dst);

#endif  // BATHAT_COMPOSITE_H
