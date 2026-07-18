#include "camera.h"

#include <camera/camera_3a.h>

#include <cmath>
#include <cstdio>
#include <vector>

// ---------------------------------------------------------------------------
// Buffer layout extraction.
//
// Per the QNX docs, a viewfinder buffer's data is valid ONLY during the
// callback, so we copy it out immediately (FrameSlot::write). This function is
// the single place that touches camera_buffer_t internals.
//
// VERIFY against <camera/camera_api.h> in your QNX 8.0 SDP: the field names
// below (framebuf, framedesc.nv12, .width/.height/.stride/.uv_offset) are the
// standard QNX names, but if your header differs this is the only spot to fix.
// ---------------------------------------------------------------------------
static bool extract_nv12(const camera_buffer_t& buf, Nv12Frame& out) {
    if (buf.frametype != CAMERA_FRAMETYPE_NV12) return false;

    const camera_frame_nv12_t& d = buf.framedesc.nv12;
    uint8_t* base = reinterpret_cast<uint8_t*>(buf.framebuf);
    if (!base) return false;

    out.y = base;
    out.uv = base + d.uv_offset;
    out.width = static_cast<int>(d.width);
    out.height = static_cast<int>(d.height);
    out.y_stride = static_cast<int>(d.stride);
    out.uv_stride = static_cast<int>(d.stride);  // NV12: UV stride == Y stride
    return out.width > 0 && out.height > 0;
}

static void vf_callback(camera_handle_t /*handle*/, camera_buffer_t* buf, void* arg) {
    FrameSlot* slot = static_cast<FrameSlot*>(arg);
    if (!slot || !buf) return;

    Nv12Frame f;
    if (extract_nv12(*buf, f)) slot->write(f);
}

// Prefer auto-exposure with a positive EV bias: brighter, but adapts to
// changing light and keeps the shutter short (no motion blur on a moving
// navigation camera). Falls back to plain auto-exposure if EV offset is
// unsupported on this sensor.
static void configure_exposure(camera_handle_t h, double ev_bias) {
    if (camera_set_exposure_mode(h, CAMERA_EXPOSUREMODE_AUTO) != CAMERA_EOK)
        std::fprintf(stderr, "  warn: could not set auto-exposure mode\n");

    unsigned num = 0;
    bool maxmin = false;
    if (camera_get_supported_ev_offsets(h, 0, &num, NULL, &maxmin) != CAMERA_EOK || num == 0) {
        std::fprintf(stderr, "  warn: EV offset unsupported; using plain auto-exposure\n");
        return;
    }

    std::vector<double> offs(num);
    if (camera_get_supported_ev_offsets(h, num, &num, offs.data(), &maxmin) != CAMERA_EOK) {
        std::fprintf(stderr, "  warn: could not read EV offsets; using plain auto-exposure\n");
        return;
    }

    double chosen;
    if (maxmin) {
        // offs holds {min, max}: clamp the requested bias into range.
        const double lo = offs.front(), hi = offs.back();
        chosen = ev_bias < lo ? lo : (ev_bias > hi ? hi : ev_bias);
    } else {
        // Discrete list: pick the supported value closest to the request.
        chosen = offs[0];
        double best = std::fabs(offs[0] - ev_bias);
        for (double o : offs) {
            const double dist = std::fabs(o - ev_bias);
            if (dist < best) { best = dist; chosen = o; }
        }
    }

    if (camera_set_ev_offset(h, chosen) == CAMERA_EOK)
        std::printf("  exposure: auto + EV offset %.2f\n", chosen);
    else
        std::fprintf(stderr, "  warn: failed to set EV offset %.2f\n", chosen);
}

bool CameraStream::start(camera_unit_t unit, int width, int height, double ev_bias,
                         FrameSlot& slot) {
    // VERIFY open mode against your SDP; CAMERA_MODE_RW is the usual choice for
    // configuring and streaming a viewfinder.
    camera_error_t err = camera_open(unit, CAMERA_MODE_RW, &handle_);
    if (err != CAMERA_EOK) {
        std::fprintf(stderr, "camera_open(unit %d) failed: %d\n", static_cast<int>(unit), err);
        return false;
    }
    unit_ = unit;

    err = camera_set_vf_property(handle_,
                                 CAMERA_IMGPROP_FORMAT, CAMERA_FRAMETYPE_NV12,
                                 CAMERA_IMGPROP_WIDTH, static_cast<uint32_t>(width),
                                 CAMERA_IMGPROP_HEIGHT, static_cast<uint32_t>(height));
    if (err != CAMERA_EOK) {
        std::fprintf(stderr, "camera_set_vf_property(unit %d) failed: %d\n",
                     static_cast<int>(unit), err);
        camera_close(handle_);
        handle_ = CAMERA_HANDLE_INVALID;
        return false;
    }

    configure_exposure(handle_, ev_bias);

    err = camera_start_viewfinder(handle_, &vf_callback, NULL, &slot);
    if (err != CAMERA_EOK) {
        std::fprintf(stderr, "camera_start_viewfinder(unit %d) failed: %d\n",
                     static_cast<int>(unit), err);
        camera_close(handle_);
        handle_ = CAMERA_HANDLE_INVALID;
        return false;
    }

    streaming_ = true;
    std::printf("camera unit %d streaming NV12 %dx%d\n", static_cast<int>(unit), width, height);
    return true;
}

void CameraStream::stop() {
    if (streaming_) {
        camera_stop_viewfinder(handle_);
        streaming_ = false;
    }
    if (handle_ != CAMERA_HANDLE_INVALID) {
        camera_close(handle_);
        handle_ = CAMERA_HANDLE_INVALID;
    }
}

CameraStream::~CameraStream() { stop(); }

// ---------------------------------------------------------------------------
// Capability probe.
// ---------------------------------------------------------------------------
static void probe_one(camera_unit_t unit) {
    camera_handle_t h = CAMERA_HANDLE_INVALID;
    if (camera_open(unit, CAMERA_MODE_RW, &h) != CAMERA_EOK) {
        std::printf("  unit %d: could not open\n", static_cast<int>(unit));
        return;
    }
    std::printf("  unit %d opened\n", static_cast<int>(unit));

    unsigned n = 0;
    if (camera_get_supported_vf_frame_types(h, 0, &n, NULL) == CAMERA_EOK && n > 0) {
        std::vector<camera_frametype_t> types(n);
        if (camera_get_supported_vf_frame_types(h, n, &n, types.data()) == CAMERA_EOK) {
            std::printf("    vf frame types:");
            for (unsigned i = 0; i < n; ++i) std::printf(" %d", static_cast<int>(types[i]));
            std::printf("   (NV12 == %d)\n", static_cast<int>(CAMERA_FRAMETYPE_NV12));
        }
    }

    n = 0;
    if (camera_get_supported_vf_resolutions(h, 0, &n, NULL) == CAMERA_EOK && n > 0) {
        std::vector<camera_res_t> res(n);
        if (camera_get_supported_vf_resolutions(h, n, &n, res.data()) == CAMERA_EOK) {
            std::printf("    vf resolutions:");
            for (unsigned i = 0; i < n; ++i)
                std::printf(" %ux%u", res[i].width, res[i].height);
            std::printf("\n");
        }
    }

    n = 0;
    bool maxmin = false;
    if (camera_get_supported_ev_offsets(h, 0, &n, NULL, &maxmin) == CAMERA_EOK && n > 0) {
        std::vector<double> offs(n);
        if (camera_get_supported_ev_offsets(h, n, &n, offs.data(), &maxmin) == CAMERA_EOK) {
            std::printf("    ev offsets (%s):", maxmin ? "range min/max" : "discrete");
            for (unsigned i = 0; i < n; ++i) std::printf(" %.2f", offs[i]);
            std::printf("\n");
        }
    } else {
        std::printf("    ev offsets: unsupported\n");
    }

    std::printf("    manual-exposure feature: %s\n",
                camera_can_feature(h, CAMERA_FEATURE_MANUALEXPOSURE) ? "yes" : "no");

    camera_close(h);
}

int camera_probe_all() {
    camera_unit_t units[16];
    unsigned n = 0;
    if (camera_get_supported_cameras(16, &n, units) != CAMERA_EOK) {
        std::fprintf(stderr,
                     "camera_get_supported_cameras failed. Is the sensor service running?\n");
        return 1;
    }
    std::printf("cameras reported: %u\n", n);
    for (unsigned i = 0; i < n; ++i) probe_one(units[i]);
    return 0;
}
