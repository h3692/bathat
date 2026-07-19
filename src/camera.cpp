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

// Pick a supported manual ISO closest to `want`, clamped to the sensor's range.
// Per the header, when the range flag is set, values[0] is the MAX and [1] the MIN.
static unsigned pick_iso(camera_handle_t h, unsigned want) {
    unsigned num = 0;
    bool is_range = false;
    if (camera_get_supported_manual_iso_values(h, 0, &num, NULL, &is_range) != CAMERA_EOK || num == 0) {
        std::fprintf(stderr, "  warn: no supported ISO values reported\n");
        return want;
    }
    std::vector<unsigned> v(num);
    if (camera_get_supported_manual_iso_values(h, num, &num, v.data(), &is_range) != CAMERA_EOK)
        return want;

    if (is_range) {
        const unsigned mx = v[0], mn = v[1];
        std::printf("  ISO range: %u..%u\n", mn, mx);
        return want < mn ? mn : (want > mx ? mx : want);
    }
    unsigned chosen = v[0];
    double best = std::fabs(double(v[0]) - double(want));
    std::printf("  ISO options:");
    for (unsigned x : v) {
        std::printf(" %u", x);
        const double d = std::fabs(double(x) - double(want));
        if (d < best) { best = d; chosen = x; }
    }
    std::printf("\n");
    return chosen;
}

// Pick a supported manual shutter speed (in seconds) closest to `want`.
static double pick_shutter(camera_handle_t h, double want) {
    unsigned num = 0;
    bool is_range = false;
    if (camera_get_supported_manual_shutter_speeds(h, 0, &num, NULL, &is_range) != CAMERA_EOK || num == 0) {
        std::fprintf(stderr, "  warn: no supported shutter speeds reported\n");
        return want;
    }
    std::vector<double> v(num);
    if (camera_get_supported_manual_shutter_speeds(h, num, &num, v.data(), &is_range) != CAMERA_EOK)
        return want;

    if (is_range) {
        const double mx = v[0], mn = v[1];
        std::printf("  shutter range: %.5f..%.5f s\n", mn, mx);
        return want < mn ? mn : (want > mx ? mx : want);
    }
    double chosen = v[0];
    double best = std::fabs(v[0] - want);
    std::printf("  shutter options:");
    for (double x : v) {
        std::printf(" %.5f", x);
        const double d = std::fabs(x - want);
        if (d < best) { best = d; chosen = x; }
    }
    std::printf("\n");
    return chosen;
}

// Make the image look natural and bright:
//  - auto white balance removes the yellow/green colour cast, and
//  - full manual exposure (ISO + shutter) forces a brighter result than the
//    auto-metered target, since EV offset is unsupported on this sensor.
static void configure_3a(camera_handle_t h, unsigned iso, double shutter) {
    if (camera_set_whitebalance_mode(h, CAMERA_WHITEBALANCEMODE_AUTO) == CAMERA_EOK)
        std::printf("  white balance: auto\n");
    else
        std::fprintf(stderr, "  warn: could not set auto white balance\n");

    if (camera_set_exposure_mode(h, CAMERA_EXPOSUREMODE_MANUAL) != CAMERA_EOK) {
        std::fprintf(stderr, "  warn: could not set manual exposure; leaving default\n");
        return;
    }

    const unsigned use_iso = pick_iso(h, iso);
    if (camera_set_manual_iso(h, use_iso) == CAMERA_EOK)
        std::printf("  ISO: %u\n", use_iso);
    else
        std::fprintf(stderr, "  warn: failed to set ISO %u\n", use_iso);

    const double use_sh = pick_shutter(h, shutter);
    if (camera_set_manual_shutter_speed(h, use_sh) == CAMERA_EOK)
        std::printf("  shutter: %.5fs (~1/%.0f)\n", use_sh, use_sh > 0 ? 1.0 / use_sh : 0.0);
    else
        std::fprintf(stderr, "  warn: failed to set shutter %.5f\n", use_sh);
}

bool CameraStream::start(camera_unit_t unit, int width, int height, unsigned iso,
                         double shutter, FrameSlot& slot) {
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

    err = camera_start_viewfinder(handle_, &vf_callback, NULL, &slot);
    if (err != CAMERA_EOK) {
        std::fprintf(stderr, "camera_start_viewfinder(unit %d) failed: %d\n",
                     static_cast<int>(unit), err);
        camera_close(handle_);
        handle_ = CAMERA_HANDLE_INVALID;
        return false;
    }

    // 3A (white balance + exposure) applies while the viewfinder is running.
    configure_3a(handle_, iso, shutter);

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
