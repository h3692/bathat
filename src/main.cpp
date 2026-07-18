#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <unistd.h>

#include "camera.h"
#include "composite.h"
#include "display.h"
#include "frameslot.h"

namespace {

volatile sig_atomic_t g_run = 1;
void on_signal(int) { g_run = 0; }

void print_usage(const char* prog) {
    std::fprintf(stderr,
                 "usage: %s [--probe] [--ev N] [--width W] [--height H]\n"
                 "  --probe     print camera capabilities and exit\n"
                 "  --ev N      exposure bias, higher is brighter (default 1.0)\n"
                 "  --width W   per-camera width  (default 640)\n"
                 "  --height H  per-camera height (default 480)\n",
                 prog);
}

}  // namespace

int main(int argc, char** argv) {
    int width = 640;
    int height = 480;
    double ev_bias = 1.0;  // positive => brighter
    bool probe = false;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--probe") == 0) {
            probe = true;
        } else if (std::strcmp(argv[i], "--ev") == 0 && i + 1 < argc) {
            ev_bias = std::atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--width") == 0 && i + 1 < argc) {
            width = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--height") == 0 && i + 1 < argc) {
            height = std::atoi(argv[++i]);
        } else {
            print_usage(argv[0]);
            return 2;
        }
    }

    if (probe) return camera_probe_all();

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    // Enumerate cameras up front so we know how many feeds to show.
    camera_unit_t units[8];
    unsigned n = 0;
    if (camera_get_supported_cameras(8, &n, units) != CAMERA_EOK || n == 0) {
        std::fprintf(stderr,
                     "error: no cameras reported. Is the sensor service running?\n"
                     "       check with: pidin ar | grep sensor\n");
        return 1;
    }
    std::printf("found %u camera(s)\n", n);

    FrameSlot slot_a, slot_b;
    CameraStream cam_a, cam_b;

    const bool have_a = cam_a.start(units[0], width, height, ev_bias, slot_a);
    if (!have_a) {
        std::fprintf(stderr, "error: failed to start camera A\n");
        return 1;
    }

    bool have_b = false;
    if (n >= 2) {
        have_b = cam_b.start(units[1], width, height, ev_bias, slot_b);
        if (!have_b)
            std::fprintf(stderr, "warn: second camera failed to start; showing single feed\n");
    } else {
        std::fprintf(stderr, "warn: only one camera found; showing single feed\n");
    }

    const int disp_w = have_b ? 2 * width : width;
    Display disp;
    if (!disp.init(disp_w, height)) {
        std::fprintf(stderr, "error: display init failed\n");
        return 1;
    }

    std::printf("streaming... press Ctrl-C to quit\n");
    while (g_run) {
        Nv12Dest dst;
        if (disp.begin_frame(dst)) {
            if (have_b) {
                std::scoped_lock lk(slot_a.mutex(), slot_b.mutex());
                if (slot_a.valid() && slot_b.valid())
                    composite_side_by_side(slot_a.view(), slot_b.view(), dst);
            } else {
                std::scoped_lock lk(slot_a.mutex());
                if (slot_a.valid()) composite_place(slot_a.view(), dst, 0);
            }
            disp.end_frame();
        }
        usleep(15000);  // ~60 Hz render sampling; frames arrive asynchronously
    }

    std::printf("\nshutting down\n");
    cam_a.stop();
    cam_b.stop();
    disp.shutdown();
    return 0;
}
