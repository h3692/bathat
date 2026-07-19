#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <unistd.h>

#include "camera.h"
#include "composite.h"
#include "depthview.h"
#include "display.h"
#include "frameslot.h"

namespace {

volatile sig_atomic_t g_run = 1;
void on_signal(int) { g_run = 0; }

void print_usage(const char* prog) {
    std::fprintf(stderr,
                 "usage: %s [--probe] [--no-display] [--no-depth] [--iso N] [--shutter S] [--width W] [--height H]\n"
                 "  --probe       print camera capabilities and exit\n"
                 "  --no-display  headless capture daemon: publish frames to shared memory only\n"
                 "  --no-depth    viewfinder shows cameras only, without the live depth tiles\n"
                 "  --iso N       manual ISO/gain, higher is brighter (default 1600)\n"
                 "  --shutter S   manual shutter in seconds, longer is brighter (default 0.0666 = ~1/15)\n"
                 "  --width W     per-camera width  (default 1536; must be a supported mode)\n"
                 "  --height H    per-camera height (default 864; see --probe)\n"
                 "\n"
                 "Frames are always published to the shared-memory rings /bat_cam0 and\n"
                 "/bat_cam1 (visible as /dev/shmem/bat_cam*) for the depth worker to read.\n"
                 "The viewfinder also reads /bat_depth0 and /bat_depth1 back (when the\n"
                 "depth worker is running) and shows each colorized depth map beside or\n"
                 "below its camera; bright/warm = near.\n",
                 prog);
}

}  // namespace

int main(int argc, char** argv) {
    // IMX708 on QNX only offers 2304x1296 and 1536x864 for the viewfinder
    // (confirmed via --probe); default to the smaller/faster mode.
    int width = 1536;
    int height = 864;
    unsigned iso = 1600;      // higher => brighter (more gain/noise)
    double shutter = 0.0666;  // seconds (~1/15); longer => brighter (more motion blur)
    bool probe = false;
    bool no_display = false;
    bool no_depth = false;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--probe") == 0) {
            probe = true;
        } else if (std::strcmp(argv[i], "--no-display") == 0) {
            no_display = true;
        } else if (std::strcmp(argv[i], "--no-depth") == 0) {
            no_depth = true;
        } else if (std::strcmp(argv[i], "--iso") == 0 && i + 1 < argc) {
            iso = static_cast<unsigned>(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--shutter") == 0 && i + 1 < argc) {
            shutter = std::atof(argv[++i]);
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

    // Shared-memory rings the depth worker reads. Created before the cameras
    // start so no frame is ever dropped for lack of a ring. If creation fails
    // the viewfinder still works; the depth pipeline just sees nothing.
    const uint32_t slot_size = static_cast<uint32_t>(width) * height * 3 / 2;
    bat_ring ring_a, ring_b;
    const bool ring_a_ok = bat_ring_create(&ring_a, "/bat_cam0", /*use_shm=*/1, BAT_FMT_NV12,
                                           width, height, slot_size) == 0;
    const bool ring_b_ok = n >= 2 && bat_ring_create(&ring_b, "/bat_cam1", /*use_shm=*/1,
                                                     BAT_FMT_NV12, width, height, slot_size) == 0;
    if (!ring_a_ok)
        std::fprintf(stderr, "warn: could not create shared-memory ring /bat_cam0; "
                             "depth worker will see no frames\n");

    FrameSlot slot_a, slot_b;
    FrameSink sink_a{no_display ? nullptr : &slot_a, ring_a_ok ? &ring_a : nullptr};
    FrameSink sink_b{no_display ? nullptr : &slot_b, ring_b_ok ? &ring_b : nullptr};
    CameraStream cam_a, cam_b;

    const bool have_a = cam_a.start(units[0], width, height, iso, shutter, sink_a);
    if (!have_a) {
        std::fprintf(stderr, "error: failed to start camera A\n");
        return 1;
    }

    bool have_b = false;
    if (n >= 2) {
        have_b = cam_b.start(units[1], width, height, iso, shutter, sink_b);
        if (!have_b)
            std::fprintf(stderr, "warn: second camera failed to start; showing single feed\n");
    } else {
        std::fprintf(stderr, "warn: only one camera found; showing single feed\n");
    }

    // Layout: one camera shows [cam0 | depth0]; two cameras stack the depth
    // tiles under their cameras ([cam0 | cam1] over [depth0 | depth1]).
    const bool show_depth = !no_display && !no_depth;
    Display disp;
    if (!no_display) {
        const int disp_w = (have_b || show_depth) ? 2 * width : width;
        const int disp_h = (have_b && show_depth) ? 2 * height : height;
        if (!disp.init(disp_w, disp_h)) {
            std::fprintf(stderr, "error: display init failed\n");
            return 1;
        }
    }

    // Depth tiles read back what the worker publishes; the rings appear only
    // once depth_worker.py starts, so the views keep retrying until then.
    std::unique_ptr<DepthView> depth_a, depth_b;
    if (show_depth) {
        depth_a = std::make_unique<DepthView>("/bat_depth0", /*use_shm=*/1, width, height);
        if (have_b)
            depth_b = std::make_unique<DepthView>("/bat_depth1", /*use_shm=*/1, width, height);
    }

    std::printf("streaming%s... press Ctrl-C to quit\n", no_display ? " (headless)" : "");
    uint64_t last_stats = bat_ring_now_ns();
    uint64_t last_count_a = 0, last_count_b = 0;
    uint64_t last_depth_a = 0, last_depth_b = 0;
    while (g_run) {
        if (!no_display) {
            if (depth_a) depth_a->poll();
            if (depth_b) depth_b->poll();
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
                if (depth_a && depth_a->valid())
                    composite_place(depth_a->view(), dst,
                                    have_b ? 0 : width, have_b ? height : 0);
                if (depth_b && depth_b->valid())
                    composite_place(depth_b->view(), dst, width, height);
                disp.end_frame();
            }
        }

        // A capture-rate line every ~2 s, from the rings' write counters.
        const uint64_t now = bat_ring_now_ns();
        const double dt = (now - last_stats) * 1e-9;
        if (dt >= 2.0) {
            bool printed = false;
            if (ring_a_ok) {
                const uint64_t ca = ring_a.hdr->wr_count;
                std::printf("cam0: %5.1f fps", (ca - last_count_a) / dt);
                last_count_a = ca;
                printed = true;
                if (ring_b_ok) {
                    const uint64_t cb = ring_b.hdr->wr_count;
                    std::printf("   cam1: %5.1f fps", (cb - last_count_b) / dt);
                    last_count_b = cb;
                }
            }
            // Depth counters live in the worker's rings, which reset if it
            // restarts -- clamp a backwards jump to 0 instead of a bogus rate.
            if (depth_a) {
                const uint64_t da = depth_a->wr_count();
                std::printf("   depth0: %4.1f fps",
                            da >= last_depth_a ? (da - last_depth_a) / dt : 0.0);
                last_depth_a = da;
                printed = true;
            }
            if (depth_b) {
                const uint64_t db = depth_b->wr_count();
                std::printf("   depth1: %4.1f fps",
                            db >= last_depth_b ? (db - last_depth_b) / dt : 0.0);
                last_depth_b = db;
            }
            if (printed) std::printf("\n");
            last_stats = now;
        }
        usleep(no_display ? 100000 : 15000);  // display path samples at ~60 Hz
    }

    std::printf("\nshutting down\n");
    cam_a.stop();
    cam_b.stop();
    if (!no_display) disp.shutdown();
    if (ring_a_ok) bat_ring_close(&ring_a);
    if (ring_b_ok) bat_ring_close(&ring_b);
    return 0;
}
