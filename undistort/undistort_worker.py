#!/usr/bin/env python3
"""Undistort worker: raw camera frames in, fisheye-rectified frames out.

Reads NV12 frames from the capture daemon's rings (/bat_cam0, /bat_cam1),
converts to BGR, applies each camera's fisheye undistortion (calib/camN.yml,
from tools/calibrate.py), and republishes the rectified BGR frame to
/bat_rect0, /bat_rect1. Downstream stages (depth worker, view compositor) read
the rectified rings, so the whole pipeline works in one undistorted space.

Like the depth worker it free-runs: always the newest frame, never queues. With
no calibration yet a camera passes through unrectified (warned once), so the
pipeline still runs before you calibrate.

On the Pi (after starting the capture daemon: ./bathat --no-display &):
    python3 undistort/undistort_worker.py
Verify with (straight lines in the scene should look straight):
    python3 tools/ringdump.py bat_rect0
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "common"))
import bat_ring
import undistort as ud


def rect_ring_path(cam_path, index):
    """/dev/shmem/bat_cam0 -> /dev/shmem/bat_rect0; file rings sit alongside."""
    directory = os.path.dirname(cam_path) or "."
    if os.path.basename(directory) == "shmem" or os.path.isdir("/dev/shmem"):
        return os.path.join("/dev/shmem", "bat_rect%d" % index)
    return os.path.join(directory, "bat_rect%d.ring" % index)


def wait_for_rings(names, timeout_s):
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            return [bat_ring.resolve_ring_path(n) for n in names]
        except FileNotFoundError as e:
            if time.monotonic() > deadline:
                sys.exit("%s\n(start the capture daemon first: ./bathat --no-display)" % e)
            time.sleep(0.25)


class CamStats:
    def __init__(self):
        self.frames = 0
        self.remap_ms = 0.0

    def add(self, remap_ms):
        self.frames += 1
        self.remap_ms += remap_ms

    def line(self, dt_s):
        if self.frames == 0:
            return "no frames"
        return "%4.1f fps   remap %5.2f ms" % (self.frames / dt_s, self.remap_ms / self.frames)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cams", nargs="+", default=None,
                        help="camera ring names/paths (default: bat_cam0, and bat_cam1 if present)")
    parser.add_argument("--balance", type=float, default=0.0,
                        help="fisheye undistort balance 0..1 (0 = crop to valid pixels, "
                             "1 = keep all FOV with black borders; default 0)")
    parser.add_argument("--max-fps", type=float, default=10.0,
                        help="per-camera rectification rate cap (default 10; "
                             "0 = every frame). Full-rate remap of both "
                             "cameras eats the CPU the depth model needs.")
    parser.add_argument("--wait", type=float, default=30.0,
                        help="seconds to wait for the camera rings to appear")
    parser.add_argument("--stats-every", type=float, default=5.0,
                        help="seconds between stats lines (0 = quiet)")
    args = parser.parse_args()

    if args.cams is None:
        cam_paths = wait_for_rings(["bat_cam0"], args.wait)
        try:
            cam_paths.append(bat_ring.resolve_ring_path("bat_cam1"))
        except FileNotFoundError:
            pass
    else:
        cam_paths = wait_for_rings(args.cams, args.wait)

    readers = [bat_ring.RingReader(p) for p in cam_paths]
    undistorters, writers = [], []
    for i, (p, reader) in enumerate(zip(cam_paths, readers)):
        hdr = reader.header()
        undistorters.append(ud.Undistorter(i, hdr.width, hdr.height, balance=args.balance))
        writers.append(bat_ring.RingWriter(rect_ring_path(p, i), bat_ring.FMT_BGR8,
                                           hdr.width, hdr.height, hdr.width * hdr.height * 3))
        state = "rectifying" if undistorters[i].enabled else "PASSTHROUGH (no calib)"
        print("cam%d: %s  ->  %s (%dx%d BGR, %s)" %
              (i, p, writers[i].path, hdr.width, hdr.height, state))
    sys.stdout.flush()

    last_idx = [None] * len(readers)
    stats = [CamStats() for _ in readers]
    interval = 1.0 / args.max_fps if args.max_fps > 0 else 0.0
    next_due = [0.0] * len(readers)
    last_stats = time.monotonic()
    try:
        while True:
            got_any = False
            for i, reader in enumerate(readers):
                if time.monotonic() < next_due[i]:
                    continue  # rate cap: let newer frames pile up, take latest
                item = reader.read_latest(last_idx[i])
                if item is None:
                    continue
                next_due[i] = time.monotonic() + interval
                meta, payload = item
                last_idx[i] = meta.frame_idx
                hdr = reader.header()

                yuv = np.frombuffer(payload, np.uint8).reshape(hdr.height * 3 // 2, hdr.width)
                bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
                t0 = time.perf_counter()
                rect = undistorters[i].apply(bgr)
                remap_ms = (time.perf_counter() - t0) * 1000.0

                writers[i].write(np.ascontiguousarray(rect, np.uint8).tobytes(),
                                 t_capture=meta.t_capture, frame_idx=meta.frame_idx)
                stats[i].add(remap_ms)
                got_any = True

            if not got_any:
                time.sleep(0.005)

            now = time.monotonic()
            if args.stats_every > 0 and now - last_stats >= args.stats_every:
                dt = now - last_stats
                print("   |   ".join("cam%d: %s" % (i, s.line(dt)) for i, s in enumerate(stats)),
                      flush=True)
                stats = [CamStats() for _ in readers]
                last_stats = now
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        for r in readers:
            r.close()
        for w in writers:
            w.close()


if __name__ == "__main__":
    main()
