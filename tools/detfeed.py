#!/usr/bin/env python3
"""Feed synthetic detections into bat_det rings — exercise the viewfinder
overlay and bat_audio without cameras or the depth worker.

Simulates one object sweeping across the 180-degree arc while closing in:
azimuth runs a triangle wave between --az-min and --az-max while closeness
ramps far -> near over each pass. Each camera ring only publishes the object
while it is inside that camera's field (yaw +/- fov/2), so the mid-arc overlap
handoff and cross-camera fusion get exercised too.

    python3 tools/detfeed.py                                # /dev/shmem (Pi)
    python3 tools/detfeed.py --rings ./bat_det0.ring ./bat_det1.ring

Expect: bat_audio pans the hum across the headphones as the azimuth sweeps,
louder and faster-pulsing toward the end of each pass. Stdlib-only.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "common"))
import bat_ring


def default_rings():
    if os.path.isdir("/dev/shmem"):
        return ["/dev/shmem/bat_det0", "/dev/shmem/bat_det1"]
    return ["./bat_det0.ring", "./bat_det1.ring"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rings", nargs="+", default=None,
                        help="detection ring paths, one per camera "
                             "(default: bat_det0 and bat_det1)")
    parser.add_argument("--fps", type=float, default=4.0,
                        help="detection updates per second (default 4, "
                             "matching the real depth cadence)")
    parser.add_argument("--period", type=float, default=12.0,
                        help="seconds per full sweep (default 12)")
    parser.add_argument("--az-min", type=float, default=-80.0)
    parser.add_argument("--az-max", type=float, default=80.0)
    parser.add_argument("--yaw0", type=float, default=-45.0)
    parser.add_argument("--yaw1", type=float, default=45.0)
    parser.add_argument("--fov", type=float, default=90.0,
                        help="per-camera horizontal FOV (default 90)")
    args = parser.parse_args()

    rings = args.rings if args.rings else default_rings()
    yaws = [args.yaw0, args.yaw1]
    writers = [bat_ring.RingWriter(path, bat_ring.FMT_DET, 256, 256,
                                   bat_ring.DET_SLOT_SIZE)
               for path in rings]
    for i, w in enumerate(writers):
        print("cam%d (yaw %+.0f): %s" % (i, yaws[i] if i < len(yaws) else 0.0, w.path))

    t0 = time.monotonic()
    try:
        while True:
            phase = ((time.monotonic() - t0) % args.period) / args.period
            # Triangle wave: left -> right in the first half, back in the second.
            tri = 2.0 * phase if phase < 0.5 else 2.0 - 2.0 * phase
            azimuth = args.az_min + (args.az_max - args.az_min) * tri
            closeness = 0.35 + 0.65 * phase  # approaches over each sweep

            for i, writer in enumerate(writers):
                yaw = yaws[i] if i < len(yaws) else 0.0
                relative = azimuth - yaw
                dets = []
                if abs(relative) <= args.fov / 2.0:
                    dets.append(bat_ring.Detection(
                        cx=0.5 + relative / args.fov, cy=0.5, radius=0.08,
                        closeness=closeness, azimuth_deg=azimuth,
                        area_frac=0.02))
                writer.write(bat_ring.pack_detections(dets))
            time.sleep(1.0 / args.fps)
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        for w in writers:
            w.close()


if __name__ == "__main__":
    main()
