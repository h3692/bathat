#!/usr/bin/env python3
"""Fake camera: feed JPEG test images into an NV12 bat_ring at a steady rate.

Lets the whole downstream pipeline (tools/ringdump.py, depth/depth_worker.py)
run and be verified on the Mac — no QNX, no cameras. Also handy on the Pi for
exercising the depth worker without pointing cameras at anything.

    python3 tools/imgcam.py --ring ./bat_cam0.ring

Cycles the images forever at --fps until Ctrl-C (or --count frames).
"""

import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "common"))
import bat_ring


def bgr_to_nv12(bgr):
    """NV12 = full-res Y plane, then one interleaved UV plane at half height."""
    h, w = bgr.shape[:2]
    i420 = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420)  # planar: Y, U, V
    y = i420[:h]
    u = i420[h:h + h // 4].reshape(h // 2, w // 2)
    v = i420[h + h // 4:].reshape(h // 2, w // 2)
    uv = np.empty((h // 2, w), np.uint8)
    uv[:, 0::2] = u
    uv[:, 1::2] = v
    return np.vstack([y, uv])


def default_ring_path():
    if os.path.isdir("/dev/shmem"):  # QNX: a real shared-memory object
        return "/dev/shmem/bat_cam0"
    return "./bat_cam0.ring"


def default_images_dir():
    # testdata/ in this repo, or the bat-tim benchmark kit (sibling checkout
    # on the Mac; ~/repos/tims-bat on the Pi).
    for candidate in (os.path.join(_ROOT, "testdata"),
                      os.path.join(_ROOT, os.pardir, "bat-tim", "testdata"),
                      os.path.join(_ROOT, os.pardir, "tims-bat", "testdata")):
        if os.path.isdir(candidate):
            return candidate
    return candidate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ring", default=default_ring_path(),
                        help="ring file to create (default: %(default)s)")
    parser.add_argument("--images", default=default_images_dir(),
                        help="directory of .jpg images to cycle through")
    parser.add_argument("--width", type=int, default=1536,
                        help="frame width, matching the real camera (default 1536)")
    parser.add_argument("--height", type=int, default=864)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--count", type=int, default=0,
                        help="stop after N frames (default 0 = run until Ctrl-C)")
    args = parser.parse_args()

    paths = sorted(glob.glob(os.path.join(args.images, "*.jpg")))
    if not paths:
        sys.exit("no .jpg files in %s" % args.images)
    frames = [bgr_to_nv12(cv2.resize(cv2.imread(p), (args.width, args.height),
                                     interpolation=cv2.INTER_AREA))
              for p in paths]
    print("feeding %d image(s) at %.1f fps into %s (%dx%d NV12), Ctrl-C to stop"
          % (len(frames), args.fps, args.ring, args.width, args.height))

    slot_size = args.width * args.height * 3 // 2
    period = 1.0 / args.fps
    written = 0
    with bat_ring.RingWriter(args.ring, bat_ring.FMT_NV12,
                             args.width, args.height, slot_size) as ring:
        try:
            while args.count <= 0 or written < args.count:
                start = time.monotonic()
                ring.write(frames[written % len(frames)].tobytes())
                written += 1
                time.sleep(max(0.0, period - (time.monotonic() - start)))
        except KeyboardInterrupt:
            pass
    print("wrote %d frames" % written)


if __name__ == "__main__":
    main()
