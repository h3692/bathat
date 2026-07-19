#!/usr/bin/env python3
"""Save frames from a bat_ring as PNGs — the eyeball check for every pipeline
stage. Works on camera rings (NV12 -> color PNG) and depth rings (float32 ->
inferno colormap PNG, bright/warm = near), auto-detected from the ring header.

    python3 tools/ringdump.py bat_cam0              # 3 fresh frames as PNGs
    python3 tools/ringdump.py bat_depth0 --count 1

Ring names resolve to /dev/shmem/<name> on QNX; plain file paths also work
(host tests). Needs numpy + OpenCV.
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "common"))
import bat_ring


def to_bgr(hdr, payload):
    if hdr.format == bat_ring.FMT_NV12:
        yuv = np.frombuffer(payload, np.uint8).reshape(hdr.height * 3 // 2, hdr.width)
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
    if hdr.format == bat_ring.FMT_F32:
        depth = np.frombuffer(payload, np.float32).reshape(hdr.height, hdr.width)
        normalized = (depth - depth.min()) / max(float(depth.max() - depth.min()), 1e-6)
        return cv2.applyColorMap((normalized * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
    sys.exit("unknown ring format %d" % hdr.format)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ring", help="ring name (bat_cam0) or file path")
    parser.add_argument("--out", default=".", help="output directory (default: .)")
    parser.add_argument("--count", type=int, default=3,
                        help="number of distinct frames to save (default 3)")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="seconds to wait for a new frame (default 10)")
    args = parser.parse_args()

    path = bat_ring.resolve_ring_path(args.ring)
    stem = os.path.splitext(os.path.basename(path))[0]
    os.makedirs(args.out, exist_ok=True)

    fmt_names = {bat_ring.FMT_NV12: "NV12", bat_ring.FMT_F32: "depth-f32"}
    with bat_ring.RingReader(path) as reader:
        last_idx = None
        saved = 0
        deadline = time.monotonic() + args.timeout
        while saved < args.count:
            got = reader.read_latest(last_idx)
            if got is None:
                if time.monotonic() > deadline:
                    sys.exit("timed out after %d frame(s): no new frame in %.0f s "
                             "— is the writer running?" % (saved, args.timeout))
                time.sleep(0.002)
                continue
            meta, payload = got
            last_idx = meta.frame_idx
            hdr = reader.header()

            out = os.path.join(args.out, "%s_f%06d.png" % (stem, meta.frame_idx))
            cv2.imwrite(out, to_bgr(hdr, payload))
            now = bat_ring.now_ns()
            print("saved %s   %ux%u %s   capture->publish %.1f ms   published %.1f ms ago"
                  % (out, hdr.width, hdr.height,
                     fmt_names.get(hdr.format, "?"),
                     (meta.t_publish - meta.t_capture) / 1e6,
                     (now - meta.t_publish) / 1e6))
            saved += 1
            deadline = time.monotonic() + args.timeout


if __name__ == "__main__":
    main()
