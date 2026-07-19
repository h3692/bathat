#!/usr/bin/env python3
"""Unit test for common/detect.py — close-blob detection on synthetic MiDaS
maps plus the pixel->azimuth conversion. Needs numpy + OpenCV (same as the
depth pipeline); run directly or via `make -C tests`.
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, "common"))
import detect


def synthetic(w=256, h=256, far=1.0):
    return np.full((h, w), far, np.float32)


def test_single_blob():
    depth = synthetic()
    # A near square (MiDaS: larger = closer) centred at (192, 64), 40x40.
    depth[44:84, 172:212] = 10.0
    blobs = detect.detect_blobs(depth, thresh_pct=85.0, min_area_frac=0.005,
                                lo=1.0, hi=10.0)
    assert len(blobs) == 1, blobs
    b = blobs[0]
    assert abs(b.cx - 191.5 / 256.0) < 0.02, b
    assert abs(b.cy - 63.5 / 256.0) < 0.02, b
    assert abs(b.area_frac - (40 * 40) / 65536.0) < 0.002, b
    assert abs(b.radius - math.sqrt(40 * 40 / math.pi) / 256.0) < 0.01, b
    assert b.closeness > 0.95, b


def test_two_blobs_sorted_by_closeness():
    depth = synthetic()
    depth[100:140, 20:60] = 6.0     # farther blob on the left
    depth[100:150, 180:240] = 10.0  # nearer blob on the right
    blobs = detect.detect_blobs(depth, thresh_pct=85.0, min_area_frac=0.005,
                                lo=1.0, hi=10.0)
    assert len(blobs) == 2, blobs
    assert blobs[0].closeness > blobs[1].closeness, blobs
    assert blobs[0].cx > 0.5 > blobs[1].cx, blobs


def test_noise_only_yields_nothing():
    rng = np.random.default_rng(7)
    depth = rng.uniform(1.0, 2.0, (256, 256)).astype(np.float32)
    blobs = detect.detect_blobs(depth, thresh_pct=85.0, min_area_frac=0.005,
                                lo=1.0, hi=2.0)
    assert blobs == [], blobs


def test_min_area_filters_specks():
    depth = synthetic()
    depth[10:14, 10:14] = 10.0  # 16 px — far below 0.5% of 65536
    blobs = detect.detect_blobs(depth, thresh_pct=85.0, min_area_frac=0.005,
                                lo=1.0, hi=10.0)
    assert blobs == [], blobs


def test_azimuth_from_new_K():
    new_K = np.array([[800.0, 0.0, 768.0],
                      [0.0, 800.0, 432.0],
                      [0.0, 0.0, 1.0]])
    # Principal point: bearing 0 -> azimuth is exactly the camera yaw.
    assert abs(detect.azimuth_deg(0.5, new_K, 1536, yaw_deg=-45.0) - (-45.0)) < 1e-6
    # Right edge: atan((1536 - 768) / 800), positive (to the user's right).
    want = 45.0 + math.degrees(math.atan2(768.0, 800.0))
    assert abs(detect.azimuth_deg(1.0, new_K, 1536, yaw_deg=45.0) - want) < 1e-6
    # Left of the principal point is negative.
    assert detect.azimuth_deg(0.25, new_K, 1536, yaw_deg=0.0) < 0.0


def test_azimuth_fallback_linear_fov():
    # No calibration: linear map across --fov degrees.
    assert abs(detect.azimuth_deg(0.5, None, 1536, yaw_deg=10.0, fov_deg=90.0)
               - 10.0) < 1e-6
    assert abs(detect.azimuth_deg(1.0, None, 1536, yaw_deg=0.0, fov_deg=90.0)
               - 45.0) < 1e-6


def test_rolling_norm():
    norm = detect.RollingNorm(alpha=0.15)
    depth = synthetic(far=2.0)
    depth[:128, :] = 8.0  # half the frame near, so P95 actually lands at 8
    lo0, hi0, ref0 = norm.update(depth)
    assert lo0 == np.percentile(depth, 5) and hi0 == np.percentile(depth, 95)
    assert ref0 == np.percentile(depth, 40)
    lo1, hi1, ref1 = norm.update(synthetic(far=2.0))  # hi decays toward new P95
    assert hi1 < hi0 and lo1 <= lo0 + 1e-6, (lo1, hi1, ref1)


def test_contrast_gate_mutes_smooth_scenes():
    # An empty corridor: depth falls off smoothly, so the "nearest 15%" is just
    # the near end of the gradient — it barely stands out from the median and
    # must be gated out (nothing is actually close).
    depth = np.tile(np.linspace(1.0, 5.0, 256, dtype=np.float32), (256, 1))
    lo, hi = np.percentile(depth, 5), np.percentile(depth, 95)
    ref = np.percentile(depth, 40)
    blobs = detect.detect_blobs(depth, thresh_pct=85.0, min_area_frac=0.005,
                                lo=lo, hi=hi, ref=ref, min_contrast=0.65)
    assert blobs == [], blobs
    # A genuinely close object towers over the median -> passes the gate.
    depth2 = synthetic()
    depth2[100:160, 100:160] = 10.0
    lo2, hi2 = np.percentile(depth2, 5), np.percentile(depth2, 95)
    ref2 = np.percentile(depth2, 40)
    blobs2 = detect.detect_blobs(depth2, thresh_pct=85.0, min_area_frac=0.005,
                                 lo=lo2, hi=hi2, ref=ref2, min_contrast=0.65)
    assert len(blobs2) == 1, blobs2


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print("test_detect: %d checks passed" % len(tests))


if __name__ == "__main__":
    main()
