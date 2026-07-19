#!/usr/bin/env python3
"""Fisheye calibration for the bat-hat cameras using a checkerboard.

Show a checkerboard on a monitor (or print one) and calibrate each camera:

  1) make a board to display fullscreen (optional helper):
        python3 tools/calibrate.py --make-board board.png

  2) capture -- auto-saves a shot whenever the whole board is detected. Move the
     board around the frame (reach the edges and corners too, and tilt it) until
     it has grabbed enough (default 20). Needs the capture daemon running:
        ./bathat --no-display &
        python3 tools/calibrate.py --cam 0 --capture

  3) solve -- detect corners in the shots, run cv2.fisheye.calibrate, and write
     calib/cam0.yml:
        python3 tools/calibrate.py --cam 0 --solve

Repeat --cam 1 for the second camera. Board geometry is given as INNER corners
(--cols/--rows, default 9x6, i.e. a 10x7-square board). Square size doesn't
matter for undistortion.

Needs numpy + OpenCV.
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
import undistort as ud


def shots_dir(cam):
    return os.path.join(ud.calib_dir(), "cam%d_shots" % cam)


def make_board(path, cols, rows, square_px=100, margin=80):
    """Write a checkerboard PNG with (cols+1)x(rows+1) squares to display."""
    nx, ny = cols + 1, rows + 1
    img = np.full((ny * square_px + 2 * margin, nx * square_px + 2 * margin), 255, np.uint8)
    for j in range(ny):
        for i in range(nx):
            if (i + j) % 2 == 0:
                y0, x0 = margin + j * square_px, margin + i * square_px
                img[y0:y0 + square_px, x0:x0 + square_px] = 0
    cv2.imwrite(path, img)
    print("wrote %s (%dx%d squares, %dx%d inner corners) -- open it fullscreen"
          % (path, nx, ny, cols, rows))


def capture(cam, cols, rows, target, cooldown):
    path = bat_ring.resolve_ring_path("bat_cam%d" % cam)
    outdir = shots_dir(cam)
    os.makedirs(outdir, exist_ok=True)
    saved = len(glob.glob(os.path.join(outdir, "*.png")))
    print("cam%d: have %d shot(s), want %d. Move the board around the whole frame."
          % (cam, saved, target))
    find_flags = (cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
                  + cv2.CALIB_CB_FAST_CHECK)
    last = 0.0
    last_idx = None
    with bat_ring.RingReader(path) as reader:
        while saved < target:
            item = reader.read_latest(last_idx)
            if item is None:
                time.sleep(0.01)
                continue
            meta, payload = item
            last_idx = meta.frame_idx
            hdr = reader.header()
            # The Y (luma) plane is the first w*h bytes -- all corner detection needs.
            gray = np.frombuffer(payload, np.uint8,
                                 count=hdr.width * hdr.height).reshape(hdr.height, hdr.width)
            found, _ = cv2.findChessboardCorners(gray, (cols, rows), find_flags)
            now = time.monotonic()
            if found and now - last >= cooldown:
                fn = os.path.join(outdir, "shot_%03d.png" % saved)
                cv2.imwrite(fn, gray)
                saved += 1
                last = now
                print("  [%d/%d] saved %s" % (saved, target, fn))
    print("captured %d. Now: python3 tools/calibrate.py --cam %d --solve" % (saved, cam))


def solve(cam, cols, rows):
    files = sorted(glob.glob(os.path.join(shots_dir(cam), "*.png")))
    if len(files) < 5:
        sys.exit("need >=5 shots, have %d in %s (run --capture first)"
                 % (len(files), shots_dir(cam)))

    objp = np.zeros((1, cols * rows, 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    subpix = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

    objpoints, imgpoints, size = [], [], None
    for f in files:
        gray = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            continue
        size = (gray.shape[1], gray.shape[0])  # (w, h)
        found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
        if not found:
            print("  no board in %s (skipped)" % os.path.basename(f))
            continue
        corners = cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), subpix)
        objpoints.append(objp)
        imgpoints.append(corners)

    if len(objpoints) < 5:
        sys.exit("only %d usable shots -- capture more, varied views" % len(objpoints))

    n = len(objpoints)
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rvecs = [np.zeros((1, 1, 3), np.float64) for _ in range(n)]
    tvecs = [np.zeros((1, 1, 3), np.float64) for _ in range(n)]
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
    rms, K, D, _, _ = cv2.fisheye.calibrate(
        objpoints, imgpoints, size, K, D, rvecs, tvecs, flags,
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))

    print("cam%d: calibrated from %d shots, RMS reprojection error %.3f px" % (cam, n, rms))
    if rms > 1.0:
        print("  (RMS > 1px is high -- capture more views, especially board near the edges)")
    ud.save_calib(ud.calib_path(cam), K, D, size[0], size[1])
    print("  wrote %s" % ud.calib_path(cam))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cam", type=int, default=0, help="camera index (0 or 1)")
    ap.add_argument("--cols", type=int, default=9, help="inner corners across (default 9)")
    ap.add_argument("--rows", type=int, default=6, help="inner corners down (default 6)")
    ap.add_argument("--capture", action="store_true", help="capture shots from the camera ring")
    ap.add_argument("--solve", action="store_true", help="calibrate from captured shots")
    ap.add_argument("--target", type=int, default=20, help="shots to capture (default 20)")
    ap.add_argument("--cooldown", type=float, default=0.7,
                    help="min seconds between auto-captured shots (default 0.7)")
    ap.add_argument("--make-board", metavar="PATH", default=None,
                    help="write a checkerboard PNG to display, then exit")
    args = ap.parse_args()

    if args.make_board:
        make_board(args.make_board, args.cols, args.rows)
    elif args.capture:
        capture(args.cam, args.cols, args.rows, args.target, args.cooldown)
    elif args.solve:
        solve(args.cam, args.cols, args.rows)
    else:
        ap.error("choose --capture, --solve, or --make-board")


if __name__ == "__main__":
    main()
