"""Fisheye undistortion, shared by the undistort worker and any tool that needs
rectified frames.

A calibration lives in calib/camN.yml (K, D, and the width/height it was solved
at), produced by tools/calibrate.py. Undistorter loads it once, builds the remap
maps for the live frame size, and applies them per frame.

If no calibration exists yet, Undistorter falls back to passthrough (returns the
frame unchanged) with a one-time warning, so the pipeline runs before you have
calibrated.

The rectified new camera matrix (new_K) is kept on the instance: later stages
turn a rectified pixel x into a bearing with atan((x - cx) / fx) using new_K.

Needs numpy + OpenCV (python3-opencv on the Pi).
"""

import os
import sys

import cv2
import numpy as np


def calib_dir():
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "calib")


def calib_path(cam_index):
    return os.path.join(calib_dir(), "cam%d.yml" % cam_index)


def save_calib(path, K, D, width, height):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    fs.write("model", "fisheye")
    fs.write("width", int(width))
    fs.write("height", int(height))
    fs.write("K", np.asarray(K, np.float64))
    fs.write("D", np.asarray(D, np.float64))
    fs.release()


def load_calib(path):
    """Return (K, D, width, height) or None if the file is missing/unreadable."""
    if not os.path.exists(path):
        return None
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    K = fs.getNode("K").mat()
    D = fs.getNode("D").mat()
    width = int(fs.getNode("width").real())
    height = int(fs.getNode("height").real())
    fs.release()
    if K is None or D is None or width <= 0 or height <= 0:
        return None
    return K, D, width, height


class Undistorter:
    """Per-camera fisheye undistortion for a fixed live frame size."""

    def __init__(self, cam_index, width, height, balance=0.0):
        self.width = width
        self.height = height
        self.enabled = False
        self.new_K = None

        calib = load_calib(calib_path(cam_index))
        if calib is None:
            sys.stderr.write("undistort: no calibration for cam%d (%s) — passthrough\n"
                             % (cam_index, calib_path(cam_index)))
            return

        K, D, cw, ch = calib
        K = np.asarray(K, np.float64).copy()
        D = np.asarray(D, np.float64)
        # If calibrated at a different resolution than the live frame, scale the
        # intrinsics to match (fx, fy, cx, cy all scale with the axis).
        if (cw, ch) != (width, height) and cw > 0 and ch > 0:
            sx, sy = width / float(cw), height / float(ch)
            K[0, 0] *= sx; K[0, 2] *= sx
            K[1, 1] *= sy; K[1, 2] *= sy

        # balance: 0 crops to only-valid pixels (no black border, some FOV lost),
        # 1 keeps all FOV (black borders at the edges).
        self.new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            K, D, (width, height), np.eye(3), balance=balance)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            K, D, np.eye(3), self.new_K, (width, height), cv2.CV_16SC2)
        self.enabled = True

    def apply(self, bgr):
        """Return the rectified BGR frame (or the input unchanged if disabled)."""
        if not self.enabled:
            return bgr
        return cv2.remap(bgr, self.map1, self.map2, interpolation=cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT)
