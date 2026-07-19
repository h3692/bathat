"""Close-blob detection on MiDaS depth maps, shared by the depth worker.

MiDaS output is relative (larger = nearer, arbitrary per-frame scale), so the
segmentation threshold is a per-frame percentile — rank-based and immune to the
scale — while the *closeness* value reported per blob comes from per-camera
rolling extremes (RollingNorm, same exponential smoothing the viewfinder uses
to colorize) so it is stable across frames and roughly comparable between the
two cameras.

Pixel x -> world azimuth uses the rectified camera matrix from
common/undistort.py (bearing = atan((x - cx) / fx) with new_K), plus the
camera's mounting yaw; with no calibration a linear FOV map stands in.

Pure numpy + OpenCV — no TFLite dependency, so tests run anywhere.
"""

import math
from collections import namedtuple

import cv2
import numpy as np

# cx/cy/radius normalized to the depth-map width/height (radius by width);
# closeness in [0,1], 1 = nearest. Matches bat_ring's DET record sans azimuth,
# which the caller adds via azimuth_deg().
Blob = namedtuple("Blob", "cx cy radius closeness area_frac")

_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))


class RollingNorm:
    """Per-camera rolling depth extremes (P5/P95, EMA-smoothed)."""

    def __init__(self, alpha=0.15):
        self.alpha = alpha
        self.lo = None
        self.hi = None

    def update(self, depth):
        """Feed one depth map; return the smoothed (lo, hi)."""
        mn = float(np.percentile(depth, 5))
        mx = float(np.percentile(depth, 95))
        if self.lo is None:
            self.lo, self.hi = mn, mx
        else:
            self.lo += self.alpha * (mn - self.lo)
            self.hi += self.alpha * (mx - self.hi)
        return self.lo, self.hi


def detect_blobs(depth, thresh_pct, min_area_frac, lo, hi):
    """Segment the nearest pixels of one depth map into blobs.

    Returns a list of Blob sorted by closeness descending (nearest first),
    capped at 8. `lo`/`hi` are the rolling extremes used to score closeness;
    the segmentation itself thresholds at the frame's `thresh_pct` percentile.
    """
    h, w = depth.shape
    # Strict >: in a near-flat scene the percentile ties with the background
    # value, and >= would swallow the whole frame as one blob.
    mask = (depth > np.percentile(depth, thresh_pct)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _KERNEL)

    n, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    span = max(hi - lo, 1e-9)
    blobs = []
    for i in range(1, n):  # label 0 is background
        area = int(stats[i, cv2.CC_STAT_AREA])
        area_frac = area / float(w * h)
        if area_frac < min_area_frac:
            continue
        mean_depth = float(depth[labels == i].mean())
        blobs.append(Blob(
            cx=float(centroids[i][0]) / w,
            cy=float(centroids[i][1]) / h,
            radius=math.sqrt(area / math.pi) / w,
            closeness=min(max((mean_depth - lo) / span, 0.0), 1.0),
            area_frac=area_frac,
        ))
    blobs.sort(key=lambda b: b.closeness, reverse=True)
    return blobs[:8]


def azimuth_deg(cx_norm, new_K, frame_w, yaw_deg, fov_deg=90.0):
    """World azimuth (degrees) of a normalized x position for one camera.

    `new_K` is the rectified 3x3 camera matrix (built at frame_w's resolution);
    the anisotropic depth-map resize preserves x as a fraction, so cx_norm maps
    straight to the rectified frame. With no calibration (new_K is None), a
    linear map across `fov_deg` stands in. 0 = straight ahead of the *user*,
    positive = right; `yaw_deg` is the camera's mounting yaw.
    """
    if new_K is None:
        bearing = (cx_norm - 0.5) * fov_deg
    else:
        x_full = cx_norm * frame_w
        bearing = math.degrees(math.atan2(x_full - new_K[0][2], new_K[0][0]))
    return yaw_deg + bearing
