#!/usr/bin/env python3
"""Depth worker: camera frames in, MiDaS depth maps out. Step 2/3 of the plan.

Reads frames from the rectified rings (/bat_rect0, /bat_rect1, written by the
undistort worker; pass --cams to override, e.g. the raw /bat_cam*), runs MiDaS
v2.1 small (256x256 TFLite) on the newest frame, and publishes each raw float32
depth map to a matching output ring (/bat_depth0, /bat_depth1). With two cameras
it alternates left/right, so each eye updates at half the model rate — as
planned. Rings may be NV12 (raw cameras) or BGR (rectified); both are handled.

The worker free-runs: it always grabs the *newest* frame and never queues, so
if inference is slower than the camera, frames are skipped on purpose. Source
capture timestamps ride along into the depth ring, and a stats line prints
every few seconds with the camera-to-depth latency.

Depth values are MiDaS relative depth (bigger = nearer, arbitrary per-frame
scale); consumers must normalize over a rolling window. Each depth map is also
segmented into close blobs (common/detect.py) and the result — centroid,
size, closeness, world azimuth — is published to /bat_det0, /bat_det1 for the
viewfinder overlay and the audio process.

On the Pi (after starting the capture daemon: ./bathat --no-display &):
    python3 depth/depth_worker.py
On the Mac, against the fake camera (tools/imgcam.py):
    python3 depth/depth_worker.py --cams ./bat_cam0.ring

Verify output with:  python3 tools/ringdump.py bat_depth0
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
import detect
from undistort import Undistorter


def load_interpreter_class():
    """Return (Interpreter class, name of the package it came from)."""
    try:
        from tflite_runtime.interpreter import Interpreter
        return Interpreter, "tflite_runtime"
    except ImportError:
        pass
    try:
        from ai_edge_litert.interpreter import Interpreter
        return Interpreter, "ai_edge_litert"
    except ImportError:
        pass
    try:
        from tensorflow.lite import Interpreter
        return Interpreter, "tensorflow"
    except ImportError:
        sys.exit("No TFLite runtime found (tried tflite_runtime, "
                 "ai_edge_litert, tensorflow).")


def default_model_path():
    # models/ in this repo (./models/ when run from the repo root).
    return os.path.join(_ROOT, "models", "midas_v21_small_256.tflite")


def wait_for_rings(names, timeout_s):
    """Resolve ring names to paths, waiting for the capture daemon if needed."""
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            return [bat_ring.resolve_ring_path(n) for n in names]
        except FileNotFoundError as e:
            if time.monotonic() > deadline:
                sys.exit("%s\n(start the pipeline first: ./bathat --no-display &  then\n"
                         " python3 undistort/undistort_worker.py — this reads its "
                         "/bat_rect* rings)" % e)
            time.sleep(0.25)


def depth_ring_path(cam_path, index):
    """/dev/shmem/bat_cam0 -> /dev/shmem/bat_depth0; file rings sit alongside."""
    directory = os.path.dirname(cam_path) or "."
    if os.path.basename(directory) == "shmem" or os.path.isdir("/dev/shmem"):
        return os.path.join("/dev/shmem", "bat_depth%d" % index)
    return os.path.join(directory, "bat_depth%d.ring" % index)


def det_ring_path(cam_path, index):
    directory = os.path.dirname(cam_path) or "."
    if os.path.basename(directory) == "shmem" or os.path.isdir("/dev/shmem"):
        return os.path.join("/dev/shmem", "bat_det%d" % index)
    return os.path.join(directory, "bat_det%d.ring" % index)


# MiDaS v2.1 small expects RGB in [0,1] normalized with ImageNet mean/std
# (same preprocessing as bat-tim/depth/bench_midas.py).
_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def preprocess(payload, fmt, cam_w, cam_h, model_w, model_h):
    if fmt == bat_ring.FMT_BGR8:
        bgr = np.frombuffer(payload, np.uint8).reshape(cam_h, cam_w, 3)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    else:  # NV12
        yuv = np.frombuffer(payload, np.uint8).reshape(cam_h * 3 // 2, cam_w)
        rgb = cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB_NV12)
    resized = cv2.resize(rgb, (model_w, model_h), interpolation=cv2.INTER_AREA)
    normalized = (resized.astype(np.float32) / 255.0 - _MEAN) / _STD
    return normalized[np.newaxis, ...]


class CamStats:
    def __init__(self):
        self.frames = 0
        self.invoke_ms = 0.0
        self.e2e_ms = 0.0

    def add(self, invoke_ms, e2e_ms):
        self.frames += 1
        self.invoke_ms += invoke_ms
        self.e2e_ms += e2e_ms

    def line(self, dt_s):
        if self.frames == 0:
            return "no frames"
        return ("%4.1f fps   invoke %5.1f ms   camera->depth %5.1f ms"
                % (self.frames / dt_s, self.invoke_ms / self.frames,
                   self.e2e_ms / self.frames))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cams", nargs="+", default=None,
                        help="camera ring names/paths (default: bat_cam0, and "
                             "bat_cam1 if present)")
    parser.add_argument("--model", default=default_model_path())
    parser.add_argument("--threads", type=int, default=4,
                        help="TFLite CPU threads (4 was fastest on the Pi)")
    parser.add_argument("--wait", type=float, default=30.0,
                        help="seconds to wait for the camera rings to appear")
    parser.add_argument("--stats-every", type=float, default=5.0,
                        help="seconds between stats lines (0 = quiet)")
    parser.add_argument("--det-thresh", type=float, default=85.0,
                        help="close-pixel percentile for blob detection")
    parser.add_argument("--min-area", type=float, default=0.005,
                        help="minimum blob area as a fraction of the depth map")
    parser.add_argument("--min-contrast", type=float, default=0.65,
                        help="how far a blob must stand above the scene's "
                             "P40 depth (fraction of the frame range) to "
                             "count as close; below it nothing is near")
    parser.add_argument("--yaw0", type=float, default=-45.0,
                        help="cam0 mounting yaw in degrees (0 = user's forward)")
    parser.add_argument("--yaw1", type=float, default=45.0,
                        help="cam1 mounting yaw in degrees")
    parser.add_argument("--fov", type=float, default=90.0,
                        help="horizontal FOV fallback when uncalibrated")
    args = parser.parse_args()

    if args.cams is None:
        cam_paths = wait_for_rings(["bat_rect0"], args.wait)
        try:
            cam_paths.append(bat_ring.resolve_ring_path("bat_rect1"))
        except FileNotFoundError:
            pass
    else:
        cam_paths = wait_for_rings(args.cams, args.wait)

    interpreter_class, runtime_name = load_interpreter_class()
    interpreter = interpreter_class(model_path=args.model, num_threads=args.threads)
    interpreter.allocate_tensors()
    input_detail = interpreter.get_input_details()[0]
    output_detail = interpreter.get_output_details()[0]
    _, model_h, model_w, _ = input_detail["shape"]
    out_shape = [int(d) for d in output_detail["shape"] if int(d) != 1]
    out_h, out_w = out_shape if len(out_shape) == 2 else (model_h, model_w)

    readers = [bat_ring.RingReader(p) for p in cam_paths]
    writers = [bat_ring.RingWriter(depth_ring_path(p, i), bat_ring.FMT_F32,
                                   out_w, out_h, out_w * out_h * 4)
               for i, p in enumerate(cam_paths)]
    det_writers = [bat_ring.RingWriter(det_ring_path(p, i), bat_ring.FMT_DET,
                                       out_w, out_h, bat_ring.DET_SLOT_SIZE)
                   for i, p in enumerate(cam_paths)]
    norms = [detect.RollingNorm() for _ in readers]
    yaws = [args.yaw0, args.yaw1]
    # new_K of the rectified stream each camera publishes (None -> FOV fallback);
    # built lazily once the frame size is known from the ring header.
    new_Ks = [None] * len(readers)
    calibrated = [False] * len(readers)
    last_idx = [None] * len(readers)
    stats = [CamStats() for _ in readers]

    print("runtime: %s   model: %s   %d thread(s)" %
          (runtime_name, os.path.basename(args.model), args.threads))
    for i, (cam, wr, dw) in enumerate(zip(cam_paths, writers, det_writers)):
        print("cam%d: %s  ->  %s (%dx%d float32)  +  %s (detections)"
              % (i, cam, wr.path, out_w, out_h, dw.path))
    sys.stdout.flush()  # stats must show up promptly even when piped to a log

    last_stats = time.monotonic()
    try:
        while True:
            got_any = False
            # Round-robin over cameras: with two, this alternates left/right.
            for i, reader in enumerate(readers):
                item = reader.read_latest(last_idx[i])
                if item is None:
                    continue
                meta, payload = item
                last_idx[i] = meta.frame_idx
                hdr = reader.header()

                tensor = preprocess(payload, hdr.format, hdr.width, hdr.height, model_w, model_h)
                t0 = time.perf_counter()
                interpreter.set_tensor(input_detail["index"], tensor)
                interpreter.invoke()
                invoke_ms = (time.perf_counter() - t0) * 1000.0
                depth = np.squeeze(interpreter.get_tensor(output_detail["index"]))

                writers[i].write(np.ascontiguousarray(depth, np.float32).tobytes(),
                                 t_capture=meta.t_capture, frame_idx=meta.frame_idx)

                if not calibrated[i]:
                    und = Undistorter(i, hdr.width, hdr.height)
                    new_Ks[i] = und.new_K if und.enabled else None
                    calibrated[i] = True
                lo, hi, ref = norms[i].update(depth)
                yaw = yaws[i] if i < len(yaws) else 0.0
                dets = [bat_ring.Detection(
                            cx=b.cx, cy=b.cy, radius=b.radius,
                            closeness=b.closeness, area_frac=b.area_frac,
                            azimuth_deg=detect.azimuth_deg(
                                b.cx, new_Ks[i], hdr.width, yaw, args.fov))
                        for b in detect.detect_blobs(depth, args.det_thresh,
                                                     args.min_area, lo, hi,
                                                     ref, args.min_contrast)]
                det_writers[i].write(bat_ring.pack_detections(dets),
                                     t_capture=meta.t_capture,
                                     frame_idx=meta.frame_idx)

                e2e_ms = (bat_ring.now_ns() - meta.t_capture) / 1e6
                stats[i].add(invoke_ms, e2e_ms)
                got_any = True

            if not got_any:
                time.sleep(0.005)  # newest frame already processed; wait a beat

            now = time.monotonic()
            if args.stats_every > 0 and now - last_stats >= args.stats_every:
                dt = now - last_stats
                print("   |   ".join("cam%d: %s" % (i, s.line(dt))
                                     for i, s in enumerate(stats)), flush=True)
                stats = [CamStats() for _ in readers]
                last_stats = now
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        for r in readers:
            r.close()
        for w in writers:
            w.close()
        for w in det_writers:
            w.close()


if __name__ == "__main__":
    main()
