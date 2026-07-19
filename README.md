# real-bat

The bat-hat pipeline: dual Camera Module 3 Wide capture on a Raspberry Pi 5
running QNX 8, monocular depth estimation with MiDaS, and (eventually)
spatialized bat-click audio. This repo holds the live pipeline; the step-1
benchmark kit and the project handover live in the sibling `../bat-tim` checkout
(`~/repos/tims-bat` on the Pi) — see its HANDOVER.md for the full plan and
history.

## How the pipeline fits together

```
camera 0 ──┐                        ┌── /bat_depth0 (float32 depth ring)
           │  bathat (C daemon)     │
camera 1 ──┘  publishes NV12 into   │  depth_worker.py (Python, MiDaS TFLite)
              /bat_cam0, /bat_cam1 ─┘  alternates cameras, newest frame wins
```

The two processes share frames through `bat_ring` shared-memory rings
(4 slots, latest-frame-wins, seqlock; format defined once in
[common/bat_ring.h](common/bat_ring.h) and mirrored in
[common/bat_ring.py](common/bat_ring.py)). On QNX the rings are visible as
`/dev/shmem/bat_cam0` etc. Every frame carries its capture timestamp
(CLOCK_MONOTONIC), so each stage can report true camera-to-output latency.

Layout: `src/` capture daemon + on-screen viewfinder (C++), `common/` ring
format, `depth/` the depth worker, `tools/` debugging tools, `tests/` host
unit tests, `models/` the MiDaS model (copied in, not committed).

## Build & run on the Pi (native, no cross-compiling)

```sh
sudo apk add qnx-screen-dev qnx-sensor-framework-dev   # once
make                                                   # -> ./bathat

./bathat --probe            # print supported cameras/resolutions
./bathat                    # viewfinder on screen + publish to rings
./bathat --no-display &     # headless capture daemon (prints fps every 2 s)

python3 depth/depth_worker.py            # rings -> MiDaS -> depth rings
python3 tools/ringdump.py bat_cam0       # save camera frames as PNGs
python3 tools/ringdump.py bat_depth0     # save depth maps as PNGs (bright = near)
```

Python deps (`python3-numpy`, `python3-opencv`, `python3-tflite-runtime`) are
already installed on the Pi via apk. The depth worker looks for the MiDaS model
in `models/` here, then in the benchmark kit next door (`../bat-tim` or
`../tims-bat`); on the Pi:

```sh
mkdir -p models && cp ~/repos/tims-bat/models/midas_v21_small_256.tflite models/
```

The IMX708 sensor under QNX offers only 2304x1296 and 1536x864; the default is
1536x864 (`--width/--height` to override, `--iso/--shutter` for exposure).

## Testing on the Mac (no Pi needed)

```sh
make -C tests    # composite + ring unit tests, incl. C <-> Python interop
```

Full pipeline dry-run with a fake camera feeding the bat-tim test images
(use the kit's venv `../bat-tim/.venv/bin/python`, which has cv2 + a TFLite
runtime):

```sh
python tools/imgcam.py --ring /tmp/bat_cam0.ring            # terminal 1
python depth/depth_worker.py --cams /tmp/bat_cam0.ring      # terminal 2
python tools/ringdump.py /tmp/bat_depth0.ring --out /tmp    # terminal 3
```
