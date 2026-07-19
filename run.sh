#!/bin/sh
# One-command launcher for the whole bathat pipeline on the Pi:
#
#   ./run.sh [flags]
#
# Starts, in dependency order, waiting for each stage's ring to appear:
#   1. ./bathat                      capture + viewfinder (VNC shows the 2x2
#                                    grid: cameras, depth heatmaps, circles)
#   2. undistort/undistort_worker.py fisheye rectification -> /bat_rect*
#   3. depth/depth_worker.py         MiDaS depth + blob detection -> /bat_depth*, /bat_det*
#   4. ./bat_audio                   detections -> directional hum (USB headphones)
#
# Flags are routed to the stage that owns them, e.g.:
#   ./run.sh --det-thresh 88 --master 0.4 --balance 0.3 --no-depth
# Logs: /tmp/bathat.<stage>.log   Ctrl-C stops every stage.

set -u

CAM_ARGS=""; UND_ARGS=""; DEPTH_ARGS=""; AUDIO_ARGS=""
while [ $# -gt 0 ]; do
    case "$1" in
        # depth worker: detection + geometry tuning
        --det-thresh|--min-area|--yaw0|--yaw1|--fov|--model|--threads)
            DEPTH_ARGS="$DEPTH_ARGS $1 $2"; shift 2 ;;
        # undistort worker
        --balance)
            UND_ARGS="$UND_ARGS $1 $2"; shift 2 ;;
        # audio process
        --master|--adev)
            AUDIO_ARGS="$AUDIO_ARGS $1 $2"; shift 2 ;;
        # capture daemon / viewfinder
        --iso|--shutter|--width|--height)
            CAM_ARGS="$CAM_ARGS $1 $2"; shift 2 ;;
        --no-display|--no-depth)
            CAM_ARGS="$CAM_ARGS $1"; shift ;;
        *)
            echo "run.sh: unknown flag: $1" >&2; exit 2 ;;
    esac
done

cd "$(dirname "$0")"

# Always run make: it no-ops when binaries are current, and prevents stale
# binaries from silently running old code after a git pull.
make bathat bat_audio || exit 1

PIDS=""
stop_all() {
    trap - INT TERM
    echo ""
    echo "run.sh: stopping"
    [ -n "$PIDS" ] && kill $PIDS 2>/dev/null
    wait 2>/dev/null
    exit 0
}
trap stop_all INT TERM

wait_for_ring() {
    _tries=0
    until [ -e "/dev/shmem/$1" ]; do
        _tries=$(( _tries + 1 ))
        if [ "$_tries" -gt 30 ]; then
            echo "run.sh: timed out waiting for /dev/shmem/$1 (check /tmp/bathat.*.log)" >&2
            stop_all
        fi
        sleep 1
    done
}

echo "run.sh: starting bathat (capture + viewfinder)"
./bathat $CAM_ARGS > /tmp/bathat.cam.log 2>&1 &
PIDS="$PIDS $!"
wait_for_ring bat_cam0

echo "run.sh: starting undistort worker"
python3 undistort/undistort_worker.py $UND_ARGS > /tmp/bathat.undistort.log 2>&1 &
PIDS="$PIDS $!"
wait_for_ring bat_rect0

echo "run.sh: starting depth worker (MiDaS + detection)"
python3 depth/depth_worker.py $DEPTH_ARGS > /tmp/bathat.depth.log 2>&1 &
PIDS="$PIDS $!"
wait_for_ring bat_det0

echo "run.sh: starting bat_audio (directional hum)"
./bat_audio $AUDIO_ARGS > /tmp/bathat.audio.log 2>&1 &
PIDS="$PIDS $!"

echo "run.sh: all stages up — logs in /tmp/bathat.*.log, Ctrl-C to stop"
wait
