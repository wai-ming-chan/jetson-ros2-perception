#!/usr/bin/env bash
# Bag-replay regression test. Runs ON THE JETSON, inside the project container:
#
#     ./docker/run.sh bash /workspace/test/on_target/replay_regression.sh [bag]
#
# With no argument it records its own ~8 s fixture from the live camera first (the
# record subscription is what triggers the camera's lazy JPEG encoder). It then replays
# the bag through a fresh detector instance in an isolated /ci namespace -- bag topics
# are remapped so this is safe to run while a live stack is up, at the cost of GPU
# contention -- and asserts the detector's self-reported rate and latency.
#
# Thresholds via env: MIN_RATE (default 10 Hz), MAX_LAT (default 60 ms). The defaults
# are lenient enough to pass alongside a running stack; on a quiet bench use e.g.
# MIN_RATE=25 MAX_LAT=35.
#
# Why this is not in GitHub Actions: the inference path needs the Jetson's GPU and a
# locally-built TensorRT engine. CI (x86, no GPU) builds everything and runs the unit
# tests; this script is the hardware tier of the same suite.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ROS setup scripts read variables they never set (COLCON_TRACE, AMENT_TRACE_SETUP_FILES);
# under `set -u` that is a fatal unbound-variable error, so nounset is suspended for them.
set +u
source /opt/ros/humble/install/setup.bash
source /workspace/install/setup.bash
set -u

BAG="${1:-}"
ENGINE="${ENGINE:-/models/yolov8n_fp16.engine}"
MIN_RATE="${MIN_RATE:-10}"
MAX_LAT="${MAX_LAT:-60}"

cleanup() {
    # SIGINT, never SIGKILL: the detector holds CUDA state, and the habit matters more
    # than the necessity (see docs/runbook.md rule 1).
    [ -n "${PLAY_PID:-}" ] && kill -INT "$PLAY_PID" 2>/dev/null || true
    [ -n "${DET_PID:-}" ] && kill -INT "$DET_PID" 2>/dev/null || true
    [ -n "${REP_PID:-}" ] && kill -INT "$REP_PID" 2>/dev/null || true
    sleep 2
}
trap cleanup EXIT

if [ -z "$BAG" ]; then
    BAG=/tmp/replay_fixture
    rm -rf "$BAG"
    echo "--- recording a ~12 s fixture from the live camera ---"
    # --signal=INT: rosbag2 finalises its database on SIGINT. timeout's default SIGTERM
    # kills it mid-write and leaves an EMPTY bag that then "replays" in zero seconds --
    # observed as the player reopening the database ten times a second.
    timeout --signal=INT 12 ros2 bag record -o "$BAG"         /image_raw/compressed /camera_info || true
    sleep 1
fi
[ -d "$BAG" ] || { echo "FAIL: no bag at $BAG"; exit 1; }

# An empty fixture must fail loudly here, not as a mysterious no-reports timeout later.
MSGS=$(ros2 bag info "$BAG" 2>/dev/null | grep -oE "Messages:\s+[0-9]+" | grep -oE "[0-9]+" || echo 0)
echo "--- fixture: $MSGS messages ---"
if [ "${MSGS:-0}" -lt 50 ]; then
    echo "FAIL: fixture has only ${MSGS:-0} messages -- is the camera publishing,"          "and did the lazy JPEG encoder engage?"
    exit 1
fi

echo "--- starting isolated pipeline (namespace /ci) ---"
ros2 run image_transport republish compressed raw --ros-args \
    -r in/compressed:=/ci/image_raw/compressed -r out:=/ci/image_raw &
REP_PID=$!
ros2 run trt_detector trt_detector_node --ros-args \
    -r __node:=trt_detector_ci \
    -p image_topic:=/ci/image_raw \
    -p engine_path:="$ENGINE" \
    -p rate_report_interval:=4.0 > /tmp/detector_ci.log 2>&1 &
DET_PID=$!
sleep 5
kill -0 "$DET_PID" 2>/dev/null || {
    echo "FAIL: detector died on startup:"; tail -5 /tmp/detector_ci.log; exit 1; }

echo "--- replaying $BAG (looped to outlast collection) ---"
ros2 bag play "$BAG" --loop \
    --remap /image_raw/compressed:=/ci/image_raw/compressed &
PLAY_PID=$!

python3 "$HERE/collect_metrics.py" \
    --rate-topic /trt_detector_ci/publish_rate \
    --latency-topic /trt_detector_ci/latency_ms \
    --min-rate "$MIN_RATE" --max-latency "$MAX_LAT" --duration 15
