#!/usr/bin/env bash
# Launch the development container with camera + GPU access.
#
# Every flag below is load-bearing; see docs/ and PROGRESS.md for how each was verified.
#   --network host          ROS 2 DDS discovery
#   --ipc host              DDS shared-memory transport
#   --device /dev/video0    raw V4L2 path (RG10 Bayer)
#   -v /tmp/argus_socket    Argus path. nvarguscamerasrc FAILS without this even when the
#                           device node is present -- the device alone is not enough.
#
# --runtime nvidia is not passed because /etc/docker/daemon.json sets nvidia as the
# default runtime. Add it back if that ever changes.
set -euo pipefail

IMAGE="${IMAGE:-jetson-ros2-perception:latest}"
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAPTURES="${CAPTURES:-$HOME/captures}"

mkdir -p "$CAPTURES"

exec docker run -it --rm \
    --network host \
    --ipc host \
    --device /dev/video0 \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v "$WORKSPACE:/workspace" \
    -v "$CAPTURES:/captures" \
    -e DISPLAY="${DISPLAY:-}" \
    "$IMAGE" \
    "$@"
