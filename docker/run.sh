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

# Preflight. Without this, a user whose shell lacks the docker group gets
# "docker: unknown server OS: ." -- Docker's way of saying its daemon query returned
# nothing, which is what a permission failure looks like at this stage. The real cause is
# unrecoverable from that message, so check explicitly.
if ! docker info >/dev/null 2>&1; then
    echo "error: cannot talk to the Docker daemon." >&2
    if ! id -nG | tr ' ' '\n' | grep -qx docker; then
        echo "  This shell's user is not in the 'docker' group." >&2
        echo "    sudo usermod -aG docker \$USER" >&2
        echo "  Group membership is fixed at login, so an already-open shell keeps its" >&2
        echo "  old groups. Start a new login shell, or run: newgrp docker" >&2
    else
        echo "  You are in the docker group, so check the daemon:" >&2
        echo "    systemctl status docker" >&2
    fi
    exit 1
fi

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
