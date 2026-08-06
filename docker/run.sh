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

# /var/lib/nvpmodel is mounted read-only so the operator console can report the
# active power mode; without it the panel shows "unknown" rather than 15W/7W.
# ~/models holds ONNX files and locally-built TensorRT engines (device-specific,
# never committed); trt_detector loads its engine from /models.

mkdir -p "$CAPTURES"

# X11 forwarding, for tools that need a window -- cameracalibrator above all, since its
# whole interface is an OpenCV window. Only wired up when a display actually exists, so
# headless runs are unaffected.
#
# Set DISPLAY and XAUTHORITY on the HOST before running this script; they are read here to
# decide what to mount. For the local GNOME desktop:
#     export DISPLAY=:1 XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
#
# Mounting the cookie is enough on its own -- `xhost +local:root` is NOT required, and is
# worth avoiding since it disables access control for every local process. Reach for it
# only if a cookie genuinely cannot be located.
GUI_ARGS=()
if [ -n "${DISPLAY:-}" ]; then
    GUI_ARGS+=(-e "DISPLAY=$DISPLAY")
    [ -d /tmp/.X11-unix ] && GUI_ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix)

    # Find the auth cookie. There is no ~/.Xauthority on this system: the local GNOME
    # session's cookie lives under /run/user/<uid>/gdm/. An `ssh -X` login does create
    # ~/.Xauthority, so check both and mount whichever exists.
    XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"
    if [ -f "$XAUTH" ]; then
        GUI_ARGS+=(-v "$XAUTH:/root/.Xauthority:ro" -e "XAUTHORITY=/root/.Xauthority")
    else
        echo "note: DISPLAY=$DISPLAY but no X cookie found (looked at $XAUTH)." >&2
        echo "      For the local desktop: export XAUTHORITY=/run/user/\$(id -u)/gdm/Xauthority" >&2
    fi
else
    echo "note: no DISPLAY -- GUI tools (e.g. cameracalibrator) will not work." >&2
fi

exec docker run -it --rm \
    -e CYCLONEDDS_URI=file:///workspace/docker/cyclonedds.xml \
    --network host \
    --ipc host \
    --device /dev/video0 \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v "$WORKSPACE:/workspace" \
    -v "$CAPTURES:/captures" \
    -v /var/lib/nvpmodel:/var/lib/nvpmodel:ro \
    -v "$HOME/models:/models" \
    "${GUI_ARGS[@]}" \
    "$IMAGE" \
    "$@"
