#!/usr/bin/env bash
# Install the host-side settings this stack depends on. Run once, with sudo.
#
# These are NOT optional tuning: without them the stack is broken in ways that look
# like application bugs -- see docs/runbook.md and PROGRESS.md Issues 11 and 12.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "run with sudo: sudo $0" >&2
    exit 1
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

install -m 644 "$HERE/sysctl/60-ros2-dds.conf" /etc/sysctl.d/60-ros2-dds.conf
sysctl --system >/dev/null
echo "installed: DDS socket buffers (rmem_max=$(sysctl -n net.core.rmem_max))"

for unit in jetson-eee-off jetson-can0; do
    install -m 644 "$HERE/systemd/$unit.service" "/etc/systemd/system/$unit.service"
done
systemctl daemon-reload
for unit in jetson-eee-off jetson-can0; do
    systemctl enable --now "$unit.service" >/dev/null 2>&1 || \
        echo "  warning: $unit did not start (check: systemctl status $unit)"
    echo "installed + enabled: $unit.service"
done

echo
echo "verify:"
echo "  sysctl net.core.rmem_max          # expect 67108864"
echo "  ethtool --show-eee eth0           # expect: EEE status: disabled"
echo "  ip -details link show can0        # expect: state UP, bitrate 500000"
