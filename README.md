# jetson-ros2-perception

ROS 2 perception and deployment stack, validated on NVIDIA Jetson hardware.

> **Scope, stated honestly:** this is a perception and deployment stack running on real
> Jetson hardware. It is not a mobile robot — there is no chassis, no drivetrain, and no
> autonomous navigation. Every number below was measured on the device described in
> *Hardware*, not in simulation and not on a workstation.

## Hardware

| | |
|---|---|
| Board | Seeed reComputer **Industrial J3011** — Jetson Orin Nano 8GB, 6 cores, 7.3 GiB |
| Camera | Arducam **IMX477** on 2-lane MIPI CSI, `/dev/video0`, NVIDIA stock driver |
| Platform | **JetPack 5.1.3 / L4T R35.5.0**, Ubuntu 20.04.6, kernel 5.10.192-tegra, CUDA 11.4 |
| ROS | **Humble**, containerised (Ubuntu 20.04 has no native Humble) |
| Industrial I/O | Isolated CAN FD (DB9), RS-232/422/485, 4x DI / 4x DO, 2x GbE |

Two constraints from this hardware shape everything:

- The 2-lane CSI exposes **exactly two sensor modes** — `3840x2160@30` and `1920x1080@60`,
  both `RG10` raw Bayer. There is no 12 MP mode.
- Only **7W and 15W** power modes exist on JetPack 5. The 25W/MAXN SUPER mode is
  JetPack 6.2-exclusive, so benchmarks have two power points.

## Why containers

ROS 2 Humble requires Python 3.10 and has no official Ubuntu 20.04 binaries. JetPack 5.1.3
is Ubuntu 20.04. Containers are therefore not a convenience here — they are the only way
to run Humble on this image without reflashing, which would put the working camera driver
at risk. See `docs/platform-decision.md`.

## Quick start

```bash
docker build -t jetson-ros2-perception:latest -f docker/Dockerfile .
./docker/run.sh

# inside the container
colcon build --symlink-install
source install/setup.bash
ros2 launch jetson_camera camera.launch.py
```

From another shell, confirm frames are flowing:

```bash
ros2 topic hz /image_raw
```

## Packages

| Package | Purpose |
|---|---|
| `jetson_camera` | Argus (hardware ISP) → `sensor_msgs/Image` + `CameraInfo` |

Planned: `gpu_preproc` (CUDA debayer/undistort/resize), `trt_detector` (TensorRT
inference), `can_bridge` (SocketCAN ↔ ROS 2), `bag_tools` (recorder + replay harness).

## Benchmarks

Measured on the hardware above. *(To be filled as pipeline variants land.)*

| Pipeline | Capture path | Resolution | Power | FPS | Latency (ms) |
|---|---|---|---|---|---|
| Passthrough | Argus (HW ISP) | 1920x1080 | 15W | — | — |
| Passthrough | V4L2 raw + CUDA debayer | 1920x1080 | 15W | — | — |
| + TensorRT FP16 | Argus | 1920x1080 | 15W | — | — |

## License

Apache-2.0 — see [LICENSE](LICENSE).
