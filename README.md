# jetson-ros2-perception

The software base layer a robot needs before any autonomy is possible — sensors
publishing at full rate, drivers that recover instead of freezing, an industrial bus on
the wire, one-click data capture, and a live view of what the machine sees — built and
validated end to end on an NVIDIA Jetson.

Concretely:

- **Camera driver (`jetson_camera`)** — C++ ROS 2 node for the IMX477 over the Jetson
  hardware ISP (Argus/GStreamer). Publishes `sensor_msgs/Image` and `CameraInfo` at a
  sustained 1920×1080 @ 60 Hz, with capture-stall detection, automatic pipeline
  recovery, and fail-fast shutdown for supervised restart (launch respawn / systemd).
- **Operator console (`operator_console`)** — browser-based operations UI served from the
  board: live video, node and host telemetry (publish rate, CPU/GPU load, power mode,
  SoC temperature), CAN frame TX/RX, and rosbag2 record control. Python standard library
  only; runs fully headless (demo below).
- **CAN bus bring-up** — SocketCAN on the carrier's isolated CAN FD interface (`mttcan`,
  500 kbit/s), validated in loopback and integrated into the console for frame injection
  and traffic monitoring.
- **Containerized deployment** — ROS 2 Humble in Docker, pinned to the board's L4T
  release; a stock JetPack 5.1.3 device reaches camera streaming with a single build and
  launch sequence.
- **On-target benchmarking** — publisher-side instrumentation of frame rate and compute
  load across sensor modes and power profiles; results and methodology in the benchmark
  table below.

*Next:* TensorRT detection, CUDA preprocessing, a `can_msgs` bridge node, and a
bag-replay regression harness in CI.

> **Scope:** this stack runs on real Jetson hardware, but it is not deployed on a mobile
> robot — there is no drivetrain and no autonomous navigation. Every number below was
> measured on the hardware described under *Hardware*, not in simulation.

![Operator console demo](docs/media/operator-console.gif)

*The operator console live: YOLOv8 detections overlaid on the camera feed at ~30 Hz, CAN
frames injected and echoed on `can0`, host telemetry and rosbag recording — all served from
the board. ([full-quality mp4](docs/media/operator-console.mp4))*

## Hardware

| | |
|---|---|
| Board | Seeed reComputer **Industrial J3011** — Jetson Orin Nano 8GB, 6 cores, 7.3 GiB |
| Camera | Arducam **IMX477** on 2-lane MIPI CSI, `/dev/video0`, NVIDIA stock driver |
| Platform | **JetPack 5.1.3 / L4T R35.5.0**, Ubuntu 20.04.6, kernel 5.10.192-tegra, CUDA 11.4 |
| ROS | **Humble**, containerised (Ubuntu 20.04 has no native Humble) |
| Industrial I/O | Isolated CAN FD (DB9), RS-232/422/485, 4x DI / 4x DO, 2x GbE |

Two constraints from this hardware currently:

- The 2-lane CSI exposes **exactly two sensor modes** — `3840x2160@30` and `1920x1080@60`,
  both `RG10` raw Bayer. There is no 12 MP mode.
- Only **7W and 15W** power modes exist on JetPack 5. The 25W/MAXN SUPER mode is
  JetPack 6.2-exclusive, so benchmarks have two power points.

## Operations

[`docs/runbook.md`](docs/runbook.md) — deployment and field debugging: first-time setup on
a fresh device, a symptom index (camera won't start, subscriber receives nothing, Ethernet
slower than Wi-Fi, CAN transmits nothing), and the platform quirks that cost time. Written
from real incidents; `PROGRESS.md` holds the underlying evidence.

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

## Operator console

A browser-based control surface for the stack — camera, pipeline telemetry, CAN traffic,
and the controls to actually operate it.

```bash
ros2 launch operator_console console.launch.py     # then open http://<jetson>:8080
```

| Shows | Controls |
|---|---|
| Live MJPEG from `/image_raw` | Start/stop `rosbag2` recording, with live size and elapsed |
| Publisher rate, resolution, encoding | Snapshot the current frame to disk |
| Calibration loaded / not | Inject a CAN frame (id + payload) |
| Power mode, CPU, GPU, SoC temp, disk free | |
| Live CAN frame table with ID/DLC/data/flags | |

This is **not** a Foxglove or `rqt` replacement — those visualise topics and do it better.
The console exists for the part they don't do: triggering recordings, capturing stills and
putting frames on the bus, from one page, on a headless board.

Two details worth calling out:

- It reports the **publisher's** frame rate, which the camera node measures and publishes on
  `~/publish_rate` — not the rate the console itself receives. Timing your own arrivals
  measures your own consumption: `ros2 topic hz` read **47.8 Hz while the publisher was at
  60.0 Hz**, because deserialising ~6 MB frames in Python is slower than producing them.
  Both figures are shown and labelled, since the gap between them is itself informative.
- **No new dependencies.** `rclpy`, `cv2` and the Python standard library only. The
  container has no flask/fastapi and no ROS web packages, and with no Humble binaries for
  Ubuntu 20.04 every addition would have meant a source build.
- **The view consumes `/image_raw/compressed`** — JPEGs encoded in C++ by the camera's
  image_transport plugin — and serves the bytes untouched. The first design subscribed to
  raw `/image_raw`; deserialising 6.2 MB frames in Python capped the console at 13–50 Hz
  and was the real cause of a laggy view. ~200 KB JPEGs deserialise trivially, and the
  console never touches a pixel. Delivery is long-polled: each request names the last
  frame it saw and is answered when a newer one exists, so every round trip carries a
  fresh frame (measured: ~30 fps to one browser, zero duplicates).
- **The console subscribes only while someone is watching.** The compressed plugin is
  lazy, and its encode runs synchronously in the camera's capture loop — measured cost is
  60 → ~36 Hz at 1080p while subscribed. Five seconds after the last browser request the
  console unsubscribes and the pipeline returns to full rate. The tradeoff is printed in
  the panel itself: publisher rate dips while you watch, and recovers when you stop.

## Packages

| Package | Purpose |
|---|---|
| `jetson_camera` | Argus (hardware ISP) → `sensor_msgs/Image` + `CameraInfo` |

| `trt_detector` | YOLOv8 via TensorRT → `Detection2DArray` + compressed overlay, per-stage latency instrumentation |

Planned: `can_bridge` (SocketCAN ↔ `can_msgs`), `bag_tools` (recorder + replay harness).
CUDA preprocessing was planned and measured out: at 4.0 ms on CPU against a 33 ms frame
budget, a kernel would not pay for itself.

## Benchmarks

Measured on the hardware above. *(To be filled as pipeline variants land.)*

| Pipeline | Capture path | Resolution | Power | FPS | CPU |
|---|---|---|---|---|---|
| `/image_raw` (bgr8) | Argus (HW ISP) | 1920x1080@60 | 15W | **60.0** | 0.65 core, GPU idle |
| `/image_raw` (bgr8) | Argus (HW ISP) | 3840x2160@30 | 15W | **29.9** | 1.0 core, GPU idle |
| ↳ same, `videoconvert n-threads=1` | Argus (HW ISP) | 3840x2160@30 | 15W | 26.9 | one core pinned at 74% |
| `/image_raw` (bgr8) | Argus (HW ISP) | 1920x1080@60 | 7W | — | — |
| + TensorRT FP16 detection (`trt_detector`) | Argus | 1920x1080 | 15W | **29–30** (every frame) | 20.2 ms e2e: pre 4.0 + infer 12.1 + post 4.0; GPU 50% |
| Camera publisher with a raw subscriber attached | Argus | 1920x1080@60 | 15W | 29.3 | DDS serialisation of 6.2 MB frames is the cost |

**Model inference** (YOLOv8n, `trtexec`, batch 1, 640×640, TensorRT 8.5.2, 15W —
engines built on-device):

| Precision | GPU compute (median) | Throughput | Engine size |
|---|---|---|---|
| FP16 | **7.52 ms** | 130.5 qps | 8.1 MB |
| FP32 | 13.39 ms | 72.9 qps | 14 MB |

Rates are measured **publisher-side**, by the node counting its own frames. `ros2 topic hz`
under-reports badly here — it is a Python subscriber deserialising ~6.2 MB per frame, so it
measures its own throughput, not the pipeline's. It read 47.8 Hz against an actual 60.0 Hz.

## License

Apache-2.0 — see [LICENSE](LICENSE).
