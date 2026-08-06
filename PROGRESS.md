# Project Progress Log

This document tracks all progress, commands executed, issues encountered, and solutions found.
Use this for reproduction on a fresh system.

**Only verified commands belong here.** Anything not yet run on real hardware does not go
in this file.

This is the raw engineering log behind the claims in the README: what was measured, what
broke, and the commands that produced each answer — including the wrong turns, because the
wrong turns are where the reusable lessons are. For the distilled, actionable form see
[`docs/runbook.md`](docs/runbook.md).

---

## Hardware

| Date | Item | Model | Status |
|------|------|-------|--------|
| 2026-02-03 | Jetson board | Seeed reComputer Industrial J3011 (Orin Nano 8GB) | Received, verified 2026-08-05 |
| 2026-02-03 | Camera | Arducam IMX477 (12MP) | Received, detected and enumerated 2026-08-05 |

---

## Verified System Inventory — 2026-08-05

Captured over SSH. This is the ground truth for every design decision in this project.

### Platform

| Property | Value |
|---|---|
| L4T | R35.5.0 (`R35 (release), REVISION: 5.0, GCID: 35550185, DATE: Tue Feb 20 2024`) |
| JetPack | 5.1.3-b29 |
| OS | Ubuntu 20.04.6 LTS |
| Kernel | 5.10.192-tegra (aarch64) |
| CUDA | 11.4 (`/usr/local/cuda-11.4`) — `nvcc` **not** on default PATH |
| Python | 3.8.10 |
| CPU / RAM | 6 cores / 7.3 GiB |
| Storage | NVMe 119.2 GB, `/` = 116 GB, 94 GB free (16% used) |
| Docker | 26.1.3 |

### Board identity

```
TNSPEC          3767-300-0003-T.1-1-0-recomputer-orin-industrial-
COMPATIBLE_SPEC 3767--0003--1--recomputer-orin-industrial-
```

`3767-0003` = Orin Nano 8GB module; `recomputer-orin-industrial` = Seeed Industrial carrier.

> **Do not trust `/proc/device-tree/model`** — it reports the generic string
> `NVIDIA Orin Nano Developer Kit`. Only `/etc/nv_boot_control.conf` identifies the
> actual carrier board, which is what determines the correct BSP.

### Camera — WORKING, no driver install required

```
vi-output, imx477 10-001a (platform:tegra-capture-vi:2)  ->  /dev/video0
NVIDIA Tegra Video Input Device (platform:tegra-camrtc-ca) -> /dev/media0
```

Bound at I²C address `0x1a` on bus 10, using NVIDIA's **stock** IMX477 driver.
No `jetson-io.py` configuration and no Arducam kernel package were needed.

Available modes (`v4l2-ctl -d /dev/video0 --list-formats-ext`):

| Format | Resolution | Frame rate |
|---|---|---|
| `RG10` (10-bit Bayer RGGB) | 3840x2160 | 30 fps |
| `RG10` (10-bit Bayer RGGB) | 1920x1080 | 60 fps |

**There is no 4056x3040 / 12 MP mode.** The Industrial carrier exposes 2-lane 15-pin CSI
connectors, and the device tree publishes only the two modes above. Any plan assuming
full-sensor readout is invalid on this hardware.

Exposed V4L2 controls: `group_hold`, `sensor_mode` (0–2), `gain` (16–357),
`exposure` (13–683710), `frame_rate` (2000000–30000000).

**No `focus_absolute` control** — the motorised focus of the Arducam autofocus module is
not reachable through V4L2 with the stock driver. Manual focus only. A userspace VCM
driver over `/dev/i2c-*` would be needed, and is not required by anything here.

Argus is available and running: `nvarguscamerasrc` is registered with GStreamer and
`nvargus-daemon` is `active`. Both capture paths are therefore open —
hardware ISP via Argus, or raw `RG10` Bayer via V4L2 into a custom CUDA kernel.

### Industrial I/O

| Interface | State |
|---|---|
| CAN (`mttcan`) | Module present at `/lib/modules/5.10.192-tegra/kernel/drivers/net/can/mttcan/native/mttcan.ko`, **not loaded**, no `can0` yet |
| UARTs | `/dev/ttyTHS0`, `/dev/ttyTHS1`, `/dev/ttyTHS3`, `/dev/ttyTHS4` |
| Network interfaces | `eth0`, `eth1` (2x GbE), `rndis0`, `usb0`, `l4tbr0` (USB device mode), `docker0`, `dummy0` |

### Power modes

```
< POWER_MODEL ID=0 NAME=15W >
< POWER_MODEL ID=1 NAME=7W >
```

Only 7W and 15W. The 25W / MAXN SUPER mode is JetPack 6.2-exclusive for Orin Nano and is
**not** available on this image. Benchmarks therefore have two power points, not three.

### User and permissions

```
groups: jetson adm cdrom sudo audio dip video plugdev render i2c lpadmin gdm
        sambashare weston-launch gpio
```

Note `i2c`, `gpio`, `video`, `render` are present — but **`docker` is not**.

---

## Phase 1: Environment Setup

### 1.1 Network and SSH — COMPLETE (2026-08-05)

Topology used: Mac and Jetson connected **directly** by Ethernet cable, no router on the link.

**Mac side** (`en0`, manually configured):

```bash
# Inspect
networksetup -getinfo "Ethernet"     # -> Manual, 192.168.50.2, Router: (null)
ifconfig en0 | grep -E 'media|status|inet '
```

**Jetson side** — reuse the existing NetworkManager profile rather than adding a new one,
to avoid two profiles racing for `eth0` on carrier-up:

```bash
sudo nmcli con mod "Wired connection 1" \
    ipv4.method manual \
    ipv4.addresses 192.168.50.3/24 \
    ipv4.gateway "" \
    connection.autoconnect yes

sudo nmcli con mod "Wired connection 2" connection.autoconnect no

sudo nmcli con up "Wired connection 1" ifname eth0
ip -br addr show eth0        # -> eth0  UP  192.168.50.3/24
```

**SSH server** (was already installed and listening):

```bash
sudo systemctl enable --now ssh
sudo ss -tlnp | grep :22     # -> LISTEN 0 128 0.0.0.0:22 ... sshd
```
Server version: `OpenSSH_8.2p1 Ubuntu-4ubuntu0.13`.

**Mac `~/.ssh/config`:**

```
Host jetson
    HostName 192.168.50.3
    User jetson
    IdentityFile ~/.ssh/id_ed25519
    ServerAliveInterval 30
    ServerAliveCountMax 6
```

```bash
chmod 600 ~/.ssh/config
ssh-copy-id jetson
ssh jetson
```

`ServerAliveInterval` is not cosmetic — a `colcon build` on this board runs long enough
that a silent link drop kills it with no useful error.

**Verification from the Mac:**

```bash
ping -c 3 192.168.50.3                      # 0% loss, ~0.6 ms
nc -z -v -G 3 192.168.50.3 22               # succeeded
ssh-keyscan -T 5 192.168.50.3               # returns host keys
```

#### Issues Encountered

**Issue 1 — `eth0` UP but with no IP address at all**

- **Symptom:** From the Mac, an ARP sweep of the whole `192.168.50.0/24` returned
  `(incomplete)` for all 254 addresses, and `ping6 ff02::1%en0` (IPv6 all-nodes multicast)
  returned replies from the Mac only. Yet `ifconfig en0` showed
  `status: active, media: 1000baseT full-duplex` — the link was physically negotiated.
- **Root cause:** `ip -br addr` on the Jetson showed `eth0 UP` with **no address**.
  NetworkManager attempted DHCP, found no server (the Mac is not one), and left the
  interface unconfigured rather than falling back to a link-local address.
- **Solution:** Static IP on both ends, same subnet — see the `nmcli` block above.
- **Lesson:** an interface with no IP cannot answer ARP *or* IPv6 multicast, so it is
  invisible to every host-discovery method while still showing an active link.
  `ip -br addr` on the device settles in one second what a network scan cannot.

**Issue 2 — two `Wired connection` profiles, neither active**

- **Symptom:** `nmcli con show` listed `Wired connection 1` and `Wired connection 2`,
  both unbound; `nmcli device status` showed `eth0 ethernet disconnected`.
- **Root cause:** The Industrial J3011 has **two** GbE ports (`eth0`, `eth1`), so
  NetworkManager auto-created a profile per interface.
- **Solution:** Configured profile 1 for `eth0` and set
  `connection.autoconnect no` on profile 2 so it cannot race for the interface.

**Issue 3 — `docker info` fails as user `jetson`**

- **Root cause:** user is not in the `docker` group.
- **Solution:** `sudo usermod -aG docker jetson`, then log out and back in.
  (Grants effectively-root access — acceptable on a dev board, worth knowing.)

#### Notes / gotchas recorded

- `ipv4.never-default` was deliberately **not** set on the wired profile. It is often
  suggested for direct links, but it silently prevents a default route from being
  installed if the same profile is later switched to DHCP on a router — presenting as
  "no internet" on an otherwise healthy link.
- `rndis0` / `usb0` / `l4tbr0` exist, so the board is also reachable over USB device mode
  (typically `192.168.55.1`) without any Ethernet configuration at all.

### 1.1b Internet access via USB Wi-Fi dongle — COMPLETE (2026-08-05)

A USB Wi-Fi dongle was added, giving the board **two independent network paths**. No
Ethernet reconfiguration was needed — the direct cable stays as the SSH path.

```
eth0    UP  192.168.50.3/24    <- direct Mac link, static, no gateway (by design)
wlan0   UP  192.168.0.200/24   <- dongle, DHCP, holds the default route
default via 192.168.0.1 dev wlan0 proto dhcp metric 600
```

`wlan0` came up under NetworkManager with no manual configuration.

**Verified reachability:**

```bash
ping -c 2 8.8.8.8                             # 0% loss
getent hosts archive.ubuntu.com               # resolves
wget -S --spider https://registry-1.docker.io/v2/   # HTTP/1.1 401 -> reachable
wget -S --spider https://nvcr.io/v2/                # HTTP/1.1 401 -> reachable
wget -O /dev/null http://archive.ubuntu.com/ubuntu/ls-lR.gz   # 9.13 MB/s (~73 Mbit/s)
```

At 9 MB/s a ~4 GB ROS 2 container image pulls in well under ten minutes.

**Do not add a gateway to `eth0`.** It would compete with the real default route on
`wlan0`. The two paths are deliberately separate: SSH does not depend on the dongle, and
internet does not depend on the cable.

#### Issue 4 — container registries appeared "unreachable"

- **Symptom:** A reachability loop using `wget --spider https://registry-1.docker.io/`
  reported failure for both Docker Hub and `nvcr.io`, while `archive.ubuntu.com` passed.
- **Root cause:** **Test error, not a network fault.** Container registries answer
  unauthenticated requests with `401 Unauthorized`; `wget --spider` treats any non-2xx
  response as failure. The 401 actually *proves* TCP + TLS reachability.
- **Solution:** Re-test with `wget -S --spider https://registry-1.docker.io/v2/` and read
  the status line directly — `HTTP/1.1 401 Unauthorized` is the healthy result.
- **Lesson:** when probing an authenticated endpoint, check the *status line*, not the
  exit code. A definitive test is an actual `docker pull hello-world`.

### 1.2 Camera Driver — COMPLETE, no action required (2026-08-05)

The IMX477 was already detected on first inspection. See the inventory above.
Nothing was installed. Contrast with the documented failure mode on JetPack 6.2, where
Arducam's installer aborts with `No DTB found for ...` on Seeed carrier boards —
this is a significant reason to stay on JetPack 5.1.3 (see "Why containers" in the
README).

#### Verification commands

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
v4l2-ctl -d /dev/video0 --list-ctrls
gst-inspect-1.0 nvarguscamerasrc | head -5
systemctl is-active nvargus-daemon
```

### 1.3 First capture — COMPLETE (2026-08-05)

- [x] Capture a still via Argus (from inside the container — see Phase 2)
- [x] Obtain a correctly exposed image — resolved, see Issue 5
- [ ] Lock manual focus against a detailed target at working distance
- [ ] Capture raw `RG10` Bayer via V4L2
- [ ] Confirm 1080p60 and 4K30 both sustain their rated frame rate

Argus negotiated the sensor at **1920x1080 @ 59.999999 fps** and produced valid JPEGs.
The capture path is confirmed working end to end.

#### Issue 5 — captured frames are black

- **Symptom:** First capture (`num-buffers=1`) produced a 407 KB JPEG that was black with
  faint noise.
- **First cause — AE had not converged.** `num-buffers=1` returns the very first frame,
  captured before Argus auto-exposure adapts. Capturing 60 frames and keeping the last
  changed the result completely.
- **Second cause — essentially no light reaching the sensor.** With AE converged the frame
  became full-scale *amplified noise* (gain driven to maximum), and a control shot with
  forced moderate settings (`exposuretimerange="20000000 33000000" gainrange="8 16"`,
  30 fps) came back black with only sparse hot pixels. Both point at the optical path,
  not the software.
- **Status:** **RESOLVED — the lens cap was still on.** With it removed, the same 60-frame
  command produced a correctly exposed 152 KB JPEG of a real scene, with converged AE and
  neutral colour. Note the correctly exposed frame is *smaller* than the black noise
  frame (152 KB vs 1.71 MB), which is the point of Lesson 2 below.
- **Lesson 1:** always discard startup frames when sanity-checking an Argus pipeline;
  `num-buffers=1` is misleading. Use `num-buffers=60 ! multifilesink` and inspect the last.
- **Lesson 2:** **do not judge image content by JPEG file size.** The AE-converged noise
  frame was 1.71 MB — 4x the black frame — purely because noise has high entropy and
  compresses badly. Bigger file read as "more detail" and was wrong. Look at the pixels.

**Working capture commands (verified):**

```bash
# Let AE converge, keep the last frame
gst-launch-1.0 -q nvarguscamerasrc num-buffers=60 \
  ! 'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=60/1' \
  ! nvjpegenc ! multifilesink location=/captures/f_%03d.jpg

# Force exposure/gain (diagnostic; 33ms needs framerate <= 30)
gst-launch-1.0 -q nvarguscamerasrc num-buffers=30 \
  exposuretimerange="20000000 33000000" gainrange="8 16" ispdigitalgainrange="4 8" \
  ! 'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' \
  ! nvjpegenc ! multifilesink location=/captures/forced_%03d.jpg
```

---

## Phase 2: ROS 2 in Docker — CONTAINER VERIFIED (2026-08-05)

### 2.1 Docker access

```bash
sudo usermod -aG docker jetson     # then log out and back in
docker ps                          # verify, no sudo needed
```

`Default Runtime` is still `runc`; `nvidia` is registered but not default. Passing
`--runtime nvidia` per-run works. Setting `"default-runtime": "nvidia"` in
`/etc/docker/daemon.json` is still **pending** — it is required for CUDA to be visible
during `docker build`, which will matter for the GPU preprocessing node.

### 2.2 Image selection

`dustynv/ros` publishes ROS 2 Humble images built for L4T. Highest available R35 tag is
**r35.4.1**; this host is **r35.5.0**. The minor skew is fine — the NVIDIA container
runtime mounts host L4T libraries in — but it is the first thing to suspect on any odd
CUDA/driver error.

```bash
docker pull dustynv/ros:humble-desktop-l4t-r35.4.1    # 6.19 GB pull, 12.9 GB on disk
```

Chose `humble-desktop` over `humble-ros-base` (5.55 GB): 94 GB free, and the extra ~640 MB
includes rviz2 and demo tooling, saving `apt` round-trips inside the container.

**Container is Ubuntu 20.04.6 / Python 3.8.10** — dustynv builds Humble *from source* on
the L4T base rather than using Ubuntu 22.04 binaries. ROS Python nodes therefore run on
3.8, not the 3.10 that upstream Humble assumes. Watch for packages using 3.10-only syntax.

### 2.3 Verified run command

```bash
docker run -it --rm \
  --runtime nvidia \
  --network host \
  --ipc host \
  --device /dev/video0 \
  -v /tmp/argus_socket:/tmp/argus_socket \
  -v /home/jetson/captures:/captures \
  dustynv/ros:humble-desktop-l4t-r35.4.1
```

Every flag is load-bearing:

| Flag | Why |
|---|---|
| `--runtime nvidia` | GPU/CUDA access (until `default-runtime` is set) |
| `--network host` | ROS 2 DDS discovery |
| `--ipc host` | DDS shared-memory transport |
| `--device /dev/video0` | V4L2 raw Bayer path |
| `-v /tmp/argus_socket` | Argus path — **`nvarguscamerasrc` fails without it**, even with the device node |

Verified inside the container: `ROS_DISTRO=humble`, 307 ROS packages, `/dev/video0`
present, `nvarguscamerasrc` registered in GStreamer, `/usr/local/cuda-11.4` and
`libcuda.so` visible, and a real Argus capture written to the host volume.

Notes:
- `v4l2-ctl` is **not** installed in the image (`apt install v4l-utils` if needed).
- Files written to mounted volumes are owned by **root** (container runs as root).
  Use `--user` or `chown` afterwards if this becomes annoying.

### 2.4 Package availability inside the image

Checked before writing any code, which saved a wasted build:

| Dependency | Status |
|---|---|
| `rclcpp`, `sensor_msgs`, `image_transport`, `camera_info_manager` | PRESENT |
| `cv_bridge`, `image_pipeline`, `camera_calibration_parsers` | PRESENT (so `camera_calibration` works) |
| `colcon`, `cmake`, `g++` | PRESENT |
| **GStreamer `-dev` headers** | **MISSING** — runtime libs only |

`gst-inspect-1.0` working does **not** imply you can compile against GStreamer. The
`appsink` API needs `libgstreamer1.0-dev` and `libgstreamer-plugins-base1.0-dev`, which
are installed in `docker/Dockerfile`.

There are **no ROS 2 Humble debs for Ubuntu 20.04 (focal)** — `apt install ros-humble-*`
resolves nothing. That is why the base image builds ROS from source, and why every ROS
dependency must either already exist in the image or be built in the workspace.

#### Issue 6 — `docker build` fails: expired ROS apt signing key

- **Date:** 2026-08-05
- **Symptom:**
  ```
  W: GPG error: http://packages.ros.org/ros2/ubuntu focal InRelease: The following
     signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics
  E: The repository 'http://packages.ros.org/ros2/ubuntu focal InRelease' is not signed.
  ```
  `apt-get update` returns 100 and the build aborts before installing anything.
- **Root cause:** the base image is ~2 years old and ships an apt source for
  `packages.ros.org` whose signing key has since expired. Any `apt-get update` in a
  derived image fails, even when installing only Ubuntu packages.
- **Solution:** delete the source rather than re-keying it — it cannot supply anything
  useful, since there are no Humble binaries for focal:
  ```dockerfile
  RUN rm -f /etc/apt/sources.list.d/ros*.list
  ```
- **Lesson:** when a pinned old base image fails at `apt-get update`, check for expired
  third-party repo keys before suspecting the network or your own Dockerfile. Ask whether
  the failing repo is needed at all — deleting it is more durable than importing a new key
  that will expire too.

#### Issue 7 — `ros2 topic hz` is not a valid instrument for large images

- **Date:** 2026-08-06
- **Symptom:** `/image_raw` appeared to publish at 30 Hz, then 47.8 Hz, against a sensor
  configured for 60 fps. Two sessions were nearly spent optimising the "slow" pipeline.
- **Root cause:** `ros2 topic hz` is a **Python** subscriber that deserialises every
  message. At 1920x1080 `bgr8` each frame is ~6.2 MB, so 60 fps is ~373 MB/s through
  `rclpy`. It cannot keep up, and reports its own consumption rate as if it were the
  publisher's. Co-locating it in the node's container made it worse still, because it then
  competed with the node for CPU (30 Hz vs 47.8 Hz for the identical pipeline).
- **Solution:** instrument the publisher directly. `argus_camera_node` now counts frames
  and logs its own rate every `rate_report_interval` seconds. It reports a steady
  **60.0 Hz**, proving the pipeline was always at line rate.
- **Corroborating evidence:** node container used only 64.8% of one core and
  `GR3D_FREQ` was 0% — nothing was saturated, which never fitted the "pipeline is slow"
  story and should have been the earlier clue.
- **Lesson:** never accept a rate reported by a tool that must consume the data to measure
  it, unless you have shown the consumer is faster than the producer. Before optimising,
  confirm *something is actually saturated*; an idle CPU and idle GPU alongside a "slow"
  pipeline means the measurement is wrong, not the code.

**Issues Encountered:**
- [ ]

### 1.4 Intrinsic calibration — DONE, but low quality (2026-08-06)

Calibrated at 1920x1080 with `camera_calibration` (8x6 interior corners, checkerboard
displayed on a laptop screen), 43 samples. Result installed as
`config/imx477_1080p.yaml`; source images kept in `calibrationdata.tar.gz` (gitignored).

```
fx = 1285.45   fy = 1286.71      ratio 1.001 -> square pixels, healthy
cx =  961.39   cy =  553.83      ideal (960, 540); cx 1.4 px off, cy 13.8 px off
D  = [0.0118, -0.0389, -0.0008, 0.0016, 0]     low distortion
```
Derived: HFOV ~73.5 deg, VFOV ~45.5 deg, effective focal ~4.2 mm.

**Quality: not good enough for metric work.** Independently re-solved from the 43 saved
images with OpenCV (k3 free, so not identical to cameracalibrator's fixed-k3 model):

| | all 37 detected views | worst 6 removed |
|---|---|---|
| overall RMS | **3.09 px** | 1.83 px |
| median per-view RMS | 0.73 px | 0.54 px |
| p90 per-view RMS | 6.25 px | 3.77 px |

A good calibration is under ~0.5 px. Removing outliers does not rescue it, so the corner
data is soft throughout, not just in a few frames. The worst views are **consecutive**
(`left-0033`..`left-0037`, plus `0024`), suggesting something degraded mid-session --
focus drift, motion during capture, or glare appearing as the angle changed.

Probable cause: **the target was displayed on a screen.** The display's pixel grid beats
against the sensor's to produce moire, and glass reflects room lights; both soften corner
localisation invisibly. Printed matte paper on rigid backing is materially better.

Usable as a placeholder (fx is right to ~0.5%), but **redo before any AprilTag pose or
metric 3D work** -- 3 px reprojection error becomes centimetres of position error at range.

#### Gotchas found while installing the result

- `cameracalibrator` writes `camera_name: narrow_stereo`. `camera_info_manager` compares
  this against the name the node registers (`imx477`) and rejects a mismatch, presenting
  as "no calibration" rather than an error. Must be edited.
- **Intrinsics are resolution-specific.** These are 1080p-only; at 3840x2160 every value
  roughly doubles. Loading the wrong one produces silently wrong geometry, hence the
  `_1080p` suffix in the filename.
- COMMIT fails with `Not available`: the calibrator calls `/camera/set_camera_info`, while
  the node advertises `/set_camera_info`. Fix properly by running the node in a `/camera`
  namespace. SAVE works regardless, so this is not blocking.
- **SAVE writes inside the container** (`/tmp/calibrationdata.tar.gz`), which `--rm`
  destroys on exit. Recover with `docker cp <container>:/tmp/calibrationdata.tar.gz .`
  *before* leaving, or point `camera_info_url` at a bind-mounted path first.
- `camera_info_manager` loads **lazily**, on the first `getCameraInfo()` call in `publish()`.
  A node that never captures a frame never logs whether the calibration loaded.

### 2.5 Operator console

#### Issue 8 — camera view lagged by several seconds in the web console

- **Date:** 2026-08-06
- **Symptom:** the MJPEG view ran visibly behind the world, first choppy, then several to
  ten seconds delayed. The publisher held 60 Hz throughout, so the pipeline was fine.
- **Three separate causes, found by measuring rather than guessing:**

  1. **A hardcoded 15 fps cap** in the stream loop, which also slept the full interval
     after every frame, adding up to 66 ms of latency. Encoding was never the limit:
     measured ~1.0 ms resize + ~6.4 ms JPEG at 960x540, roughly 117 fps of headroom.
  2. **Frames were queued, not dropped.** The loop wrote every frame regardless of whether
     the client was consuming. Undrained data sits in the kernel send buffer, which allows
     up to 4 MB here (`/proc/sys/net/ipv4/tcp_wmem`) — over 100 JPEGs. MJPEG displays in
     order, so the viewer fell steadily further behind. Observed: `Send-Q` of 230 KB on
     `ss -tn state established '( sport = :8080 )'`.
  3. **Dead clients were never reaped.** Every page reload opens another `/stream.mjpg`,
     and browsers do not always close the old one. Seven connections and 24 threads were
     observed after a few reloads, each stale stream still encoding.
- **Solution:**
  - Raise the cap (`stream_fps`, default 60) and poll on 2 ms granularity so a fresh frame
    goes out as soon as it is allowed rather than waiting out a fixed sleep.
  - Cap `SO_SNDBUF` to 128 KB and check `select()` for writability before sending; when the
    client is behind, **discard the frame instead of buffering it**.
  - Bound concurrent streams (`max_streams`, default 2) plus a stall watchdog
    (`stream_stall_timeout`, default 5 s).
- **Lesson 1:** for live video, dropping a frame always beats delivering a stale one. Any
  path that can queue faster than it drains converts throughput into latency.
- **Lesson 2:** a loop that only ever *checks* writability never *attempts* a write, so it
  can never observe the `BrokenPipeError` meant to end it — a dead client with a full
  buffer would spin for the life of the process. Every wait needs a timeout that is not
  the happy path.
- **Verified:** zombie stream reaped within 9 s; `Send-Q` 0 for a client that keeps up;
  28–39 fps delivered with the publisher steady at 60 Hz.


## Phase 3: Perception + Benchmarks — NOT STARTED

---

## Phase 4: Data Tooling, CI, Deployment — NOT STARTED

---

## Phase 5: CAN / Industrial I/O — INTERFACE UP (2026-08-06)

### 5.1 Feasibility check (do this before anything else)

The driver existing means nothing if the device tree does not enable the controller —
the usual failure on third-party carriers.

```bash
for d in /proc/device-tree/*can*/; do
  echo "$d $(tr -d '\0' < $d/status)  $(tr -d '\0' < $d/compatible)"
done
```
```
/proc/device-tree/mttcan@c310000/  okay      nvidia,tegra194-mttcan   <- CAN0, usable
/proc/device-tree/mttcan@c320000/  disabled  nvidia,tegra194-mttcan   <- CAN1, off
```

One controller enabled, matching the single DB9 on the Industrial carrier.
`can-utils` (`candump`, `cansend`, `cangen`) was already installed.

### 5.2 Bring-up — VERIFIED

```bash
sudo modprobe mttcan
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 500000 loopback on
sudo ip link set can0 up
ip -details link show can0
```

Result:

```
34: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP qlen 10
    can <LOOPBACK> state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 0
    bitrate 500000 sample-point 0.870
    tq 20 prop-seg 43 phase-seg1 43 phase-seg2 13 sjw 1
```

`ERROR-ACTIVE` is the healthy state, not a fault — it means the controller is
participating normally. Sample point 0.870 is the standard automotive value.

### 5.3 Loopback: what it does and does not prove

A CAN transmission must be **acknowledged by another node**, so a lone device on a bus
cannot send anything at all. Loopback makes the controller feed its own transmissions back
into its receive path, which is why a single-node test is possible.

- **Proves:** driver loaded, interface configured, SocketCAN send/receive path, and
  application code end to end.
- **Does NOT prove:** transceiver, wiring, termination, or real arbitration/ACK. Those
  need a second node. Say so in the README rather than implying a full bus bring-up.

Verified through the operator console's `/api/can/send` endpoint:

```
send 123 DEADBEEF          -> id=123 dlc=4 data=DEADBEEF
send 1A0 0102030405060708  -> id=1A0 dlc=8 data=0102030405060708
send 7FF AA                -> id=7FF dlc=1 data=AA
```

**Every frame arrives twice**, which is expected and not a defect. Two loopback mechanisms
stack: SocketCAN's *software* loopback (on by default) delivers locally-transmitted frames
to other sockets on the same interface, and the controller's *hardware* loopback echoes TX
into RX. With `loopback off` on a real bus there is exactly one copy. Deliberately not
deduplicated — periodic status frames are legitimately identical, and collapsing them
would hide real bus behaviour.

### 5.4 Not yet done

- [ ] Persist across reboot: `/etc/modules-load.d/mttcan.conf` plus a systemd unit or
      `systemd-networkd` link file for the bitrate. Currently manual after every boot.
- [ ] Dedicated `can_bridge` node publishing proper ROS messages (`can_msgs` is **not** in
      the container image and there are no Humble debs for focal, so it needs either a
      source build or a small local message package).
- [ ] Test against a second CAN node to validate the transceiver.

---

## Useful Commands Reference

### System information

```bash
cat /etc/nv_tegra_release                  # L4T version: R35.x = JP5, R36.x = JP6
lsb_release -d                             # 20.04 = Foxy-era, 22.04 = Humble-native
uname -rm
apt-cache show nvidia-jetpack | grep Version
grep -E 'TNSPEC|COMPATIBLE_SPEC' /etc/nv_boot_control.conf   # real carrier identity
```

### Performance monitoring

```bash
sudo tegrastats                            # GPU/CPU/RAM/power, live
sudo nvpmodel -q                           # current power mode
sudo nvpmodel -m 0                         # 15W
sudo nvpmodel -m 1                         # 7W
sudo jetson_clocks                         # pin clocks to max (disables DVFS)
sudo pip3 install jetson-stats && jtop     # optional dashboard
```

### CUDA

```bash
export PATH=/usr/local/cuda/bin:$PATH      # nvcc is not on PATH by default
nvcc --version
```

### Camera debugging

```bash
dmesg | grep -iE 'imx477|nvcsi|tegra-capture'
v4l2-ctl -d /dev/video0 --all
```

### Remote access

```bash
ssh jetson                                 # alias defined in ~/.ssh/config on the Mac
rsync -av --delete ./ jetson:~/ws/         # push a working tree
```

---

## Troubleshooting Log

### Issue Template

```
**Date:** YYYY-MM-DD
**Phase:** X.X
**Problem:** Description
**Symptom:**
\`\`\`
paste observed output here
\`\`\`
**Root Cause:**
**Solution:**
\`\`\`bash
commands that fixed it
\`\`\`
**Lesson:** what to check first next time
**References:** URLs that helped
```

---

## Performance Benchmarks

Two power modes only (7W / 15W) — 25W is not available on JetPack 5.1.3.

| Pipeline | Capture path | Resolution | Power mode | FPS | Latency (ms) | Notes |
|---|---|---|---|---|---|---|
| ROS 2 `/image_raw` (bgr8) | Argus (HW ISP) | 1920x1080@60 | 15W (`pmode:0000`) | **60.0** | - | **Valid, publisher-side** (node-instrumented, 2026-08-06): 300–302 frames per 5.0 s window, stable. Node container CPU **64.8% of one core**; `GR3D_FREQ 0%` (GPU idle — Argus uses the ISP). Sustains the sensor's full rate; no capture bottleneck at 1080p. |
| ~~same~~ | ~~Argus~~ | ~~1920x1080@60~~ | ~~15W~~ | ~~47.8~~ | - | **Superseded — not a pipeline number.** `ros2 topic hz` in a separate container. Subscriber-limited. |
| ~~same~~ | ~~Argus~~ | ~~1920x1080@60~~ | ~~default~~ | ~~30.1~~ | - | **Superseded — not a pipeline number.** `ros2 topic hz` co-located in the node's own container; contention on top of subscriber limit. |
| ROS 2 `/image_raw` (bgr8) | Argus (HW ISP) | 3840x2160@30 | 15W | 26.9 | - | `videoconvert n-threads=1` (GStreamer default). **11% short of line rate.** Container CPU 0.92 core, with one core pinned at 73–74% and the other five near-idle — a single-threaded stage, not a saturated board. |
| ROS 2 `/image_raw` (bgr8) | Argus (HW ISP) | 3840x2160@30 | 15W | **29.9** | - | **`videoconvert n-threads=0` (all cores). Full rate recovered.** One-word change; no CUDA required. |
| Passthrough | Argus (HW ISP) | 1920x1080@60 | 7W | - | - | TODO |
| Passthrough | Argus (HW ISP) | 1920x1080 | 7W | - | - | TODO |
| Passthrough | Argus (HW ISP) | 3840x2160 | 15W | - | - | TODO |
| Passthrough | V4L2 raw RG10 + CUDA debayer | 1920x1080 | 15W | - | - | TODO |
| Passthrough | V4L2 raw RG10 + CPU debayer | 1920x1080 | 15W | - | - | TODO |
| + TensorRT detect FP32 | Argus | 1920x1080 | 15W | - | - | TODO |
| + TensorRT detect FP16 | Argus | 1920x1080 | 15W | - | - | TODO |
| + TensorRT detect FP16 | Argus | 1920x1080 | 7W | - | - | TODO |

Record `tegrastats` output alongside each row.

### Model inference — trtexec, batch 1, 640x640 input, 15W (2026-08-06)

YOLOv8n (opset 12 ONNX, simplified), engines built on-device with TensorRT 8.5.2:

| Precision | GPU compute (median) | Latency (mean) | Throughput | Engine size | Build time |
|---|---|---|---|---|---|
| FP16 | **7.52 ms** | 8.36 ms | 130.5 qps | 8.1 MB | 794 s |
| FP32 | 13.39 ms | 14.46 ms | 72.9 qps | 14 MB | 259 s |

FP16 is 1.79x faster at ~half the engine size. H2D 0.41 ms + D2H 0.31 ms — memcpy is
negligible, so GPU preprocessing is unlikely to pay for itself here (measure in the node
before deciding). Mean inference of 8.36 ms fits a 60 fps frame budget (16.7 ms) with
~8 ms remaining for preprocess + decode + NMS.

Engines are architecture- and TRT-version-specific: built on-device, never committed.
The portable artifact is the ONNX (exported on the Mac: ultralytics, opset 12,
`simplify=True`; input `images` 1x3x640x640, output `output0` 1x84x8400).

---

## Notes

- 2026-08-05: Project restarted after a ~6 month pause, re-scoped around ROS 2 and edge
  deployment on the Jetson.
- 2026-08-05: Week 1 infrastructure is done in one session — SSH, internet, Docker, a
  Humble container with working camera and GPU passthrough, and a real Argus capture.
- 2026-08-05: **Week 1 complete.** `jetson-ros2-perception` workspace builds and the
  `jetson_camera` node publishes `/image_raw` + `/camera_info` from Argus. Docker
  `default-runtime` set to `nvidia`; image builds reproducibly from `docker/Dockerfile`.
- **Next actions (week 1 tail, then week 2):**
  1. **Lock manual focus** against a detailed target at working distance — must happen
     *before* calibration, or the intrinsics describe a configuration that no longer exists.
  2. Calibrate intrinsics with `camera_calibration`; write `imx477.yaml` and point
     `camera_info_url` at it (the node currently warns that no calibration file exists).
  3. **Establish a valid publisher-side frame rate.** The 30 Hz baseline was measured with
     `ros2 topic hz`, whose Python subscriber deserializes 6.2 MB per frame and may itself
     be the bottleneck. Instrument the node with its own counter before drawing any
     conclusion about `videoconvert` or DDS.
  4. ~~`git init` and first commit~~ — **done 2026-08-05**, pushed to
     `github.com/wai-ming-chan/jetson-ros2-perception` (public, Apache-2.0).
     The remote was pre-seeded with a LICENSE commit, so the first local commit was
     rebased onto it rather than force-pushed.
