# Operations runbook

Deployment and field-debugging guide for the Jetson perception stack.

Every entry below comes from a failure that actually occurred on this hardware, with the
command that diagnosed it. Symptoms are listed the way you encounter them — what you see —
not the way the fix is organised.

**Target:** Seeed reComputer Industrial J3011 (Orin Nano 8GB), JetPack 5.1.3 / L4T R35.5.0.

---

## 1. Quick reference

```bash
# start everything (camera + detector + console)
cd ~/jetson-ros2-perception && ./docker/run.sh
ros2 launch jetson_bringup bringup.launch.py

# stop           NEVER `docker rm -f` -- see rule 1 below
docker stop -t 30 <container>

# console        http://<jetson-ip>:8080
```

Variants: `bringup.launch.py detector:=false` (camera only), `console:=false` (headless).

### Three rules that prevent most incidents

1. **Never `docker rm -f` or `kill -9` a container holding the camera.** A SIGKILLed Argus
   client leaves a session dangling inside `nvargus-daemon`. Damage accumulates until the
   camera stops starting at all. Use `docker stop -t 30`.
2. **After any unclean camera exit, restart the daemon before relaunching:**
   `sudo systemctl restart nvargus-daemon`.
3. **Never run two stacks at once.** Check `docker ps` first. Two detectors will saturate
   the GPU and halve the frame rate of both.

---

## 2. First-time setup on a fresh device

Assumes JetPack 5.1.3 flashed and the IMX477 attached to CAM0.

```bash
# 1. Docker access (log out and back in afterwards -- group membership is fixed at login)
sudo usermod -aG docker $USER

# 2. NVIDIA runtime as default -- required for CUDA to be visible during `docker build`
sudo tee /etc/docker/daemon.json >/dev/null <<'EOF'
{ "default-runtime": "nvidia",
  "runtimes": { "nvidia": { "path": "nvidia-container-runtime", "runtimeArgs": [] } } }
EOF
sudo systemctl restart docker

# 3. Host settings the stack depends on (DDS buffers, EEE-off, CAN)
cd ~/jetson-ros2-perception && sudo deploy/install.sh

# 4. Build the image and the workspace
docker build -t jetson-ros2-perception:latest -f docker/Dockerfile .
./docker/run.sh
colcon build --symlink-install

# 5. Detection engine (device-specific; never committed, always built locally)
mkdir -p ~/models   # place yolov8n.onnx here first
/usr/src/tensorrt/bin/trtexec --onnx=/models/yolov8n.onnx --fp16 \
    --saveEngine=/models/yolov8n_fp16.engine        # ~13 min on Orin Nano
```

### Verify the host settings

```bash
sysctl net.core.rmem_max        # 67108864   -- DDS needs this for 6.2 MB frames
ethtool --show-eee eth0         # disabled   -- EEE adds ~1 s wake latency
ip -details link show can0      # UP, bitrate 500000, <LOOPBACK>
```

`deploy/install.sh` is idempotent and safe to re-run after editing any unit.

---

## 3. Symptom index

| Symptom | Most likely cause | Section |
|---|---|---|
| Camera won't start, `Failed to create **CaptureSession**` | stale Argus session from an unclean exit | [4.1](#41-camera-will-not-start) |
| Camera won't start, `Failed to create **CameraProvider**` | daemon dead or restarting | [4.1](#41-camera-will-not-start) |
| Camera worked, then stopped; repeated rebuild logs | daemon segfault, or CSI hardware wedged | [4.2](#42-camera-stops-mid-run) |
| Ctrl-C does nothing, node won't die | teardown blocked on a wedged daemon | [4.3](#43-node-will-not-shut-down) |
| Captured image is black | lens cap, or AE not converged | [4.4](#44-black-or-noisy-images) |
| Node subscribes but receives **nothing** | kernel socket buffers too small for the message | [4.5](#45-a-subscriber-receives-nothing) |
| Web console unreachable | container exited, or wrong IP | [4.6](#46-console-problems) |
| Console video laggy or frozen | duplicate stack, or viewing on the Jetson's own desktop | [4.6](#46-console-problems) |
| Streaming fine over Wi-Fi, 1 Hz over Ethernet | Energy-Efficient Ethernet | [4.7](#47-ethernet-slower-than-wi-fi) |
| CAN Send does nothing, traffic table empty | `loopback` not set on a single-node bus | [4.8](#48-can-transmits-nothing) |
| Frame rate lower than expected | see the measurement warning | [5](#5-measuring-performance-correctly) |
| `sudo deploy/install.sh` reports success but nothing changed | unit was already active | [4.9](#49-installer-reports-success-but-changes-nothing) |

---

## 4. Procedures

### 4.1 Camera will not start

Read the *specific* Argus error — the two failure modes need different fixes.

```bash
docker logs <container> 2>&1 | grep -E "CaptureSession|CameraProvider"
systemctl status nvargus-daemon
ps -o pcpu,etime -C nvargus-daemon      # >20% CPU while idle means degraded
```

| Error | Meaning | Fix |
|---|---|---|
| `Failed to create CaptureSession` | daemon alive; the camera is held, or a stale session blocks it | check `docker ps` for another stack; then `sudo systemctl restart nvargus-daemon` |
| `Failed to create CameraProvider` | cannot reach the daemon at all — dead or restarting | wait ~15 s (systemd auto-restarts it), then relaunch |

If a restart does not fix it, go to 4.2.

### 4.2 Camera stops mid-run

The node retries for about a minute (10 rebuilds with 2→10 s backoff), which covers a
normal daemon restart. If it exits `FATAL` anyway, the daemon or the capture hardware is
in a state software cannot clear.

```bash
journalctl -u nvargus-daemon --no-pager -n 30
```

**Daemon segfault** — recoverable:
```
nvargus-daemon.service: Main process exited, code=killed, status=11/SEGV
nvargus-daemon.service: Scheduled restart job
```
systemd restarts it automatically; relaunch the stack. Triggered by a CSI capture timeout,
which appears just above the crash as `(fusa) Error: Timeout ... fusaViHandler.cpp`.

**CSI hardware wedged** — needs a reboot:
```
SCF: Error InvalidState: Timeout!! Skipping requests on sensor GUID 0
  (in FusaCaptureViCsiHw.cpp, function waitCsiFrameEnd())
SCF: Error BadParameter: CC has already been disposed
SCF: Error InvalidState: Sensor 0 already in same state
```
These persist *through* a daemon restart, and capture cycles (a few frames, timeout,
rebuild, repeat). A daemon restart clears software state; it cannot reset the VI/CSI
hardware.

```bash
docker stop -t 30 <container>    # stop hammering the CSI first
sudo reboot
cd ~/jetson-ros2-perception && sudo deploy/install.sh   # only if not yet installed
```

Prevention: rules 1–3 in section 1. Every unclean client exit makes this more likely.

### 4.3 Node will not shut down

`gst_element_set_state(NULL)` on `nvarguscamerasrc` is an RPC to `nvargus-daemon`; when the
daemon is wedged it never returns, and Ctrl-C appears to do nothing.

The node bounds this to 5 s, then logs `shutdown wedged` and force-exits. If you see that
message, the daemon needs restarting **before** the next launch — the force-exit could not
release its Argus session.

```bash
sudo systemctl restart nvargus-daemon
```

If a node predates this fix and truly hangs: `docker stop -t 30` (SIGTERM reaches the
launch, which is PID 1 in these containers). Accept that the daemon will need a restart.

### 4.4 Black or noisy images

In order of likelihood:

1. **Lens cap.** Genuinely the most common cause.
2. **Auto-exposure has not converged.** The first frames after startup are black or
   full-scale noise. Never sanity-check with `num-buffers=1`:
   ```bash
   gst-launch-1.0 -q nvarguscamerasrc num-buffers=60 \
     ! 'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=60/1' \
     ! nvjpegenc ! multifilesink location=/captures/f_%03d.jpg
   # inspect the LAST frame, not the first
   ```
3. **Aperture closed** on the lens iris ring, or a protective film.

**Do not judge image content by file size.** A maximum-gain noise frame compressed to
1.71 MB while a correctly exposed frame of the same scene was 152 KB — noise has high
entropy. Look at the pixels.

### 4.5 A subscriber receives nothing

Specifically: best-effort subscribers get **zero** messages while reliable ones limp at a
few Hz. This affects raw image topics (6.2 MB per 1080p frame), not compressed ones.

```bash
sysctl net.core.rmem_max     # 212992 (208 KB) is the broken default
```

The default holds 1/30th of one frame. DDS fragments each message into thousands of
datagrams; the buffer overflows mid-message and best-effort discards the whole frame on any
lost fragment. **Both halves of the fix are required** — the kernel must permit large
buffers *and* CycloneDDS must request them:

```bash
sudo deploy/install.sh      # installs /etc/sysctl.d/60-ros2-dds.conf
# and run containers with: -e CYCLONEDDS_URI=file:///workspace/docker/cyclonedds.xml
```

`docker/run.sh` sets `CYCLONEDDS_URI` automatically.

### 4.6 Console problems

```bash
ss -tln | grep :8080                    # is anything listening?
docker ps -a --format "{{.Names}} {{.Status}}"   # Exited (137) = it was SIGKILLed
docker logs <container> | tail -20
```

| Symptom | Cause | Fix |
|---|---|---|
| Page will not load | container exited | check `docker ps -a`; restart the stack |
| Page loads, video blank | camera not publishing | section 4.1 |
| Video laggy / low frame rate | two stacks running | `docker ps`; stop the duplicate |
| Video laggy, only on the Jetson's own screen | Firefox on the desktop costs ~22% CPU | view from another machine |
| Very slow over Ethernet only | EEE | section 4.7 |

### 4.7 Ethernet slower than Wi-Fi

Counter-intuitive but real: a gigabit direct link delivering 1 Hz while Wi-Fi manages 15.

```bash
ethtool --show-eee eth0     # "EEE status: enabled - active" is the problem
```

Energy-Efficient Ethernet (802.3az) idles the PHY between packets; on this Mac↔Tegra
pairing, waking costs **~1 second**. Any traffic with idle gaps — an interactive stream,
DDS discovery, request/reply — pays that on every gap. Bulk transfers never idle, which is
why a throughput test looks perfect and exonerates the link.

Diagnose by timing single fetches from idle, not bulk:
```
1.007s  1.007s  0.006s  0.006s  0.006s     <- stalls from idle, then 40 MB/s
```

Fix: `sudo deploy/install.sh` (installs `jetson-eee-off.service`). Verify with
`ethtool --show-eee`, **not** the command's own output — `ethtool` prints
`eee unmodified, ignoring` even when it applies the change.

### 4.8 CAN transmits nothing

```bash
ip -details link show can0
```

Look for two things:

- **`<LOOPBACK>` present?** On a single-node bus it must be. A CAN frame requires
  acknowledgement from another node; with nobody else on the wire the controller retries,
  fails, and drops to `ERROR-PASSIVE` with `TX: 0 packets`. `cansend` still "succeeds"
  because it only queues the frame.
- **`ERROR-PASSIVE`?** If loopback is enabled and transfers work, this is a stale
  high-water counter from earlier failures, not a live fault. Confirm by transmitting:

```bash
candump -T 4000 can0 & sleep 1; cansend can0 123#DEADBEEF; sleep 3
# expect the frame echoed back, and TX errors 0
```

Each frame appears **twice** under `loopback on` — SocketCAN's software loopback plus the
controller's hardware loopback. Expected; not deduplicated, because genuinely repeated
frames are normal CAN traffic.

**For a real multi-node bus:** remove `loopback on` from
`deploy/systemd/jetson-can0.service`. Leaving it on means this node neither hears others
nor reaches them.

### 4.9 Installer reports success but changes nothing

Two silent failures, both fixed in the current `deploy/`, worth knowing if you write
similar units:

- `ip link set <iface> type can ...` is **rejected while the interface is UP** — the
  command fails, systemd proceeds, and the old configuration survives. Bring the link down
  first (`ExecStartPre=-/sbin/ip link set down can0`).
- `systemctl enable --now` does **not** re-run a oneshot that is already active under
  `RemainAfterExit=yes`. Editing a unit and re-running the installer changes nothing. Use
  `systemctl restart`.

Always verify the end state (`ip -details link show can0`), never the installer's output.

---

## 5. Measuring performance correctly

Most "performance problems" found on this project were measurement errors. Three rules:

**Do not trust `ros2 topic hz` on image topics.** It is a Python subscriber that
deserialises every message; at 6.2 MB per frame it measures *its own* throughput. Measured
47.8 Hz against an actual publisher rate of 60.0 Hz — and 30 Hz when run inside the
publisher's own container, where it also competed for CPU.

**Instrument the publisher.** `jetson_camera` publishes its measured rate on
`~/publish_rate`, `trt_detector` publishes rate and per-stage latency on `~/publish_rate`
and `~/latency_ms`. Those are the authoritative numbers, and the console displays them.

**Check that something is actually saturated before optimising.** An idle CPU and idle GPU
alongside a "slow" pipeline means the measurement is wrong, not the code. Conversely, when
4K30 ran 11% short, one core sat at 74% while five idled — a single-threaded stage, fixed
with `videoconvert n-threads=0`, not a rewrite.

Useful commands:

```bash
tegrastats --interval 1000          # per-core CPU, GR3D_FREQ (GPU), power, temps
docker stats --no-stream            # per-container CPU and memory
ss -tn state established '( sport = :8080 )'   # Send-Q > 0 = client not draining
```

---

## 6. Platform notes

Facts about this hardware that are not obvious and will cost time:

- **`/proc/device-tree/model` lies.** It reports "NVIDIA Orin Nano Developer Kit" on the
  Industrial carrier. Identify the board with
  `grep TNSPEC /etc/nv_boot_control.conf` — the correct BSP depends on it.
- **Two sensor modes only:** 3840×2160@30 and 1920×1080@60, both `RG10` raw Bayer. There
  is no 12 MP mode; the carrier exposes 2-lane CSI.
- **Power modes are 7W and 15W.** The 25W/MAXN SUPER mode is JetPack 6.2-exclusive.
- **No autofocus** through V4L2 with the stock driver. Manual focus only.
- **`nvargus-daemon` is unmanaged shared state.** Every client crash degrades it
  cumulatively; nothing inside a container can repair it. A production deployment should
  pair the camera unit with an `OnFailure=` hook that bounces the daemon.
- **The camera works with NVIDIA's stock IMX477 driver** — no Arducam package, no
  `jetson-io.py`. Installing Arducam's kernel package is a documented way to break it on
  Seeed carriers.
- **ROS 2 Humble has no Ubuntu 20.04 binaries.** `apt install ros-humble-*` resolves
  nothing; the container image builds Humble from source. Every missing dependency is a
  source build, so check availability before designing around a package.
- **TensorRT engines are device- and version-specific.** Build on the target, never commit
  them. The portable artifact is the ONNX.

---

## 7. Calibration

Camera intrinsics, when needed (AprilTag pose, rectification, metric 3D).

```bash
# on the HOST, before starting the container -- run.sh reads these to decide what to mount
export DISPLAY=:1 XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
./docker/run.sh
# inside, with the camera running in another shell:
ros2 run camera_calibration cameracalibrator \
    --size 8x6 --square 0.020 --ros-args -r image:=/image_raw
```

Setting `DISPLAY` *inside* the container has no effect. Mounting the X cookie is
sufficient; `xhost` is not required and needlessly disables access control.

**Gotchas that cost a redo:**

- `--size` is **interior corners**, not squares (a 9×7-square board is `8x6`).
- **Measure the printed square.** Printers rescale; that number sets the scale of every
  distance the camera ever estimates.
- `cameracalibrator` writes `camera_name: narrow_stereo`. `camera_info_manager` compares it
  against the node's name (`imx477`) and rejects a mismatch — presenting as "no
  calibration" rather than an error. Edit it.
- **Intrinsics are resolution-specific.** 1080p values are wrong at 4K (roughly double).
  Hence the `_1080p` suffix on the file.
- **SAVE writes inside the container**, which `--rm` destroys on exit. Recover with
  `docker cp <container>:/tmp/calibrationdata.tar.gz .` *before* leaving.
- **COMMIT fails** with `Not available`: the calibrator calls `/camera/set_camera_info`
  while the node advertises `/set_camera_info`. Use SAVE instead.
- Check the **RMS reprojection error**, not just the matrix. Under ~0.5 px is good; above
  ~1.0 px, redo. A calibration with a plausible-looking matrix and poor RMS silently
  corrupts everything downstream.

---

## 8. Full incident history

`PROGRESS.md` records every issue with the commands that produced the diagnosis, in the
order they were encountered — including the wrong turns. This runbook is the distilled,
actionable form; that file is the evidence.
