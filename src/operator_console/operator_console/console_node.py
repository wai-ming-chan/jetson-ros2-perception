#!/usr/bin/env python3
"""Operator console for the Jetson perception stack.

A single ROS 2 node that also serves a web UI: live camera, pipeline telemetry, CAN
traffic, and the controls needed to actually operate the thing (record a bag, snapshot a
frame, inject a CAN frame).

Deliberately NOT a Foxglove replacement. Foxglove and rqt visualise topics; this exposes a
*control surface* -- start/stop recording, send frames, capture stills -- which is the part
they do not do. Use both.

Dependencies: rclpy, sensor_msgs, cv2, numpy. Everything else is the Python standard
library. That is intentional: the container image has no flask/fastapi/tornado and no ROS
web packages, and on Ubuntu 20.04 there are no Humble binaries to install them from, so
each extra dependency would mean a source build.
"""

import collections
import json
import mimetypes
import os
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Float32

from operator_console.canutil import (  # noqa: E402  (pure helpers, unit-tested)
    CAN_FRAME_SIZE,
    build_can_frame,
    parse_can_frame,
)


def deque_hz(dq, lock):
    """Rate implied by a deque of monotonic timestamps."""
    with lock:
        t = list(dq)
    if len(t) < 2:
        return 0.0
    span = t[-1] - t[0]
    return (len(t) - 1) / span if span > 0 else 0.0


class State:
    """Everything the web handlers read. Guarded by a single lock -- the contents are
    small and updates are cheap, so finer-grained locking would buy nothing."""

    def __init__(self, can_history):
        self.lock = threading.Lock()
        self.frame_jpeg = None         # latest frame, already JPEG-encoded (bytes)
        self.frame_seq = 0
        self.width = 0
        self.height = 0
        self.encoding = ""
        self.frame_times = collections.deque(maxlen=90)
        self.publisher_hz = None       # authoritative, reported BY the camera node
        self.calibrated = False
        self.calib_name = ""
        self.can_frames = collections.deque(maxlen=can_history)
        self.can_status = "not started"
        self.can_count = 0
        self.recording = None          # subprocess.Popen
        self.record_path = ""
        self.record_started = 0.0

    def image_hz(self):
        return deque_hz(self.frame_times, self.lock)



# --------------------------------------------------------------------------------------
# Host telemetry
#
# Read straight from /proc and /sys rather than parsing tegrastats: tegrastats is a
# long-running process, and these files are already bind-visible inside the container.
# Every reader degrades to None rather than raising -- a missing sysfs path on some other
# board must not take the console down.
# --------------------------------------------------------------------------------------

_prev_cpu = {}


def cpu_percent():
    """Aggregate CPU busy% since the previous call."""
    global _prev_cpu
    try:
        with open("/proc/stat") as fh:
            parts = fh.readline().split()
    except OSError:
        return None
    vals = [int(v) for v in parts[1:8]]
    idle = vals[3] + vals[4]
    total = sum(vals)
    prev_idle, prev_total = _prev_cpu.get("v", (idle, total))
    _prev_cpu["v"] = (idle, total)
    dt, di = total - prev_total, idle - prev_idle
    if dt <= 0:
        return None
    return round(100.0 * (dt - di) / dt, 1)


def gpu_percent():
    for path in ("/sys/devices/gpu.0/load",
                 "/sys/devices/platform/gpu.0/load",
                 "/sys/devices/platform/17000000.gpu/load"):
        try:
            with open(path) as fh:
                return round(int(fh.read().strip()) / 10.0, 1)   # per-mille -> percent
        except (OSError, ValueError):
            continue
    return None


def power_mode():
    """nvpmodel writes e.g. 'pmode:0000'. On this board 0 = 15W, 1 = 7W."""
    try:
        with open("/var/lib/nvpmodel/status") as fh:
            raw = fh.read().strip()
    except OSError:
        return None
    mode = raw.split(":")[-1].lstrip("0") or "0"
    return {"0": "15W", "1": "7W"}.get(mode, "mode " + mode)


def soc_temp_c():
    try:
        for zone in sorted(os.listdir("/sys/devices/virtual/thermal")):
            if not zone.startswith("thermal_zone"):
                continue
            base = "/sys/devices/virtual/thermal/" + zone
            with open(base + "/type") as fh:
                if fh.read().strip() not in ("CPU-therm", "cpu-thermal"):
                    continue
            with open(base + "/temp") as fh:
                return round(int(fh.read().strip()) / 1000.0, 1)
    except OSError:
        pass
    return None


def disk_free_gb(path="/captures"):
    try:
        st = os.statvfs(path)
        return round(st.f_bavail * st.f_frsize / 1e9, 1)
    except OSError:
        return None


# --------------------------------------------------------------------------------------
# CAN
# --------------------------------------------------------------------------------------

def can_reader(iface, state, stop_event):
    """Read raw SocketCAN frames into state, reconnecting as needed.

    Reads the socket directly rather than going through a ROS CAN message: can_msgs is not
    in this image, and adding a message package would pull in rosidl generation for a
    handful of fields. The dedicated bridge node can publish proper messages later; the
    console only needs to observe.

    Retries rather than giving up: the console is routinely started before `can0` exists
    (bringing the interface up needs root, and often happens after the stack is running),
    and an operator panel that permanently reports "no such device" because of startup
    ordering is useless.
    """
    while not stop_event.is_set():
        sock = None
        try:
            sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sock.settimeout(0.5)
            sock.bind((iface,))
        except OSError as exc:
            if sock is not None:
                sock.close()
            with state.lock:
                state.can_status = "waiting for {} ({})".format(
                    iface, exc.strerror or exc)
            stop_event.wait(3.0)
            continue

        with state.lock:
            state.can_status = "listening on " + iface

        while not stop_event.is_set():
            try:
                raw = sock.recv(CAN_FRAME_SIZE)
            except socket.timeout:
                continue
            except OSError as exc:
                with state.lock:
                    state.can_status = "{} went away ({}); retrying".format(iface, exc)
                break
            if len(raw) < CAN_FRAME_SIZE:
                continue

            entry = parse_can_frame(raw)
            if entry is None:
                continue
            entry["t"] = round(time.monotonic(), 3)
            with state.lock:
                state.can_frames.appendleft(entry)
                state.can_count += 1

        sock.close()


def can_send(iface, can_id, payload, extended=False):
    """Transmit one frame. Returns None on success, else an error string."""
    try:
        frame = build_can_frame(can_id, payload, extended)
    except ValueError as exc:
        return str(exc)
    try:
        sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((iface,))
        sock.send(frame)
        sock.close()
    except OSError as exc:
        return str(exc)
    return None


# --------------------------------------------------------------------------------------
# ROS node
# --------------------------------------------------------------------------------------

class OperatorConsole(Node):

    def __init__(self):
        super().__init__("operator_console")

        self.image_topic = self.declare_parameter("image_topic", "/image_raw").value
        self.port = self.declare_parameter("port", 8080).value
        self.can_interface = self.declare_parameter("can_interface", "can0").value
        self.capture_dir = self.declare_parameter("capture_dir", "/captures").value
        # Record the COMPRESSED stream, not raw. Raw 1080p bgr8 is 6.2 MB/frame; at
        # 60 Hz that is ~370 MB/s and fills a 90 GB disk in about four minutes. The
        # compressed topic is ~250 KB/frame -- ~15 MB/s -- and is what any downstream
        # consumer wants anyway. Recording it also makes rosbag2 a subscriber, which is
        # what causes the camera to encode at all (the plugin is lazy).
        self.record_topics = self.declare_parameter(
            "record_topics",
            ["/image_raw/compressed", "/camera_info", "/trt_detector/detections"]).value
        # Refuse to start below this much free space. Without a floor, a forgotten
        # recording fills the disk and takes the whole stack down with it.
        self.min_free_gb = float(self.declare_parameter(
            "record_min_free_gb", 5.0,
            ParameterDescriptor(dynamic_typing=True)).value)
        self.rate_topic = self.declare_parameter(
            "rate_topic", "/argus_camera/publish_rate").value

        self.state = State(can_history=self.declare_parameter("can_history", 200).value)

        # Consume the CAMERA's JPEG stream (/image_raw/compressed, encoded in C++ by
        # image_transport), not the raw topic. Raw frames are 6.2 MB and rclpy tops out
        # around 13-50 Hz just deserialising them -- that ceiling was the whole reason the
        # live view lagged. Compressed frames are ~200 KB, deserialise trivially, and are
        # already exactly what the browser needs, so this node never touches a pixel.
        #
        # The subscription is ON-DEMAND, managed by a timer on the executor thread
        # (creating/destroying subscriptions from HTTP threads would race the executor).
        # image_transport's compressed plugin is lazy -- the camera only pays for JPEG
        # encoding while a subscriber exists -- and that encode runs synchronously in the
        # capture loop, costing measured throughput: 60 Hz -> ~40 Hz at 1080p while
        # subscribed. Subscribing only while a browser is actually pulling frames hands
        # the pipeline its full rate back the moment the last viewer goes away.
        self._dims_checked = 0.0
        self._image_sub = None
        self._last_frame_request = 0.0
        self.create_timer(1.0, self._manage_image_sub)
        self.create_subscription(
            CameraInfo, self._info_topic(), self._on_info, qos_profile_sensor_data)
        # The publisher's own measurement. Without this the console could only report the
        # rate at which IT receives frames, which for 6 MB messages in Python is far below
        # what the pipeline actually produces -- 9 Hz observed against an actual 60 Hz.
        self.create_subscription(Float32, self.rate_topic, self._on_rate, 10)

        self._stop = threading.Event()
        threading.Thread(
            target=can_reader, args=(self.can_interface, self.state, self._stop),
            daemon=True).start()

        cpu_percent()          # prime the delta; the first call has no baseline
        os.makedirs(self.capture_dir, exist_ok=True)
        self._serve()

        self.get_logger().info(
            "operator console on http://0.0.0.0:{} (image={}, can={})".format(
                self.port, self.image_topic, self.can_interface))

    def _manage_image_sub(self):
        want = (time.monotonic() - self._last_frame_request) < 5.0
        if want and self._image_sub is None:
            self._image_sub = self.create_subscription(
                CompressedImage, self.image_topic + "/compressed",
                self._on_compressed, qos_profile_sensor_data)
            self.get_logger().info("viewer active: subscribed to compressed stream")
        elif not want and self._image_sub is not None:
            self.destroy_subscription(self._image_sub)
            self._image_sub = None
            self.get_logger().info(
                "no viewer for 5 s: unsubscribed; camera stops paying for JPEG encode")

    def _info_topic(self):
        base = self.image_topic.rsplit("/", 1)[0]
        return (base + "/camera_info") if base else "/camera_info"

    def _on_compressed(self, msg):
        with self.state.lock:
            self.state.frame_jpeg = bytes(msg.data)
            self.state.frame_seq += 1
            self.state.encoding = msg.format
            self.state.frame_times.append(time.monotonic())

    def _on_rate(self, msg):
        with self.state.lock:
            self.state.publisher_hz = float(msg.data)

    def _on_info(self, msg):
        # An all-zero K means camera_info_manager has no calibration loaded.
        with self.state.lock:
            self.state.calibrated = any(msg.k)
            self.state.calib_name = msg.distortion_model if any(msg.k) else ""

    def _serve(self):
        handler = make_handler(self)
        self.httpd = ThreadingHTTPServer(("0.0.0.0", int(self.port)), handler)
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def destroy_node(self):
        self._stop.set()
        self.stop_recording()
        try:
            self.httpd.shutdown()
        except Exception:
            pass
        super().destroy_node()

    # -- controls ----------------------------------------------------------------------

    def start_recording(self):
        with self.state.lock:
            if self.state.recording and self.state.recording.poll() is None:
                return "already recording"
        free = disk_free_gb(self.capture_dir)
        if free is not None and free < self.min_free_gb:
            return "only {:.1f} GB free (need {:.1f})".format(free, self.min_free_gb)

        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.capture_dir, "rosbag2_" + stamp)
        cmd = ["ros2", "bag", "record", "-o", path] + list(self.record_topics)
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as exc:
            return str(exc)
        with self.state.lock:
            self.state.recording = proc
            self.state.record_path = path
            self.state.record_started = time.monotonic()
        return None

    def stop_recording(self):
        with self.state.lock:
            proc = self.state.recording
            self.state.recording = None
        if proc and proc.poll() is None:
            proc.terminate()          # SIGTERM lets rosbag2 close the file cleanly
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
            return None
        return "not recording"

    def snapshot(self):
        jpeg, _ = self.get_frame()
        if jpeg is None:
            return None, "no frame available"
        path = os.path.join(
            self.capture_dir, "snapshot_{}.jpg".format(time.strftime("%Y%m%d_%H%M%S")))
        # The frame is already a JPEG from the camera's encoder; write the bytes as-is
        # rather than decoding and re-encoding (which would also lose quality).
        with open(path, "wb") as fh:
            fh.write(jpeg)
        return path, None

    def current_seq(self):
        """Sequence number of the newest frame; used by long-poll waits."""
        with self.state.lock:
            return self.state.frame_seq

    def get_frame(self):
        """Return (jpeg_bytes, frame_seq) for the newest frame. No encoding happens here
        at all -- the camera's image_transport plugin already produced the JPEG in C++."""
        with self.state.lock:
            return self.state.frame_jpeg, self.state.frame_seq

    def _dims(self):
        """(width, height) of the current frame, decoded at most every 5 s.

        CompressedImage carries no dimensions, and decoding every frame just to read its
        size would reintroduce the per-frame CPU cost this refactor removed. Resolution
        changes only when the camera restarts in a new mode, so a slow probe is fine.
        """
        now = time.monotonic()
        if now - self._dims_checked > 5.0:
            self._dims_checked = now
            jpeg, _ = self.get_frame()
            if jpeg is not None:
                img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    with self.state.lock:
                        self.state.height, self.state.width = img.shape[:2]
        with self.state.lock:
            return self.state.width, self.state.height

    def status(self):
        st = self.state
        with st.lock:
            rec = st.recording is not None and st.recording.poll() is None
            rec_elapsed = time.monotonic() - st.record_started if rec else 0.0
            payload = {
                "image": {
                    "topic": self.image_topic,
                    "encoding": st.encoding,
                    "connected": bool(st.frame_times) and
                                 (time.monotonic() - st.frame_times[-1]) < 2.0,
                    "publisher_hz": st.publisher_hz,
                },
                "calibration": {"loaded": st.calibrated, "model": st.calib_name},
                "can": {
                    "interface": self.can_interface,
                    "status": st.can_status,
                    "count": st.can_count,
                },
                "recording": {
                    "active": rec,
                    "path": st.record_path if rec else "",
                    "elapsed": round(rec_elapsed, 1),
                    "size_mb": dir_size_mb(st.record_path) if rec else 0,
                },
            }
        payload["image"]["console_rx_hz"] = round(st.image_hz(), 2)
        payload["image"]["width"], payload["image"]["height"] = self._dims()
        payload["host"] = {
            "cpu": cpu_percent(), "gpu": gpu_percent(),
            "power": power_mode(), "temp": soc_temp_c(),
            "disk_free_gb": disk_free_gb(self.capture_dir),
        }
        return payload


def dir_size_mb(path):
    total = 0
    try:
        for root, _, files in os.walk(path):
            for name in files:
                try:
                    total += os.path.getsize(os.path.join(root, name))
                except OSError:
                    pass
    except OSError:
        return 0
    return round(total / 1e6, 1)


# --------------------------------------------------------------------------------------
# HTTP
# --------------------------------------------------------------------------------------

def make_handler(node):

    web_root = os.path.join(get_package_share_directory("operator_console"), "web")

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass                      # keep the ROS log readable

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                return self._static("index.html")
            if path == "/api/status":
                return self._json(node.status())
            if path == "/api/can":
                with node.state.lock:
                    return self._json(list(node.state.can_frames)[:80])
            if path == "/frame.jpg":
                return self._frame()
            if path.startswith("/static/"):
                return self._static(os.path.basename(path))
            self.send_error(404)

        def do_POST(self):
            path = self.path.split("?", 1)[0]
            length = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(length) if length else b"{}"
            try:
                body = json.loads(raw or b"{}")
            except ValueError:
                return self._json({"error": "invalid JSON"}, 400)

            if path == "/api/record/start":
                err = node.start_recording()
                return self._json({"ok": err is None, "error": err})
            if path == "/api/record/stop":
                err = node.stop_recording()
                return self._json({"ok": err is None, "error": err})
            if path == "/api/snapshot":
                p, err = node.snapshot()
                return self._json({"ok": err is None, "path": p, "error": err})
            if path == "/api/can/send":
                try:
                    can_id = int(str(body.get("id", "123")), 16)
                    payload = bytes.fromhex(str(body.get("data", "")).replace(" ", ""))
                except ValueError as exc:
                    return self._json({"ok": False, "error": str(exc)}, 400)
                err = can_send(node.can_interface, can_id, payload,
                               bool(body.get("ext", False)))
                return self._json({"ok": err is None, "error": err})
            self.send_error(404)

        def _static(self, name):
            full = os.path.join(web_root, name)
            if not os.path.isfile(full):
                return self.send_error(404)
            with open(full, "rb") as fh:
                body = fh.read()
            ctype = mimetypes.guess_type(full)[0] or "application/octet-stream"
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _frame(self):
            """One JPEG, optionally long-polled.

            Marks the viewer active, which makes the management timer (re)subscribe. The
            first request after an idle period may therefore return a stale frame while
            the subscription spins up (~1-2 s); the long-poll timeout covers the gap.

            With ?since=<seq>, the request is held (up to ~1.5 s) until a frame newer
            than <seq> exists, then answered. The client's next request can therefore be
            issued immediately after the previous frame renders, and every response is a
            fresh frame -- without this, a self-paced client wastes most of its round
            trips fetching duplicates (measured: 733 requests, 206 distinct frames).
            The frame's sequence number is returned in X-Frame-Seq for the client to
            chain. Without ?since it answers immediately (paused view, first request).
            """
            node._last_frame_request = time.monotonic()
            since = -1
            if "?" in self.path:
                for kv in self.path.split("?", 1)[1].split("&"):
                    if kv.startswith("since="):
                        try:
                            since = int(kv[6:])
                        except ValueError:
                            pass
            if since >= 0:
                deadline = time.monotonic() + 1.5
                while node.current_seq() == since and time.monotonic() < deadline:
                    time.sleep(0.004)

            jpeg, seq = node.get_frame()
            if jpeg is None:
                return self.send_error(503, "no frame available")
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Frame-Seq", str(seq))
            self.end_headers()
            self.wfile.write(jpeg)

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = OperatorConsole()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
