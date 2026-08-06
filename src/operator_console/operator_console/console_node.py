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
import select
import mimetypes
import os
import socket
import struct
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
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32

# SocketCAN frame layout: can_id (u32), len (u8), 3 pad, data (8 bytes).
CAN_FRAME_FMT = "<IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF


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
        self.frame = None              # latest BGR frame (numpy)
        self.frame_seq = 0
        self.width = 0
        self.height = 0
        self.encoding = ""
        self.frame_times = collections.deque(maxlen=90)
        self.publisher_hz = None       # authoritative, reported BY the camera node
        self.calibrated = False
        self.calib_name = ""
        self.stream_times = collections.deque(maxlen=60)
        self.can_frames = collections.deque(maxlen=can_history)
        self.can_status = "not started"
        self.can_count = 0
        self.recording = None          # subprocess.Popen
        self.record_path = ""
        self.record_started = 0.0

    def image_hz(self):
        return deque_hz(self.frame_times, self.lock)

    def _unused_image_hz(self):
        with self.lock:
            t = list(self.frame_times)
        if len(t) < 2:
            return 0.0
        span = t[-1] - t[0]
        return (len(t) - 1) / span if span > 0 else 0.0


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

            can_id, length, data = struct.unpack(CAN_FRAME_FMT, raw)
            extended = bool(can_id & CAN_EFF_FLAG)
            entry = {
                "id": "{:08X}".format(can_id & CAN_EFF_MASK) if extended
                      else "{:03X}".format(can_id & CAN_SFF_MASK),
                "ext": extended,
                "rtr": bool(can_id & CAN_RTR_FLAG),
                "err": bool(can_id & CAN_ERR_FLAG),
                "dlc": length,
                "data": data[:length].hex().upper(),
                "t": round(time.monotonic(), 3),
            }
            with state.lock:
                state.can_frames.appendleft(entry)
                state.can_count += 1

        sock.close()


def can_send(iface, can_id, payload, extended=False):
    """Transmit one frame. Returns None on success, else an error string."""
    if len(payload) > 8:
        return "payload longer than 8 bytes"
    try:
        sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((iface,))
        ident = can_id | (CAN_EFF_FLAG if extended else 0)
        sock.send(struct.pack(CAN_FRAME_FMT, ident, len(payload), payload.ljust(8, b"\0")))
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
        self.stream_width = self.declare_parameter("stream_width", 960).value
        self.stream_quality = self.declare_parameter("stream_jpeg_quality", 75).value
        # A CAP, not a target. Measured on this board (1080p source, MJPEG to one
        # client), delivered rate and total host CPU:
        #     width 640  -> ~55-60 fps, 58% cpu   (hits the cap)
        #     width 960  -> ~36 fps,    48% cpu   (default: good detail/smoothness balance)
        # The publisher holds 60 Hz throughout, so streaming costs CPU but not throughput.
        # This was 15 fps hardcoded, which was the sole cause of visible lag.
        # dynamic_typing so both `stream_fps:=60` and `stream_fps:=60.0` work.
        # Declared strictly, an int override against a float default raises
        # InvalidParameterTypeException and the node dies at startup.
        self.stream_fps = float(self.declare_parameter(
            "stream_fps", 60.0,
            ParameterDescriptor(dynamic_typing=True)).value)
        self.capture_dir = self.declare_parameter("capture_dir", "/captures").value
        self.record_topics = self.declare_parameter(
            "record_topics", ["/image_raw", "/camera_info"]).value
        self.rate_topic = self.declare_parameter(
            "rate_topic", "/argus_camera/publish_rate").value

        self.max_streams = int(self.declare_parameter("max_streams", 8).value)
        # Seconds a client may go without accepting data before we hang up. A
        # browser that navigates away can leave a socket whose buffer stays full:
        # select() then reports "not writable" forever, and because the loop never
        # attempts a write it never sees BrokenPipeError either, so without this it
        # spins for the life of the process holding a stream slot.
        self.stream_stall_timeout = float(self.declare_parameter(
            "stream_stall_timeout", 10.0,
            ParameterDescriptor(dynamic_typing=True)).value)
        self.state = State(can_history=self.declare_parameter("can_history", 200).value)

        # Every page load opens another /stream.mjpg connection, and a browser does not
        # always close the old one promptly. Without a bound, each stale connection keeps
        # an encoder thread busy: seven connections and 24 threads were observed after a
        # handful of reloads. Retire the oldest beyond max_streams.
        self._streams = collections.OrderedDict()
        self._stream_lock = threading.Lock()
        self._stream_seq = 0

        # One encode per frame, shared by every client. Serialised so two clients cannot
        # encode the same frame concurrently.
        self._encode_lock = threading.Lock()
        self._enc_seq = -1
        self._enc_jpeg = None

        # Best-effort, depth 1. A reliable subscriber would queue 6 MB messages faster than
        # Python can drain them; for a viewer, the newest frame is the only one that
        # matters and older ones should be dropped, not buffered.
        self.create_subscription(
            Image, self.image_topic, self._on_image, qos_profile_sensor_data)
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

    def register_stream(self):
        with self._stream_lock:
            self._stream_seq += 1
            sid = self._stream_seq
            self._streams[sid] = threading.Event()
            while len(self._streams) > self.max_streams:
                old_sid, old_ev = next(iter(self._streams.items()))
                old_ev.set()
                del self._streams[old_sid]
            return sid, self._streams[sid]

    def unregister_stream(self, sid):
        with self._stream_lock:
            self._streams.pop(sid, None)

    def active_streams(self):
        with self._stream_lock:
            return len(self._streams)

    def _info_topic(self):
        base = self.image_topic.rsplit("/", 1)[0]
        return (base + "/camera_info") if base else "/camera_info"

    def _on_image(self, msg):
        if msg.encoding not in ("bgr8", "rgb8"):
            return
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            frame = frame[:, :, ::-1]
        with self.state.lock:
            self.state.frame = frame
            self.state.frame_seq += 1
            self.state.width, self.state.height = msg.width, msg.height
            self.state.encoding = msg.encoding
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
        with self.state.lock:
            frame = None if self.state.frame is None else self.state.frame.copy()
        if frame is None:
            return None, "no frame available"
        path = os.path.join(
            self.capture_dir, "snapshot_{}.jpg".format(time.strftime("%Y%m%d_%H%M%S")))
        cv2.imwrite(path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        return path, None

    def current_seq(self):
        """Cheap check for whether a new frame exists. Callers MUST use this before
        encode_stream_frame(): encoding costs ~7 ms, and a poll loop that encodes first and
        checks the sequence afterwards becomes a busy-encode loop that starves the executor
        feeding it frames."""
        with self.state.lock:
            return self.state.frame_seq

    def encode_stream_frame(self):
        """Return (jpeg_bytes, frame_seq) for the newest frame, encoding at most once per
        frame no matter how many clients are streaming.

        Resizes inside the state lock rather than copying the full-resolution frame and
        resizing after: the copy existed only to release the lock sooner, but resize
        already writes to a fresh buffer, so it was ~1.1 ms of pure memcpy per frame.
        """
        with self._encode_lock:
            with self.state.lock:
                seq = self.state.frame_seq
                if seq == self._enc_seq:
                    return self._enc_jpeg, seq          # another client already encoded it
                frame = self.state.frame
                if frame is None:
                    return None, seq
                if self.stream_width and frame.shape[1] > self.stream_width:
                    scale = self.stream_width / float(frame.shape[1])
                    frame = cv2.resize(frame, None, fx=scale, fy=scale,
                                       interpolation=cv2.INTER_AREA)
                else:
                    frame = frame.copy()

            ok, buf = cv2.imencode(".jpg", frame,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), int(self.stream_quality)])
            self._enc_jpeg = buf.tobytes() if ok else None
            self._enc_seq = seq
            return self._enc_jpeg, seq

    def status(self):
        st = self.state
        with st.lock:
            rec = st.recording is not None and st.recording.poll() is None
            rec_elapsed = time.monotonic() - st.record_started if rec else 0.0
            payload = {
                "image": {
                    "topic": self.image_topic,
                    "width": st.width, "height": st.height,
                    "encoding": st.encoding,
                    "connected": st.frame is not None,
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
        payload["image"]["stream_hz"] = round(deque_hz(st.stream_times, st.lock), 2)
        payload["image"]["stream_clients"] = self.active_streams()
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
            if path == "/stream.mjpg":
                return self._stream()
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
            """One JPEG. Used when the live stream is paused, so the panel shows a frozen
            last frame instead of collapsing to an empty box."""
            jpeg, _ = node.encode_stream_frame()
            if jpeg is None:
                return self.send_error(503, "no frame available")
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(jpeg)

        def _stream(self):
            """multipart/x-mixed-replace MJPEG, with frames DROPPED rather than queued.

            The naive loop writes every frame regardless of whether the client is keeping
            up. Anything the client has not drained sits in the kernel send buffer, which
            defaults to as much as 4 MB here -- over a hundred JPEGs. MJPEG is displayed in
            order, so the viewer falls steadily further behind: observed as several seconds
            of lag, with 230 KB already queued on one socket.

            Two changes bound it. The send buffer is capped so the kernel cannot hoard
            frames, and the socket is polled for writability before each send; when the
            client is behind, the frame is discarded instead of buffered. For a live view,
            a dropped frame is always better than a stale one.
            """
            sid, cancelled = node.register_stream()
            sock = self.connection
            peer = "%s:%s" % self.client_address
            node.get_logger().info(
                "stream %d open from %s (active=%d)" % (sid, peer, node.active_streams()))
            reason = "closed"
            sent = 0
            try:
                # Small enough that only a frame or two can be in flight. The kernel
                # doubles the requested value.
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 128 * 1024)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                pass

            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()

            last_seq = -1
            min_interval = 1.0 / max(1.0, float(node.stream_fps))
            next_send = 0.0
            stalled_since = None
            try:
                while not cancelled.is_set():
                    now = time.monotonic()
                    if now < next_send:
                        time.sleep(min(0.002, next_send - now))
                        continue

                    # Writable means the send buffer has drained. If it has not, the client
                    # is behind: skip this frame rather than adding to the backlog. If it
                    # stays unwritable past the timeout the client is gone, not merely
                    # slow, so hang up and free the slot.
                    if not select.select([], [sock], [], 0)[1]:
                        if stalled_since is None:
                            stalled_since = now
                        elif now - stalled_since > node.stream_stall_timeout:
                            reason = "stalled %.0fs (client not draining)" % (
                                now - stalled_since)
                            break
                        time.sleep(0.002)
                        continue
                    stalled_since = None

                    # Check the sequence BEFORE encoding. Encoding first and
                    # discarding afterwards costs a full resize+JPEG on every poll.
                    if node.current_seq() == last_seq:
                        time.sleep(0.002)
                        continue

                    jpeg, seq = node.encode_stream_frame()
                    if jpeg is None or seq == last_seq:
                        time.sleep(0.002)
                        continue

                    last_seq = seq
                    next_send = now + min_interval
                    with node.state.lock:
                        node.state.stream_times.append(now)
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpeg)))
                    self.end_headers()
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                    sent += 1
            except (BrokenPipeError, ConnectionResetError, OSError) as exc:
                reason = "disconnected (%s)" % type(exc).__name__
            finally:
                if cancelled.is_set():
                    reason = "retired: max_streams exceeded"
                node.unregister_stream(sid)
                node.get_logger().info(
                    "stream %d closed after %d frames from %s -- %s"
                    % (sid, sent, peer, reason))

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
