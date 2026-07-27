import collections
import heapq
import json
import queue
import selectors
import shutil
import socket
import subprocess
import threading
import time

from agv_remote_protocol.network_impairment import NetworkImpairmentQueue
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


CAMERA_NAMES = (
    "front",
    "rear",
    "left_front",
    "left_rear",
    "right_front",
    "right_rear",
)


def encoder_arguments(encoder, bitrate_kbps, keyframe_interval):
    common = [
        "-b:v",
        f"{int(bitrate_kbps)}k",
        "-maxrate",
        f"{int(bitrate_kbps)}k",
        "-bufsize",
        f"{max(250, int(bitrate_kbps // 2))}k",
        "-g",
        str(int(keyframe_interval)),
        "-bf",
        "0",
    ]
    if encoder == "h264_nvenc":
        return [
            "-c:v",
            "h264_nvenc",
            "-preset",
            "p1",
            "-tune",
            "ull",
            "-rc",
            "cbr",
            "-zerolatency",
            "1",
            "-delay",
            "0",
            "-rc-lookahead",
            "0",
            "-surfaces",
            "1",
            "-forced-idr",
            "1",
            "-aud",
            "1",
            *common,
        ]
    return [
        "-c:v",
        "libx264",
        "-preset",
        "ultrafast",
        "-tune",
        "zerolatency",
        "-x264-params",
        (
            f"scenecut=0:keyint={int(keyframe_interval)}:"
            f"min-keyint={int(keyframe_interval)}:repeat-headers=1"
        ),
        *common,
    ]


def encoder_works(ffmpeg_path, encoder):
    command = [
        ffmpeg_path,
        "-hide_banner",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        "320x180",
        "-r",
        "1",
        "-i",
        "pipe:0",
        "-frames:v",
        "1",
        *encoder_arguments(encoder, 250, 1),
        "-f",
        "null",
        "-",
    ]
    try:
        result = subprocess.run(
            command,
            input=bytes(320 * 180 * 3),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            timeout=5.0,
            check=False,
        )
        return result.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def udp_port_is_bound(port):
    expected = f"{int(port):04X}"
    for path in ("/proc/net/udp", "/proc/net/udp6"):
        try:
            with open(path, "r", encoding="ascii") as stream:
                for line in stream:
                    fields = line.split()
                    if len(fields) > 1 and fields[1].split(":")[-1] == expected:
                        return True
        except OSError:
            continue
    return False


def wait_for_udp_listener(port, timeout_s=0.5):
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    while time.monotonic() < deadline:
        if udp_port_is_bound(port):
            return True
        time.sleep(0.005)
    return udp_port_is_bound(port)


class EncodedPacketProxy:
    """Delay, drop, and bandwidth-limit one or more encoded UDP streams."""

    def __init__(
        self,
        host,
        input_base_port,
        output_base_port,
        delay_ms,
        jitter_ms,
        loss_percent,
        seed,
        capacity_mbps,
        max_queue_delay_ms,
        camera_names=CAMERA_NAMES,
    ):
        self.camera_names = tuple(camera_names)
        self.host = host
        self.input_base_port = int(input_base_port)
        self.output_base_port = int(output_base_port)
        self.capacity_bytes_per_second = max(0.0, float(capacity_mbps)) * 1.0e6 / 8.0
        self.max_queue_delay_s = max(0.05, float(max_queue_delay_ms) * 0.001)
        self.selector = selectors.DefaultSelector()
        self.sockets = {}
        self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.queues = {}
        self.ready = []
        self.ready_sequence = 0
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.metrics = {
            name: {
                "input_packets": 0,
                "input_bytes": 0,
                "output_packets": 0,
                "output_bytes": 0,
                "network_drops": 0,
                "congestion_drops": 0,
                "last_packet_delay_ms": None,
                "last_output_at": None,
            }
            for name in self.camera_names
        }

        for index, name in enumerate(self.camera_names):
            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            udp_socket.bind((self.host, self.input_base_port + index))
            udp_socket.setblocking(False)
            self.sockets[name] = udp_socket
            self.selector.register(udp_socket, selectors.EVENT_READ, name)
            self.queues[name] = NetworkImpairmentQueue(
                delay_ms=delay_ms,
                jitter_ms=jitter_ms,
                loss_percent=loss_percent,
                seed=int(seed) + index,
            )

        burst_seconds = 0.5
        self.max_tokens = (
            max(65536.0, self.capacity_bytes_per_second * burst_seconds)
            if self.capacity_bytes_per_second > 0.0
            else float("inf")
        )
        self.tokens = self.max_tokens
        self.last_token_time = time.monotonic()

    def input_port(self, camera_name):
        return self.input_base_port + self.camera_names.index(camera_name)

    def output_port(self, camera_name):
        return self.output_base_port + self.camera_names.index(camera_name)

    def start(self):
        self.thread = threading.Thread(
            target=self.run, name="h264_udp_network_proxy", daemon=True
        )
        self.thread.start()

    def receive_packets(self):
        for key, _mask in self.selector.select(timeout=0.002):
            camera_name = key.data
            udp_socket = key.fileobj
            for _ in range(32):
                try:
                    payload, _address = udp_socket.recvfrom(65535)
                except BlockingIOError:
                    break
                received_at = time.monotonic()
                with self.lock:
                    metric = self.metrics[camera_name]
                    metric["input_packets"] += 1
                    metric["input_bytes"] += len(payload)
                if not self.queues[camera_name].enqueue((payload, received_at)):
                    with self.lock:
                        self.metrics[camera_name]["network_drops"] += 1

    def collect_ready(self, now):
        for camera_name in self.camera_names:
            for _release_at, (payload, received_at) in self.queues[camera_name].pop_ready(now):
                heapq.heappush(
                    self.ready,
                    (received_at, self.ready_sequence, camera_name, payload),
                )
                self.ready_sequence += 1

    def refill_tokens(self, now):
        if self.capacity_bytes_per_second <= 0.0:
            return
        elapsed = max(0.0, now - self.last_token_time)
        self.tokens = min(
            self.max_tokens,
            self.tokens + elapsed * self.capacity_bytes_per_second,
        )
        self.last_token_time = now

    def send_ready(self, now):
        self.refill_tokens(now)
        while self.ready:
            received_at, _sequence, camera_name, payload = self.ready[0]
            age = now - received_at
            if age > self.max_queue_delay_s:
                heapq.heappop(self.ready)
                with self.lock:
                    self.metrics[camera_name]["congestion_drops"] += 1
                continue
            if self.capacity_bytes_per_second > 0.0 and self.tokens < len(payload):
                break
            heapq.heappop(self.ready)
            if self.capacity_bytes_per_second > 0.0:
                self.tokens -= len(payload)
            self.send_socket.sendto(
                payload, (self.host, self.output_port(camera_name))
            )
            with self.lock:
                metric = self.metrics[camera_name]
                metric["output_packets"] += 1
                metric["output_bytes"] += len(payload)
                metric["last_packet_delay_ms"] = age * 1000.0
                metric["last_output_at"] = now

    def run(self):
        while not self.stop_event.is_set():
            try:
                self.receive_packets()
                now = time.monotonic()
                self.collect_ready(now)
                self.send_ready(now)
            except OSError:
                if not self.stop_event.is_set():
                    time.sleep(0.01)

    def snapshot(self):
        with self.lock:
            metrics = {name: dict(values) for name, values in self.metrics.items()}
        for name in self.camera_names:
            metrics[name]["pending_packets"] = self.queues[name].pending_count
        metrics["ready_packets"] = len(self.ready)
        return metrics

    def close(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        for udp_socket in self.sockets.values():
            try:
                self.selector.unregister(udp_socket)
            except Exception:
                pass
            udp_socket.close()
        self.selector.close()
        self.send_socket.close()


class CameraCodecPipeline:
    def __init__(
        self,
        node,
        camera_name,
        publisher,
        proxy,
        ffmpeg_path,
        encoder,
        frame_rate,
        bitrate_kbps,
        network_delay_ms,
        decode_locally=True,
    ):
        self.node = node
        self.camera_name = camera_name
        self.publisher = publisher
        self.proxy = proxy
        self.ffmpeg_path = ffmpeg_path
        self.encoder = encoder
        self.frame_rate = float(frame_rate)
        self.bitrate_kbps = int(bitrate_kbps)
        self.decode_locally = bool(decode_locally)
        self.expected_latency_s = max(0.0, float(network_delay_ms) * 0.001) + (
            1.5 / max(1.0, self.frame_rate)
        )
        self.frame_queue = queue.Queue(maxsize=1)
        self.sent_timestamps = collections.deque()
        self.timestamp_lock = threading.Lock()
        self.metric_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.encoder_process = None
        self.decoder_process = None
        self.writer_thread = None
        self.reader_thread = None
        self.started = False
        self.width = 0
        self.height = 0
        self.input_pixel_format = ""
        self.input_count = 0
        self.last_input_at = None
        self.encoded_count = 0
        self.decoded_count = 0
        self.input_queue_drops = 0
        self.last_output_at = None
        self.last_end_to_end_ms = None
        self.last_error = ""
        self.encoder_fallback_reason = ""
        self.encoder_fallback_attempted = False
        self.timestamp_resync_drops = 0

    def start(self, width, height, pixel_format):
        if self.started:
            return
        self.width = int(width)
        self.height = int(height)
        self.input_pixel_format = pixel_format
        keyframe_interval = max(1, int(round(self.frame_rate / 3.0)))
        output_url = (
            f"udp://127.0.0.1:{self.proxy.input_port(self.camera_name)}"
            "?pkt_size=1316"
        )
        input_url = (
            f"udp://127.0.0.1:{self.proxy.output_port(self.camera_name)}"
            "?fifo_size=2000000&overrun_nonfatal=1"
        )
        decoder_command = [
            self.ffmpeg_path,
            "-hide_banner",
            "-loglevel",
            "error",
            "-fflags",
            "nobuffer+discardcorrupt+flush_packets",
            "-flags",
            "low_delay",
            "-avioflags",
            "direct",
            "-max_delay",
            "0",
            "-probesize",
            "32",
            "-analyzeduration",
            "0",
            "-fpsprobesize",
            "0",
            "-threads",
            "1",
            "-f",
            "h264",
            "-i",
            input_url,
            "-an",
            "-flush_packets",
            "1",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-vsync",
            "0",
            "pipe:1",
        ]
        self.encoder_command_prefix = [
            self.ffmpeg_path,
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "rawvideo",
            "-pix_fmt",
            pixel_format,
            "-s",
            f"{self.width}x{self.height}",
            "-r",
            f"{self.frame_rate:g}",
            "-i",
            "pipe:0",
            "-an",
        ]
        self.encoder_command_suffix = [
            "-vsync", "0", "-flush_packets", "1",
            "-bsf:v", "dump_extra=freq=keyframe",
            "-f", "h264", output_url
        ]
        self.keyframe_interval = keyframe_interval
        if self.decode_locally:
            self.decoder_process = subprocess.Popen(
                decoder_command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        # Do not send SPS/PPS/IDR until the local or C++ decoder has bound UDP.
        if not wait_for_udp_listener(self.proxy.output_port(self.camera_name), 3.0):
            self.last_error = "decoder UDP listener was not ready before encoder start"
        self.launch_encoder(self.encoder)
        self.started = True
        self.writer_thread = threading.Thread(
            target=self.write_frames,
            name=f"h264_writer_{self.camera_name}",
            daemon=True,
        )
        self.writer_thread.start()
        if self.decode_locally:
            self.reader_thread = threading.Thread(
                target=self.read_frames,
                name=f"h264_reader_{self.camera_name}",
                daemon=True,
            )
            self.reader_thread.start()

    def launch_encoder(self, encoder):
        command = [
            *self.encoder_command_prefix,
            *encoder_arguments(encoder, self.bitrate_kbps, self.keyframe_interval),
            *self.encoder_command_suffix,
        ]
        self.encoder = encoder
        self.encoder_process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            bufsize=0,
        )

    def encoder_failure(self, fallback_text=""):
        process = self.encoder_process
        return_code = None if process is None else process.poll()
        detail = fallback_text
        if process is not None and return_code is not None and process.stderr is not None:
            try:
                stderr = process.stderr.read().decode("utf-8", errors="replace").strip()
                if stderr:
                    detail = stderr.splitlines()[-1]
            except OSError:
                pass
        prefix = f"{self.encoder} exited"
        if return_code is not None:
            prefix += f" with code {return_code}"
        return f"{prefix}: {detail}" if detail else prefix

    def recover_encoder(self, reason):
        if self.encoder != "h264_nvenc" or self.encoder_fallback_attempted:
            self.last_error = reason
            self.node.get_logger().error(
                f"H.264 encoder [{self.camera_name}] failed: {reason}"
            )
            return False
        self.encoder_fallback_attempted = True
        self.encoder_fallback_reason = reason
        self.node.get_logger().warning(
            f"H.264 encoder [{self.camera_name}] cannot use NVENC; "
            "falling back to libx264"
        )
        try:
            self.launch_encoder("libx264")
            self.last_error = ""
            return True
        except OSError as exc:
            self.last_error = f"libx264 fallback failed: {exc}"
            self.node.get_logger().error(self.last_error)
            return False

    def submit(self, payload, width, height, pixel_format, received_at):
        with self.metric_lock:
            self.input_count += 1
            self.last_input_at = received_at
        if not self.started:
            self.start(width, height, pixel_format)
        if width != self.width or height != self.height or pixel_format != self.input_pixel_format:
            self.last_error = "camera format changed after encoder startup"
            return
        item = (payload, received_at)
        try:
            self.frame_queue.put_nowait(item)
        except queue.Full:
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                pass
            with self.metric_lock:
                self.input_queue_drops += 1
            self.frame_queue.put_nowait(item)

    def write_frames(self):
        while not self.stop_event.is_set():
            try:
                item = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                break
            payload, received_at = item
            while not self.stop_event.is_set():
                process = self.encoder_process
                if process is None or process.poll() is not None:
                    if self.recover_encoder(self.encoder_failure()):
                        continue
                    return
                try:
                    process.stdin.write(payload)
                    process.stdin.flush()
                    with self.timestamp_lock:
                        self.sent_timestamps.append(received_at)
                        while len(self.sent_timestamps) > 120:
                            self.sent_timestamps.popleft()
                    with self.metric_lock:
                        self.encoded_count += 1
                    break
                except (BrokenPipeError, OSError) as exc:
                    if self.recover_encoder(self.encoder_failure(str(exc))):
                        continue
                    return

    def read_exact(self, size):
        chunks = []
        remaining = size
        while remaining > 0 and not self.stop_event.is_set():
            data = self.decoder_process.stdout.read(remaining)
            if not data:
                return None
            chunks.append(data)
            remaining -= len(data)
        return b"".join(chunks) if remaining == 0 else None

    def read_frames(self):
        frame_size = self.width * self.height * 3
        while not self.stop_event.is_set():
            payload = self.read_exact(frame_size)
            if payload is None:
                if not self.stop_event.is_set():
                    self.last_error = "decoder stopped"
                break
            now = time.monotonic()
            with self.timestamp_lock:
                while len(self.sent_timestamps) > 1:
                    first_error = abs(
                        (now - self.sent_timestamps[0]) - self.expected_latency_s
                    )
                    next_error = abs(
                        (now - self.sent_timestamps[1]) - self.expected_latency_s
                    )
                    if next_error >= first_error:
                        break
                    self.sent_timestamps.popleft()
                    self.timestamp_resync_drops += 1
                source_time = self.sent_timestamps.popleft() if self.sent_timestamps else None
            message = Image()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.header.frame_id = f"{self.camera_name}_camera_remote"
            message.height = self.height
            message.width = self.width
            message.encoding = "bgr8"
            message.is_bigendian = 0
            message.step = self.width * 3
            message.data = payload
            if self.stop_event.is_set() or not rclpy.ok():
                break
            try:
                self.publisher.publish(message)
            except Exception as exc:
                if self.stop_event.is_set() or not rclpy.ok():
                    break
                self.last_error = f"decoded image publish failed: {exc}"
                break
            with self.metric_lock:
                self.decoded_count += 1
                self.last_output_at = now
                self.last_end_to_end_ms = (
                    None if source_time is None else (now - source_time) * 1000.0
                )

    def snapshot(self):
        encoder_return_code = (
            None if self.encoder_process is None else self.encoder_process.poll()
        )
        with self.metric_lock:
            return {
                "started": self.started,
                "width": self.width,
                "height": self.height,
                "input_count": self.input_count,
                "last_input_at": self.last_input_at,
                "encoded_count": self.encoded_count,
                "decoded_count": self.decoded_count,
                "input_queue_drops": self.input_queue_drops,
                "last_output_at": self.last_output_at,
                "last_end_to_end_ms": self.last_end_to_end_ms,
                "last_error": self.last_error,
                "encoder": self.encoder,
                "encoder_running": self.started and encoder_return_code is None,
                "encoder_return_code": encoder_return_code,
                "encoder_fallback_reason": self.encoder_fallback_reason,
                "timestamp_resync_drops": self.timestamp_resync_drops,
            }

    def close_process(self, process):
        if process is None:
            return
        if process.stdin is not None:
            try:
                process.stdin.close()
            except OSError:
                pass
        if process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=1.0)
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            process.kill()
            try:
                process.wait(timeout=1.0)
            except (subprocess.TimeoutExpired, KeyboardInterrupt):
                pass

    def close(self):
        self.stop_event.set()
        try:
            self.frame_queue.put_nowait(None)
        except queue.Full:
            pass
        self.close_process(self.encoder_process)
        self.close_process(self.decoder_process)
        for thread in (self.writer_thread, self.reader_thread):
            if thread is not None:
                thread.join(timeout=1.0)


class H264VideoLink(Node):
    """Encode ROS camera feeds as low-latency H.264 over delayed UDP."""

    def __init__(self):
        super().__init__("remote_h264_video_link")
        self.declare_parameter("input_prefix", "/sensing/camera")
        self.declare_parameter("output_prefix", "/remote/video")
        self.declare_parameter("camera_name", "")
        self.declare_parameter("encoder", "h264_nvenc")
        self.declare_parameter("frame_rate", 15.0)
        self.declare_parameter("bitrate_kbps", 1500)
        self.declare_parameter("network_delay_ms", 50.0)
        self.declare_parameter("network_jitter_ms", 0.0)
        self.declare_parameter("network_loss_percent", 0.0)
        self.declare_parameter("network_seed", 6100)
        self.declare_parameter("link_capacity_mbps", 100.0)
        self.declare_parameter("max_queue_delay_ms", 500.0)
        # Keep fixed video ports below Linux's default ephemeral range
        # (32768-60999), otherwise unrelated ROS sockets can randomly claim them.
        self.declare_parameter("proxy_input_base_port", 15100)
        self.declare_parameter("decoder_input_base_port", 15200)
        self.declare_parameter("stale_after_s", 1.0)
        self.declare_parameter("decode_locally", True)
        self.declare_parameter("subscription_retry_s", 3.0)

        self.bridge = CvBridge()
        self.input_prefix = str(self.get_parameter("input_prefix").value).rstrip("/")
        self.output_prefix = str(self.get_parameter("output_prefix").value).rstrip("/")
        self.frame_rate = float(self.get_parameter("frame_rate").value)
        self.bitrate_kbps = int(self.get_parameter("bitrate_kbps").value)
        self.stale_after_s = float(self.get_parameter("stale_after_s").value)
        self.subscription_retry_s = max(
            1.0, float(self.get_parameter("subscription_retry_s").value)
        )
        self.decode_locally = bool(self.get_parameter("decode_locally").value)
        requested_camera = str(self.get_parameter("camera_name").value).strip()
        if requested_camera and requested_camera not in CAMERA_NAMES:
            raise ValueError(f"unknown camera_name: {requested_camera}")
        self.camera_names = (requested_camera,) if requested_camera else CAMERA_NAMES
        self.ffmpeg_path = shutil.which("ffmpeg")
        if self.ffmpeg_path is None:
            raise RuntimeError("ffmpeg is required for the H.264 remote video link")

        requested_encoder = str(self.get_parameter("encoder").value)
        if encoder_works(self.ffmpeg_path, requested_encoder):
            self.encoder = requested_encoder
        elif requested_encoder != "libx264" and encoder_works(self.ffmpeg_path, "libx264"):
            self.encoder = "libx264"
            self.get_logger().warning(
                f"{requested_encoder} is unavailable; falling back to libx264"
            )
        else:
            raise RuntimeError("neither requested H.264 encoder nor libx264 is usable")

        self.proxy = EncodedPacketProxy(
            host="127.0.0.1",
            input_base_port=self.get_parameter("proxy_input_base_port").value,
            output_base_port=self.get_parameter("decoder_input_base_port").value,
            delay_ms=self.get_parameter("network_delay_ms").value,
            jitter_ms=self.get_parameter("network_jitter_ms").value,
            loss_percent=self.get_parameter("network_loss_percent").value,
            seed=self.get_parameter("network_seed").value,
            capacity_mbps=self.get_parameter("link_capacity_mbps").value,
            max_queue_delay_ms=self.get_parameter("max_queue_delay_ms").value,
            camera_names=self.camera_names,
        )
        self.proxy.start()

        self.camera_output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.camera_input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.camera_publishers = (
            {
                name: self.create_publisher(
                    Image, f"{self.output_prefix}/{name}/image", self.camera_output_qos
                )
                for name in self.camera_names
            }
            if self.decode_locally
            else {name: None for name in self.camera_names}
        )
        self.pipelines = {
            name: CameraCodecPipeline(
                node=self,
                camera_name=name,
                publisher=self.camera_publishers[name],
                proxy=self.proxy,
                ffmpeg_path=self.ffmpeg_path,
                encoder=self.encoder,
                frame_rate=self.frame_rate,
                bitrate_kbps=self.bitrate_kbps,
                network_delay_ms=self.get_parameter("network_delay_ms").value,
                decode_locally=self.decode_locally,
            )
            for name in self.camera_names
        }
        self.started_at = time.monotonic()
        self.subscription_last_reset = {
            name: self.started_at for name in self.camera_names
        }
        self.subscription_reset_count = {name: 0 for name in self.camera_names}
        self.camera_subscriptions = {
            name: self.create_camera_subscription(name) for name in self.camera_names
        }
        self.diagnostic_pub = self.create_publisher(
            String, "/remote/network/video_state", 10
        )
        self.previous_counts = {
            name: (0, 0, 0, 0, 0) for name in self.camera_names
        }
        self.previous_metric_time = time.monotonic()
        self.diagnostic_timer = self.create_timer(1.0, self.publish_diagnostic)
        self.get_logger().info(
            "H.264 remote video [%s]: %s, %d kbit/s, %.1f ms network delay, "
            "%.1f Mbit/s link capacity"
            % (
                ",".join(self.camera_names),
                self.encoder,
                self.bitrate_kbps,
                float(self.get_parameter("network_delay_ms").value),
                float(self.get_parameter("link_capacity_mbps").value),
            )
        )

    def image_payload(self, msg):
        encoding = str(msg.encoding).lower()
        expected_step = int(msg.width) * 3
        if encoding in ("rgb8", "bgr8"):
            pixel_format = "rgb24" if encoding == "rgb8" else "bgr24"
            raw = bytes(msg.data)
            if int(msg.step) == expected_step:
                return raw, pixel_format
            rows = [
                raw[row * int(msg.step) : row * int(msg.step) + expected_step]
                for row in range(int(msg.height))
            ]
            return b"".join(rows), pixel_format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return frame.tobytes(), "bgr24"

    def create_camera_subscription(self, camera_name):
        return self.create_subscription(
            Image,
            f"{self.input_prefix}/{camera_name}/image",
            lambda msg, name=camera_name: self.on_image(name, msg),
            self.camera_input_qos,
        )

    def refresh_stalled_subscriptions(self, now):
        for name in self.camera_names:
            codec = self.pipelines[name].snapshot()
            last_input_at = codec["last_input_at"]
            reference_time = max(
                self.subscription_last_reset[name],
                last_input_at if last_input_at is not None else 0.0,
            )
            input_age = now - reference_time
            if input_age < self.subscription_retry_s:
                continue
            old_subscription = self.camera_subscriptions[name]
            self.destroy_subscription(old_subscription)
            self.camera_subscriptions[name] = self.create_camera_subscription(name)
            self.subscription_last_reset[name] = now
            self.subscription_reset_count[name] += 1
            self.get_logger().warning(
                "Camera input [%s] has been silent for %.1f s; rebuilding ROS subscription"
                % (name, input_age)
            )

    def on_image(self, camera_name, msg):
        try:
            payload, pixel_format = self.image_payload(msg)
            self.pipelines[camera_name].submit(
                payload,
                msg.width,
                msg.height,
                pixel_format,
                time.monotonic(),
            )
        except Exception as exc:
            self.pipelines[camera_name].last_error = str(exc)

    def publish_diagnostic(self):
        now = time.monotonic()
        self.refresh_stalled_subscriptions(now)
        elapsed = max(1.0e-6, now - self.previous_metric_time)
        proxy = self.proxy.snapshot()
        cameras = {}
        any_live = False
        for name in self.camera_names:
            codec = self.pipelines[name].snapshot()
            packet = proxy[name]
            (
                old_input,
                old_encoded,
                old_decoded,
                old_input_bytes,
                old_output_bytes,
            ) = self.previous_counts[name]
            input_fps = (codec["input_count"] - old_input) / elapsed
            encoded_fps = (codec["encoded_count"] - old_encoded) / elapsed
            output_fps = (codec["decoded_count"] - old_decoded) / elapsed
            encoded_bitrate_mbps = (
                (packet["input_bytes"] - old_input_bytes) * 8.0 / elapsed / 1.0e6
            )
            bitrate_mbps = (
                (packet["output_bytes"] - old_output_bytes) * 8.0 / elapsed / 1.0e6
            )
            self.previous_counts[name] = (
                codec["input_count"],
                codec["encoded_count"],
                codec["decoded_count"],
                packet["input_bytes"],
                packet["output_bytes"],
            )
            last_output_at = (
                codec["last_output_at"]
                if self.decode_locally
                else packet["last_output_at"]
            )
            output_age_s = None if last_output_at is None else now - last_output_at
            live = output_age_s is not None and output_age_s <= self.stale_after_s
            any_live = any_live or live
            camera_state = {
                "encoder": codec["encoder"],
                "encoder_running": codec["encoder_running"],
                "encoder_return_code": codec["encoder_return_code"],
                "encoder_fallback_reason": codec["encoder_fallback_reason"],
                "last_input_age_ms": (
                    None
                    if codec["last_input_at"] is None
                    else (now - codec["last_input_at"]) * 1000.0
                ),
                "subscription_resets": self.subscription_reset_count[name],
                "input_fps": input_fps,
                "encoded_fps": encoded_fps,
                "bitrate_mbps": bitrate_mbps,
                "encoded_bitrate_mbps": encoded_bitrate_mbps,
                "last_packet_delay_ms": packet["last_packet_delay_ms"],
                "input_queue_drops": codec["input_queue_drops"],
                "network_drops": packet["network_drops"],
                "congestion_drops": packet["congestion_drops"],
                "last_error": codec["last_error"],
                "timestamp_resync_drops": codec["timestamp_resync_drops"],
            }
            if self.decode_locally:
                camera_state.update(
                    {
                        "state": (
                            "LIVE"
                            if live
                            else ("STALE" if codec["started"] else "WAITING")
                        ),
                        "output_fps": output_fps,
                        "last_frame_age_ms": (
                            None if output_age_s is None else output_age_s * 1000.0
                        ),
                        "last_delivery_delay_ms": codec["last_end_to_end_ms"],
                        "estimated_end_to_end_ms": codec["last_end_to_end_ms"],
                    }
                )
            else:
                camera_state["tx_state"] = (
                    "LIVE" if live else ("STALE" if codec["started"] else "WAITING")
                )
            cameras[name] = camera_state
        self.previous_metric_time = now
        state = {
            "link": "LIVE" if any_live else "WAITING",
            "transport": "H264/UDP/ANNEXB",
            "role": "sender" if not self.decode_locally else "loopback",
            "encoder": self.encoder,
            "bitrate_kbps_per_camera": self.bitrate_kbps,
            "link_capacity_mbps": float(self.get_parameter("link_capacity_mbps").value),
            "configured_delay_ms": float(self.get_parameter("network_delay_ms").value),
            "configured_jitter_ms": float(self.get_parameter("network_jitter_ms").value),
            "configured_loss_percent": float(
                self.get_parameter("network_loss_percent").value
            ),
            "pending_packets": sum(
                proxy[name]["pending_packets"] for name in self.camera_names
            )
            + proxy["ready_packets"],
            "cameras": cameras,
        }
        message = String()
        message.data = json.dumps(state, separators=(",", ":"))
        self.diagnostic_pub.publish(message)

    def destroy_node(self):
        for pipeline in self.pipelines.values():
            pipeline.close()
        self.proxy.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = H264VideoLink()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
