import json
import time

from agv_remote_protocol.network_impairment import NetworkImpairmentQueue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class VideoNetworkEmulator(Node):
    """Relay camera frames through a configurable application-level network."""

    CAMERA_NAMES = (
        "front",
        "rear",
        "left_front",
        "left_rear",
        "right_front",
        "right_rear",
    )

    def __init__(self):
        super().__init__("remote_video_network_emulator")
        self.declare_parameter("input_prefix", "/sensing/camera")
        self.declare_parameter("output_prefix", "/remote/video")
        self.declare_parameter("network_delay_ms", 50.0)
        self.declare_parameter("network_jitter_ms", 0.0)
        self.declare_parameter("network_loss_percent", 0.0)
        self.declare_parameter("network_seed", 6000)
        self.declare_parameter("delivery_rate", 200.0)
        self.declare_parameter("diagnostic_rate", 2.0)
        self.declare_parameter("stale_after_s", 1.0)

        self.input_prefix = str(self.get_parameter("input_prefix").value).rstrip("/")
        self.output_prefix = str(self.get_parameter("output_prefix").value).rstrip("/")
        self.stale_after_s = max(
            0.1, float(self.get_parameter("stale_after_s").value)
        )
        self.network = NetworkImpairmentQueue(
            delay_ms=self.get_parameter("network_delay_ms").value,
            jitter_ms=self.get_parameter("network_jitter_ms").value,
            loss_percent=self.get_parameter("network_loss_percent").value,
            seed=self.get_parameter("network_seed").value,
        )

        self.input_counts = {name: 0 for name in self.CAMERA_NAMES}
        self.output_counts = {name: 0 for name in self.CAMERA_NAMES}
        self.dropped_counts = {name: 0 for name in self.CAMERA_NAMES}
        self.last_output_at = {name: None for name in self.CAMERA_NAMES}
        self.last_delivery_delay_ms = {name: None for name in self.CAMERA_NAMES}
        self.input_fps = {name: 0.0 for name in self.CAMERA_NAMES}
        self.output_fps = {name: 0.0 for name in self.CAMERA_NAMES}
        self.metric_counts = {
            name: (0, 0) for name in self.CAMERA_NAMES
        }
        self.metric_time = time.monotonic()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.camera_publishers = {
            name: self.create_publisher(
                Image, f"{self.output_prefix}/{name}/image", qos
            )
            for name in self.CAMERA_NAMES
        }
        self.camera_subscriptions = [
            self.create_subscription(
                Image,
                f"{self.input_prefix}/{name}/image",
                lambda msg, camera_name=name: self.on_image(camera_name, msg),
                qos,
            )
            for name in self.CAMERA_NAMES
        ]
        self.diagnostic_pub = self.create_publisher(
            String, "/remote/network/video_state", 10
        )

        delivery_rate = max(20.0, float(self.get_parameter("delivery_rate").value))
        diagnostic_rate = max(0.5, float(self.get_parameter("diagnostic_rate").value))
        self.delivery_timer = self.create_timer(1.0 / delivery_rate, self.deliver_ready)
        self.diagnostic_timer = self.create_timer(
            1.0 / diagnostic_rate, self.publish_diagnostic
        )
        self.get_logger().info(
            "Six-camera network emulator: %.1f ms delay, %.1f ms jitter, %.2f%% loss"
            % (
                self.network.delay_ms,
                self.network.jitter_ms,
                self.network.loss_percent,
            )
        )

    def on_image(self, camera_name, msg):
        received_at = time.monotonic()
        self.input_counts[camera_name] += 1
        if not self.network.enqueue((camera_name, msg, received_at)):
            self.dropped_counts[camera_name] += 1

    def deliver_ready(self):
        now = time.monotonic()
        for _release_at, (camera_name, msg, received_at) in self.network.pop_ready(now):
            self.camera_publishers[camera_name].publish(msg)
            self.output_counts[camera_name] += 1
            self.last_output_at[camera_name] = now
            self.last_delivery_delay_ms[camera_name] = (now - received_at) * 1000.0

    def update_rates(self, now):
        elapsed = now - self.metric_time
        if elapsed < 1.0:
            return
        for name in self.CAMERA_NAMES:
            previous_input, previous_output = self.metric_counts[name]
            self.input_fps[name] = (
                self.input_counts[name] - previous_input
            ) / elapsed
            self.output_fps[name] = (
                self.output_counts[name] - previous_output
            ) / elapsed
            self.metric_counts[name] = (
                self.input_counts[name],
                self.output_counts[name],
            )
        self.metric_time = now

    def publish_diagnostic(self):
        now = time.monotonic()
        self.update_rates(now)
        cameras = {}
        any_live = False
        for name in self.CAMERA_NAMES:
            last_output = self.last_output_at[name]
            age_s = None if last_output is None else now - last_output
            live = age_s is not None and age_s <= self.stale_after_s
            any_live = any_live or live
            cameras[name] = {
                "state": "LIVE" if live else ("STALE" if last_output else "WAITING"),
                "input_fps": self.input_fps[name],
                "output_fps": self.output_fps[name],
                "last_frame_age_ms": None if age_s is None else age_s * 1000.0,
                "last_delivery_delay_ms": self.last_delivery_delay_ms[name],
                "input_count": self.input_counts[name],
                "output_count": self.output_counts[name],
                "dropped_count": self.dropped_counts[name],
            }
        state = {
            "link": "LIVE" if any_live else "WAITING",
            "configured_delay_ms": self.network.delay_ms,
            "configured_jitter_ms": self.network.jitter_ms,
            "configured_loss_percent": self.network.loss_percent,
            "pending_frames": self.network.pending_count,
            "cameras": cameras,
        }
        msg = String()
        msg.data = json.dumps(state, separators=(",", ":"))
        self.diagnostic_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VideoNetworkEmulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
