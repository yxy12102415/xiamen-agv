import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from agv_remote_drive.h264_video_link import CAMERA_NAMES, EncodedPacketProxy


class UdpVideoProxy(Node):
    def __init__(self):
        super().__init__("remote_h264_udp_proxy")
        self.declare_parameter("input_base_port", 15100)
        self.declare_parameter("output_base_port", 15200)
        self.declare_parameter("delay_ms", 50.0)
        self.declare_parameter("jitter_ms", 0.0)
        self.declare_parameter("loss_percent", 0.0)
        self.declare_parameter("seed", 7100)
        self.declare_parameter("capacity_mbps", 100.0)
        self.declare_parameter("max_queue_delay_ms", 250.0)

        self.proxy = EncodedPacketProxy(
            host="127.0.0.1",
            input_base_port=int(self.get_parameter("input_base_port").value),
            output_base_port=int(self.get_parameter("output_base_port").value),
            delay_ms=float(self.get_parameter("delay_ms").value),
            jitter_ms=float(self.get_parameter("jitter_ms").value),
            loss_percent=float(self.get_parameter("loss_percent").value),
            seed=int(self.get_parameter("seed").value),
            capacity_mbps=float(self.get_parameter("capacity_mbps").value),
            max_queue_delay_ms=float(self.get_parameter("max_queue_delay_ms").value),
        )
        self.proxy.start()
        self.diagnostic_pub = self.create_publisher(
            String, "/remote/network/video_state", 10
        )
        self.diagnostic_timer = self.create_timer(1.0, self.publish_diagnostic)
        self.get_logger().info(
            "Direct H.264 UDP proxy: 15100-15105 -> 15200-15205, "
            f"{float(self.get_parameter('delay_ms').value):.1f} ms delay"
        )

    def publish_diagnostic(self):
        snapshot = self.proxy.snapshot()
        cameras = {}
        for name in CAMERA_NAMES:
            metric = snapshot[name]
            cameras[name] = {
                "state": "LIVE" if metric["output_packets"] else "WAITING",
                "input_packets": metric["input_packets"],
                "output_packets": metric["output_packets"],
                "network_drops": metric["network_drops"],
                "congestion_drops": metric["congestion_drops"],
                "last_packet_delay_ms": metric["last_packet_delay_ms"],
            }
        message = String()
        message.data = json.dumps(
            {
                "link": "LIVE" if any(
                    camera["output_packets"] for camera in cameras.values()
                ) else "WAITING",
                "transport": "H264/UDP/ANNEXB",
                "role": "network",
                "cameras": cameras,
            },
            separators=(",", ":"),
        )
        self.diagnostic_pub.publish(message)

    def destroy_node(self):
        self.proxy.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = UdpVideoProxy()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
