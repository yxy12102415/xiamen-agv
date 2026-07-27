import json
import socket
import time

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String, UInt8MultiArray

from .protocol import ITEM_REMOTE_COMMAND
from .protocol import ITEM_REMOTE_DRIVING
from .protocol import ITEM_VEHICLE_STATE
from .protocol import MSG_DOMAIN_COMMAND
from .protocol import MSG_VEHICLE_STATE
from .protocol import ProtocolError
from .protocol import pack_packet
from .protocol import pack_remote_command
from .protocol import pack_remote_driving
from .protocol import unpack_packet
from .protocol import unpack_vehicle_state
from .network_impairment import NetworkImpairmentQueue


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class DomainUdpGateway(Node):
    """Translate ROS operator commands to the documented 40 Hz UDP/TLV protocol."""

    def __init__(self):
        super().__init__("domain_udp_gateway")
        self.declare_parameter("local_host", "127.0.0.1")
        self.declare_parameter("local_port", 46000)
        self.declare_parameter("vehicle_host", "127.0.0.1")
        self.declare_parameter("vehicle_port", 46001)
        self.declare_parameter("endian", "little")
        self.declare_parameter("tlv_length_includes_header", False)
        self.declare_parameter("send_rate", 40.0)
        self.declare_parameter("operator_timeout", 0.25)
        self.declare_parameter("vehicle_state_timeout", 0.5)
        self.declare_parameter("network_delay_ms", 50.0)
        self.declare_parameter("network_jitter_ms", 0.0)
        self.declare_parameter("network_loss_percent", 0.0)
        self.declare_parameter("network_seed", 7100)
        self.declare_parameter("network_delivery_rate", 200.0)
        self.declare_parameter("igv_id", 1)
        self.declare_parameter("control_mode", 5)
        self.declare_parameter("obstacle_bypass", False)
        self.declare_parameter("max_velocity", 0.5)
        self.declare_parameter("max_reverse_velocity", 0.3)
        self.declare_parameter("front_steer_limit", 0.47822)
        self.declare_parameter("max_brake_decel", 1.0)
        self.declare_parameter("control_topic", "/remote/operator/control_cmd")
        self.declare_parameter("two_axis_topic", "/remote/operator/two_axis_steer_cmd")
        self.declare_parameter("gear_topic", "/remote/operator/gear_cmd")
        self.declare_parameter("deadman_topic", "/remote/operator/deadman")
        self.declare_parameter("estop_topic", "/remote/operator/emergency_stop")
        self.declare_parameter("vehicle_state_topic", "/remote/protocol/vehicle_state")
        self.declare_parameter("diagnostic_topic", "/remote/protocol/domain_state")

        self.local_host = str(self.get_parameter("local_host").value)
        self.local_port = int(self.get_parameter("local_port").value)
        self.vehicle_address = (
            str(self.get_parameter("vehicle_host").value),
            int(self.get_parameter("vehicle_port").value),
        )
        self.endian = str(self.get_parameter("endian").value)
        self.length_includes_header = bool(
            self.get_parameter("tlv_length_includes_header").value
        )
        self.operator_timeout = float(self.get_parameter("operator_timeout").value)
        self.vehicle_state_timeout = float(
            self.get_parameter("vehicle_state_timeout").value
        )
        self.igv_id = int(self.get_parameter("igv_id").value)
        self.control_mode = int(self.get_parameter("control_mode").value)
        self.obstacle_bypass = bool(self.get_parameter("obstacle_bypass").value)
        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_reverse_velocity = float(
            self.get_parameter("max_reverse_velocity").value
        )
        self.front_steer_limit = float(self.get_parameter("front_steer_limit").value)
        self.max_brake_decel = float(self.get_parameter("max_brake_decel").value)

        self.control = None
        self.two_axis = None
        self.gear = None
        self.deadman = False
        self.estop = False
        self.times = {}
        self.message_id = 0
        self.tx_count = 0
        self.rx_count = 0
        self.rx_error_count = 0
        self.last_vehicle_state_time = None
        self.last_vehicle_state = None
        self.last_vehicle_state_latency_ms = None
        self.outbound_network = NetworkImpairmentQueue(
            delay_ms=self.get_parameter("network_delay_ms").value,
            jitter_ms=self.get_parameter("network_jitter_ms").value,
            loss_percent=self.get_parameter("network_loss_percent").value,
            seed=self.get_parameter("network_seed").value,
        )

        self.create_subscription(
            Control,
            self.get_parameter("control_topic").value,
            lambda msg: self.remember("control", msg),
            1,
        )
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter("two_axis_topic").value,
            lambda msg: self.remember("two_axis", msg),
            1,
        )
        self.create_subscription(
            GearCommand,
            self.get_parameter("gear_topic").value,
            lambda msg: self.remember("gear", msg),
            1,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("deadman_topic").value,
            self.on_deadman,
            1,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("estop_topic").value,
            self.on_estop,
            1,
        )

        self.vehicle_state_pub = self.create_publisher(
            String, self.get_parameter("vehicle_state_topic").value, 10
        )
        self.diagnostic_pub = self.create_publisher(
            String, self.get_parameter("diagnostic_topic").value, 10
        )
        self.tx_pub = self.create_publisher(UInt8MultiArray, "/remote/protocol/tx", 10)
        self.rx_pub = self.create_publisher(UInt8MultiArray, "/remote/protocol/rx", 10)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.local_host, self.local_port))
        self.socket.setblocking(False)

        send_rate = float(self.get_parameter("send_rate").value)
        self.timer = self.create_timer(1.0 / max(1.0, send_rate), self.on_timer)
        delivery_rate = float(self.get_parameter("network_delivery_rate").value)
        self.network_timer = self.create_timer(
            1.0 / max(send_rate, delivery_rate), self.on_network_timer
        )
        self.get_logger().info(
            "Domain UDP gateway %s:%d -> %s:%d at %.1f Hz, %.1f ms one-way delay "
            "(%s-endian prototype)"
            % (
                self.local_host,
                self.local_port,
                self.vehicle_address[0],
                self.vehicle_address[1],
                send_rate,
                self.outbound_network.delay_ms,
                self.endian,
            )
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def remember(self, name, msg):
        setattr(self, name, msg)
        self.times[name] = self.now_seconds()

    def on_deadman(self, msg):
        self.deadman = bool(msg.data)
        self.times["deadman"] = self.now_seconds()

    def on_estop(self, msg):
        self.estop = bool(msg.data)
        self.times["estop"] = self.now_seconds()

    def fresh(self, name):
        timestamp = self.times.get(name)
        return timestamp is not None and self.now_seconds() - timestamp <= self.operator_timeout

    def operator_ready(self):
        return (
            self.fresh("control")
            and self.fresh("gear")
            and self.fresh("deadman")
            and self.deadman
            and not self.estop
        )

    def protocol_gear(self):
        if self.gear is None:
            return 5
        return {
            GearCommand.DRIVE: 1,
            GearCommand.REVERSE: 2,
            GearCommand.PARK: 5,
            GearCommand.NEUTRAL: 5,
        }.get(int(self.gear.command), 5)

    def command_values(self):
        ready = self.operator_ready()
        velocity = float(self.control.longitudinal.velocity) if ready else 0.0
        acceleration = float(self.control.longitudinal.acceleration) if ready else 0.0
        steering = float(self.control.lateral.steering_tire_angle) if ready else 0.0
        steering_scale = 100.0 * steering / max(1.0e-6, self.front_steer_limit)
        velocity_limit = self.max_velocity if velocity >= 0.0 else self.max_reverse_velocity
        speed_scale = 100.0 * velocity / max(1.0e-6, velocity_limit)
        braking = (
            not ready
            or (velocity >= 0.0 and acceleration < -1.0e-4)
            or (velocity < 0.0 and acceleration > 1.0e-4)
        )
        brake_scale = (
            100.0
            if not ready
            else 100.0 * abs(acceleration) / max(1.0e-6, self.max_brake_decel)
            if braking
            else 0.0
        )
        command = {
            "igv_id": self.igv_id,
            "control_mode": self.control_mode,
            "gear_command": self.protocol_gear(),
            "obstacle_bypass": int(self.obstacle_bypass),
            "estop": int(self.estop),
            "start_enable": int(ready),
        }
        driving = {
            "steering_scale": clamp(steering_scale, -100.0, 100.0),
            "speed_scale": clamp(speed_scale, -100.0, 100.0),
            "brake_scale": clamp(brake_scale, 0.0, 100.0),
            "buttons": [0] * 8,
        }
        return command, driving

    def send_command(self):
        command, driving = self.command_values()
        packet = pack_packet(
            MSG_DOMAIN_COMMAND,
            self.message_id,
            [
                (ITEM_REMOTE_COMMAND, pack_remote_command(command, self.endian)),
                (ITEM_REMOTE_DRIVING, pack_remote_driving(driving, self.endian)),
            ],
            self.endian,
            self.length_includes_header,
        )
        self.outbound_network.enqueue((packet, self.vehicle_address))
        self.message_id = (self.message_id + 1) & 0xFFFF
        debug = UInt8MultiArray()
        debug.data = list(packet)
        self.tx_pub.publish(debug)

    def flush_outbound(self):
        for _release_at, (packet, address) in self.outbound_network.pop_ready():
            self.socket.sendto(packet, address)
            self.tx_count += 1

    def receive_states(self):
        while True:
            try:
                data, address = self.socket.recvfrom(65535)
            except BlockingIOError:
                return
            debug = UInt8MultiArray()
            debug.data = list(data)
            self.rx_pub.publish(debug)
            try:
                packet = unpack_packet(data, self.endian, self.length_includes_header)
                if packet["msg_type"] != MSG_VEHICLE_STATE:
                    raise ProtocolError("unexpected msg_type 0x%04X" % packet["msg_type"])
                state_item = next(
                    value for item_type, value in packet["items"]
                    if item_type == ITEM_VEHICLE_STATE
                )
                state = unpack_vehicle_state(state_item, self.endian)
                packet_time = packet["sys_time_s"] + packet["sys_time_ms"] * 0.001
                state["message_id"] = packet["msg_id"]
                state["source"] = "%s:%d" % address
                state["transport_latency_ms"] = max(0.0, (time.time() - packet_time) * 1000.0)
                self.last_vehicle_state_latency_ms = state["transport_latency_ms"]
                self.last_vehicle_state = state
                self.last_vehicle_state_time = self.now_seconds()
                self.rx_count += 1
                msg = String()
                msg.data = json.dumps(state, separators=(",", ":"))
                self.vehicle_state_pub.publish(msg)
            except (ProtocolError, StopIteration) as exc:
                self.rx_error_count += 1
                self.get_logger().warning("Rejected vehicle-state datagram: %s" % exc)

    def publish_diagnostic(self):
        age = None
        if self.last_vehicle_state_time is not None:
            age = self.now_seconds() - self.last_vehicle_state_time
        state = {
            "link": "LIVE" if age is not None and age <= self.vehicle_state_timeout else "STALE",
            "operator_ready": self.operator_ready(),
            "tx_count": self.tx_count,
            "rx_count": self.rx_count,
            "rx_error_count": self.rx_error_count,
            "vehicle_state_age_s": age,
            "last_vehicle_state_latency_ms": self.last_vehicle_state_latency_ms,
            "network": self.outbound_network.statistics(),
            "endian_assumption": self.endian,
            "tlv_length_includes_header_assumption": self.length_includes_header,
        }
        msg = String()
        msg.data = json.dumps(state, separators=(",", ":"))
        self.diagnostic_pub.publish(msg)

    def on_timer(self):
        try:
            self.send_command()
        except OSError as exc:
            self.get_logger().error("UDP gateway error: %s" % exc)
        if self.tx_count % 4 == 0:
            self.publish_diagnostic()

    def on_network_timer(self):
        try:
            self.flush_outbound()
            self.receive_states()
        except OSError as exc:
            self.get_logger().error("UDP gateway network I/O error: %s" % exc)

    def destroy_node(self):
        self.socket.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DomainUdpGateway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
