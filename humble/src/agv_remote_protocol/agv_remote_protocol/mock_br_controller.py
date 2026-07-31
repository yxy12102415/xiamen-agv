import json
import math
import socket
import time

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand, GearReport
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from .protocol import ITEM_REMOTE_COMMAND
from .protocol import ITEM_REMOTE_DRIVING
from .protocol import ITEM_VEHICLE_STATE
from .protocol import MSG_DOMAIN_COMMAND
from .protocol import MSG_VEHICLE_STATE
from .protocol import ProtocolError
from .protocol import pack_packet
from .protocol import pack_vehicle_state
from .protocol import unpack_packet
from .protocol import unpack_remote_command
from .protocol import unpack_remote_driving
from .network_impairment import NetworkImpairmentQueue


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class MockBrController(Node):
    """Vehicle-side UDP server used until the real B&R controller is available."""

    def __init__(self):
        super().__init__("mock_br_controller")
        self.declare_parameter("local_host", "127.0.0.1")
        self.declare_parameter("local_port", 46001)
        self.declare_parameter("domain_host", "127.0.0.1")
        self.declare_parameter("domain_port", 46000)
        self.declare_parameter("endian", "little")
        self.declare_parameter("tlv_length_includes_header", False)
        self.declare_parameter("update_rate", 40.0)
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("network_delay_ms", 50.0)
        self.declare_parameter("network_jitter_ms", 0.0)
        self.declare_parameter("network_loss_percent", 0.0)
        self.declare_parameter("network_seed", 7000)
        self.declare_parameter("network_delivery_rate", 200.0)
        self.declare_parameter("max_velocity", 3.0)
        self.declare_parameter("max_reverse_velocity", 0.3)
        self.declare_parameter("front_steer_limit", 0.47822)
        self.declare_parameter("rear_steer_limit", 0.39444)
        self.declare_parameter("max_drive_accel", 0.35)
        self.declare_parameter("max_brake_decel", 1.0)
        self.declare_parameter("battery_level", 100.0)

        self.local_host = str(self.get_parameter("local_host").value)
        self.local_port = int(self.get_parameter("local_port").value)
        self.domain_address = (
            str(self.get_parameter("domain_host").value),
            int(self.get_parameter("domain_port").value),
        )
        self.endian = str(self.get_parameter("endian").value)
        self.length_includes_header = bool(
            self.get_parameter("tlv_length_includes_header").value
        )
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_reverse_velocity = float(
            self.get_parameter("max_reverse_velocity").value
        )
        self.front_steer_limit = float(self.get_parameter("front_steer_limit").value)
        self.rear_steer_limit = float(self.get_parameter("rear_steer_limit").value)
        self.max_drive_accel = float(self.get_parameter("max_drive_accel").value)
        self.max_brake_decel = float(self.get_parameter("max_brake_decel").value)
        self.battery_level = float(self.get_parameter("battery_level").value)

        self.command = None
        self.driving = None
        self.last_command_time = None
        self.last_sender = None
        self.odom = None
        self.odom_time = None
        self.front_steer = 0.0
        self.gear_report = GearReport.DRIVE
        self.state_message_id = 0
        self.rx_count = 0
        self.rx_error_count = 0
        self.last_command_latency_ms = None
        self.outbound_network = NetworkImpairmentQueue(
            delay_ms=self.get_parameter("network_delay_ms").value,
            jitter_ms=self.get_parameter("network_jitter_ms").value,
            loss_percent=self.get_parameter("network_loss_percent").value,
            seed=self.get_parameter("network_seed").value,
        )

        self.create_subscription(Odometry, "/localization/kinematic_state", self.on_odom, 1)
        self.create_subscription(Float32, "/agv/status/front_steer", self.on_front_steer, 1)
        self.create_subscription(GearReport, "/vehicle/status/gear_status", self.on_gear, 1)

        self.control_pub = self.create_publisher(Control, "/remote/control/control_cmd", 1)
        self.two_axis_pub = self.create_publisher(
            Float32MultiArray, "/remote/control/two_axis_steer_cmd", 1
        )
        self.gear_pub = self.create_publisher(GearCommand, "/remote/control/gear_cmd", 1)
        self.deadman_pub = self.create_publisher(Bool, "/remote/control/deadman", 1)
        self.estop_pub = self.create_publisher(
            Bool, "/remote/control/emergency_stop", 1
        )
        self.state_pub = self.create_publisher(String, "/remote/protocol/mock_br_state", 10)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.local_host, self.local_port))
        self.socket.setblocking(False)

        update_rate = float(self.get_parameter("update_rate").value)
        self.timer = self.create_timer(1.0 / max(1.0, update_rate), self.on_timer)
        delivery_rate = float(self.get_parameter("network_delivery_rate").value)
        self.network_timer = self.create_timer(
            1.0 / max(update_rate, delivery_rate), self.on_network_timer
        )
        self.get_logger().info(
            "Mock B&R UDP controller listening on %s:%d at %.1f Hz with %.1f ms "
            "one-way state delay"
            % (
                self.local_host,
                self.local_port,
                update_rate,
                self.outbound_network.delay_ms,
            )
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def on_odom(self, msg):
        self.odom = msg
        self.odom_time = self.now_seconds()

    def on_front_steer(self, msg):
        self.front_steer = float(msg.data)

    def on_gear(self, msg):
        self.gear_report = int(msg.report)

    def command_fresh(self):
        return (
            self.last_command_time is not None
            and self.now_seconds() - self.last_command_time <= self.command_timeout
        )

    def receive_commands(self):
        while True:
            try:
                data, address = self.socket.recvfrom(65535)
            except BlockingIOError:
                return
            try:
                packet = unpack_packet(data, self.endian, self.length_includes_header)
                if packet["msg_type"] != MSG_DOMAIN_COMMAND:
                    raise ProtocolError("unexpected msg_type 0x%04X" % packet["msg_type"])
                command_value = next(
                    value for item_type, value in packet["items"]
                    if item_type == ITEM_REMOTE_COMMAND
                )
                driving_value = next(
                    value for item_type, value in packet["items"]
                    if item_type == ITEM_REMOTE_DRIVING
                )
                self.command = unpack_remote_command(command_value, self.endian)
                self.driving = unpack_remote_driving(driving_value, self.endian)
                self.last_command_time = self.now_seconds()
                packet_time = packet["sys_time_s"] + packet["sys_time_ms"] * 0.001
                self.last_command_latency_ms = max(
                    0.0, (time.time() - packet_time) * 1000.0
                )
                self.last_sender = address
                self.rx_count += 1
            except (ProtocolError, StopIteration) as exc:
                self.rx_error_count += 1
                self.get_logger().warning("Rejected domain-command datagram: %s" % exc)

    def protocol_gear_to_ros(self, value):
        return {
            1: GearCommand.DRIVE,
            2: GearCommand.REVERSE,
            3: GearCommand.DRIVE,
            4: GearCommand.REVERSE,
            5: GearCommand.PARK,
        }.get(int(value), GearCommand.PARK)

    def publish_control(self):
        fresh = self.command_fresh() and self.command is not None and self.driving is not None
        enabled = (
            fresh
            and self.command["control_mode"] == 5
            and bool(self.command["start_enable"])
            and not bool(self.command["estop"])
        )
        speed_scale = clamp(float(self.driving["speed_scale"]), -100.0, 100.0) if fresh else 0.0
        velocity_limit = self.max_velocity if speed_scale >= 0.0 else self.max_reverse_velocity
        velocity = speed_scale * 0.01 * velocity_limit if enabled else 0.0
        steering_scale = (
            clamp(float(self.driving["steering_scale"]), -100.0, 100.0) if enabled else 0.0
        )
        front_steer = steering_scale * 0.01 * self.front_steer_limit
        rear_steer = clamp(
            front_steer * self.rear_steer_limit / max(1.0e-6, self.front_steer_limit),
            -self.rear_steer_limit,
            self.rear_steer_limit,
        )
        brake_ratio = (
            clamp(float(self.driving["brake_scale"]), 0.0, 100.0) * 0.01
            if fresh else 1.0
        )
        if brake_ratio > 1.0e-3:
            acceleration = -self.max_brake_decel if velocity >= 0.0 else self.max_drive_accel
            acceleration *= brake_ratio
        elif velocity > 0.0:
            acceleration = self.max_drive_accel
        elif velocity < 0.0:
            acceleration = -self.max_drive_accel
        else:
            acceleration = 0.0

        stamp = self.get_clock().now().to_msg()
        control = Control()
        control.stamp = stamp
        control.longitudinal.stamp = stamp
        control.longitudinal.velocity = float(velocity)
        control.longitudinal.acceleration = float(acceleration)
        control.longitudinal.is_defined_acceleration = True
        control.lateral.stamp = stamp
        control.lateral.steering_tire_angle = float(front_steer)
        control.lateral.steering_tire_rotation_rate = 0.261799
        control.lateral.is_defined_steering_tire_rotation_rate = True
        self.control_pub.publish(control)

        two_axis = Float32MultiArray()
        two_axis.data = [front_steer, rear_steer, velocity, acceleration]
        self.two_axis_pub.publish(two_axis)

        gear = GearCommand()
        gear.stamp = stamp
        gear.command = self.protocol_gear_to_ros(
            self.command["gear_command"] if fresh else 5
        )
        self.gear_pub.publish(gear)

        deadman = Bool()
        deadman.data = enabled
        self.deadman_pub.publish(deadman)
        estop = Bool()
        estop.data = bool(self.command["estop"]) if fresh else False
        self.estop_pub.publish(estop)

    def feedback_gear(self):
        return {
            GearReport.DRIVE: 1,
            GearReport.REVERSE: 2,
            GearReport.PARK: 5,
            GearReport.NEUTRAL: 5,
        }.get(self.gear_report, 5)

    def send_state(self):
        odom_fresh = self.odom_time is not None and self.now_seconds() - self.odom_time <= 0.5
        x = y = heading = velocity = 0.0
        if self.odom is not None:
            x = float(self.odom.pose.pose.position.x)
            y = float(self.odom.pose.pose.position.y)
            heading = math.degrees(yaw_from_quaternion(self.odom.pose.pose.orientation))
            velocity = float(self.odom.twist.twist.linear.x)
        state = {
            "igv_id": int(self.command["igv_id"]) if self.command else 1,
            "control_mode": int(self.command["control_mode"]) if self.command_fresh() else 7,
            "gear_state": self.feedback_gear(),
            "headlights_state": 0,
            "task_running_state": 0,
            "position_valid": int(odom_fresh),
            "heading_valid": int(odom_fresh),
            "obstacle_detect_state": 0,
            "start_ready": 1,
            "battery_level": self.battery_level,
            "speed_feedback": velocity,
            "steering_feedback": math.degrees(self.front_steer),
            "pose_x": x,
            "pose_y": y,
            "pose_heading": heading,
            "polygons": [],
            "faults": [],
        }
        packet = pack_packet(
            MSG_VEHICLE_STATE,
            self.state_message_id,
            [(ITEM_VEHICLE_STATE, pack_vehicle_state(state, self.endian))],
            self.endian,
            self.length_includes_header,
        )
        address = self.last_sender if self.last_sender is not None else self.domain_address
        self.outbound_network.enqueue((packet, address))
        self.state_message_id = (self.state_message_id + 1) & 0xFFFF

    def flush_outbound(self):
        for _release_at, (packet, address) in self.outbound_network.pop_ready():
            self.socket.sendto(packet, address)

    def publish_diagnostic(self):
        state = {
            "command_fresh": self.command_fresh(),
            "rx_count": self.rx_count,
            "rx_error_count": self.rx_error_count,
            "last_sender": self.last_sender,
            "last_command_latency_ms": self.last_command_latency_ms,
            "network": self.outbound_network.statistics(),
            "mock_controller": True,
        }
        msg = String()
        msg.data = json.dumps(state, separators=(",", ":"))
        self.state_pub.publish(msg)

    def on_timer(self):
        try:
            self.publish_control()
            self.send_state()
        except OSError as exc:
            self.get_logger().error("Mock B&R UDP error: %s" % exc)
        if self.state_message_id % 4 == 0:
            self.publish_diagnostic()

    def on_network_timer(self):
        try:
            self.receive_commands()
            self.flush_outbound()
        except OSError as exc:
            self.get_logger().error("Mock B&R UDP network I/O error: %s" % exc)

    def destroy_node(self):
        self.socket.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MockBrController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
