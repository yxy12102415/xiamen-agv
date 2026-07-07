import socket
import struct

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import (
    ControlModeReport,
    GearCommand,
    GearReport,
    SteeringReport,
    VelocityReport,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, String, UInt8MultiArray


CAN_EFF_FLAG = 0x80000000


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def int16_scaled(value, scale, lower, upper):
    value = clamp(value, lower, upper)
    raw = int(round(value / scale))
    return int(clamp(raw, -32768, 32767))


def uint16_scaled(value, scale, lower, upper):
    value = clamp(value, lower, upper)
    raw = int(round(value / scale))
    return int(clamp(raw, 0, 65535))


def gear_report_from_chassis(value):
    return {
        0: GearReport.PARK,
        1: GearReport.DRIVE,
        2: GearReport.REVERSE,
        3: GearReport.NEUTRAL,
    }.get(int(value), GearReport.NONE)


def control_mode_from_chassis(value):
    return {
        0: ControlModeReport.MANUAL,
        1: ControlModeReport.AUTONOMOUS,
        2: ControlModeReport.AUTONOMOUS_STEER_ONLY,
        3: ControlModeReport.AUTONOMOUS_VELOCITY_ONLY,
    }.get(int(value), ControlModeReport.MANUAL)


def checksum(message_id, payload):
    total = (message_id & 0xFF) + ((message_id >> 8) & 0xFF) + sum(payload[:-1])
    return total & 0xFF


def framed_debug_message(message_id, payload):
    msg = UInt8MultiArray()
    msg.data = [message_id & 0xFF, (message_id >> 8) & 0xFF, len(payload), *payload]
    return msg


def stamp_now(node):
    return node.get_clock().now().to_msg()


class SocketCanTransport:
    def __init__(self, interface):
        self.interface = interface
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.socket.bind((interface,))
        self.socket.setblocking(False)

    def send(self, arbitration_id, payload):
        can_id = arbitration_id & 0x1FFFFFFF
        frame = struct.pack("=IB3x8s", can_id, len(payload), bytes(payload).ljust(8, b"\x00"))
        self.socket.send(frame)

    def receive(self, max_frames=32):
        frames = []
        for _ in range(max_frames):
            try:
                frame = self.socket.recv(16)
            except BlockingIOError:
                break
            can_id, can_dlc, data = struct.unpack("=IB3x8s", frame)
            frames.append((can_id & 0x1FFFFFFF, bytearray(data[:can_dlc])))
        return frames

    def close(self):
        self.socket.close()


class UdpTransport:
    def __init__(self, host, port, bind_host="0.0.0.0", bind_port=0):
        self.address = (host, int(port))
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if int(bind_port) > 0:
            self.socket.bind((bind_host, int(bind_port)))
        self.socket.setblocking(False)

    def send(self, arbitration_id, payload):
        packet = struct.pack("<HB", arbitration_id & 0xFFFF, len(payload)) + bytes(payload)
        self.socket.sendto(packet, self.address)

    def receive(self, max_frames=32):
        frames = []
        for _ in range(max_frames):
            try:
                packet, _ = self.socket.recvfrom(2048)
            except BlockingIOError:
                break
            if len(packet) < 3:
                continue
            arbitration_id, payload_len = struct.unpack_from("<HB", packet, 0)
            payload = bytearray(packet[3 : 3 + payload_len])
            frames.append((arbitration_id, payload))
        return frames

    def close(self):
        self.socket.close()


class RealVehicleInterfaceNode(Node):
    """Vehicle interface shaped like a real chassis bridge.

    The default configuration is intentionally non-actuating. Set dry_run=false,
    shadow_mode=false, and autonomous_enable=true only after the vendor protocol,
    limits, E-stop chain, and bench tests have been verified.
    """

    def __init__(self):
        super().__init__("real_vehicle_interface")

        self.declare_parameter("control_topic", "/control/command/control_cmd")
        self.declare_parameter("two_axis_steer_topic", "/agv/control/two_axis_steer_cmd")
        self.declare_parameter("gear_topic", "/control/command/gear_cmd")
        self.declare_parameter("tx_debug_topic", "/agv/chassis/tx_debug")
        self.declare_parameter("rx_debug_topic", "/agv/chassis/rx_debug")
        self.declare_parameter("state_topic", "/agv/vehicle_interface/state")
        self.declare_parameter("emergency_stop_topic", "/agv/safety/emergency_stop")
        self.declare_parameter("feedback_odom_topic", "/localization/kinematic_state")
        self.declare_parameter("update_rate", 50.0)
        self.declare_parameter("transport", "dry_run")
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("udp_host", "192.168.1.10")
        self.declare_parameter("udp_port", 5000)
        self.declare_parameter("udp_bind_host", "0.0.0.0")
        self.declare_parameter("udp_bind_port", 0)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("shadow_mode", True)
        self.declare_parameter("autonomous_enable", False)
        self.declare_parameter("publish_feedback_from_commands", False)
        self.declare_parameter("command_timeout", 0.1)
        self.declare_parameter("max_velocity", 2.0)
        self.declare_parameter("max_acceleration", 0.2)
        self.declare_parameter("max_brake_decel", 1.0)
        self.declare_parameter("front_steer_limit", 0.47822)
        self.declare_parameter("rear_steer_limit", 0.39444)
        self.declare_parameter("max_steer_rate", 0.261799)
        self.declare_parameter("steering_mode", "axle12")

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.shadow_mode = bool(self.get_parameter("shadow_mode").value)
        self.autonomous_enable = bool(self.get_parameter("autonomous_enable").value)
        self.publish_feedback_from_commands = bool(
            self.get_parameter("publish_feedback_from_commands").value
        )
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.max_brake_decel = float(self.get_parameter("max_brake_decel").value)
        self.front_steer_limit = float(self.get_parameter("front_steer_limit").value)
        self.rear_steer_limit = float(self.get_parameter("rear_steer_limit").value)

        self.control = Control()
        self.gear_command = GearCommand()
        self.gear_command.command = GearCommand.PARK
        self.front_steer = 0.0
        self.rear_steer = 0.0
        self.two_axis_velocity = None
        self.two_axis_acceleration = None
        self.last_command_time = None
        self.last_feedback_time = None
        self.manual_override = False
        self.emergency_stop_active = False
        self.fault_level = 0
        self.fault_codes = [0, 0, 0, 0]
        self.actual_velocity = 0.0
        self.actual_acceleration = 0.0
        self.actual_yaw_rate = 0.0
        self.brake_actual = 0.0
        self.alive_counter = 0
        self.transport = None

        self.create_subscription(
            Control, self.get_parameter("control_topic").value, self.on_control, 1
        )
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter("two_axis_steer_topic").value,
            self.on_two_axis_steer,
            1,
        )
        self.create_subscription(
            GearCommand, self.get_parameter("gear_topic").value, self.on_gear, 1
        )

        self.tx_debug_pub = self.create_publisher(
            UInt8MultiArray, self.get_parameter("tx_debug_topic").value, 10
        )
        self.rx_debug_pub = self.create_publisher(
            UInt8MultiArray, self.get_parameter("rx_debug_topic").value, 10
        )
        self.state_pub = self.create_publisher(String, self.get_parameter("state_topic").value, 10)
        self.emergency_stop_pub = self.create_publisher(
            Bool, self.get_parameter("emergency_stop_topic").value, 10
        )
        self.front_steer_pub = self.create_publisher(Float32, "/agv/status/front_steer", 10)
        self.rear_steer_pub = self.create_publisher(Float32, "/agv/status/rear_steer", 10)
        self.velocity_pub = self.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", 10
        )
        self.steering_pub = self.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", 10
        )
        self.gear_report_pub = self.create_publisher(GearReport, "/vehicle/status/gear_status", 10)
        self.control_mode_pub = self.create_publisher(
            ControlModeReport, "/vehicle/status/control_mode", 10
        )
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter("feedback_odom_topic").value, 10
        )

        self.open_transport()
        update_rate = float(self.get_parameter("update_rate").value)
        self.timer = self.create_timer(1.0 / max(1.0, update_rate), self.on_timer)

        self.get_logger().info(
            "Real vehicle interface started: transport=%s dry_run=%s shadow_mode=%s autonomous_enable=%s"
            % (
                self.get_parameter("transport").value,
                self.dry_run,
                self.shadow_mode,
                self.autonomous_enable,
            )
        )

    def open_transport(self):
        if self.dry_run:
            return
        transport = str(self.get_parameter("transport").value)
        if transport == "socketcan":
            interface = str(self.get_parameter("can_interface").value)
            self.transport = SocketCanTransport(interface)
            return
        if transport == "udp":
            self.transport = UdpTransport(
                str(self.get_parameter("udp_host").value),
                int(self.get_parameter("udp_port").value),
                str(self.get_parameter("udp_bind_host").value),
                int(self.get_parameter("udp_bind_port").value),
            )
            return
        raise RuntimeError("Unsupported real vehicle transport: %s" % transport)

    def on_control(self, msg):
        self.control = msg
        self.last_command_time = self.get_clock().now()

    def on_two_axis_steer(self, msg):
        if len(msg.data) >= 1:
            self.front_steer = clamp(
                float(msg.data[0]), -self.front_steer_limit, self.front_steer_limit
            )
        if len(msg.data) >= 2:
            self.rear_steer = clamp(
                float(msg.data[1]), -self.rear_steer_limit, self.rear_steer_limit
            )
        if len(msg.data) >= 3:
            self.two_axis_velocity = clamp(float(msg.data[2]), -self.max_velocity, self.max_velocity)
        if len(msg.data) >= 4:
            self.two_axis_acceleration = clamp(
                float(msg.data[3]), -self.max_brake_decel, self.max_acceleration
            )
        self.last_command_time = self.get_clock().now()

    def on_gear(self, msg):
        self.gear_command = msg

    def command_is_fresh(self):
        if self.last_command_time is None:
            return False
        age = (self.get_clock().now() - self.last_command_time).nanoseconds * 1e-9
        return age <= self.command_timeout

    def chassis_gear(self):
        if self.gear_command.command == GearCommand.DRIVE:
            return 1
        if self.gear_command.command == GearCommand.REVERSE:
            return 2
        if self.gear_command.command == GearCommand.NEUTRAL:
            return 3
        return 0

    def chassis_steering_mode(self):
        return {
            "front_only": 0,
            "crab": 1,
            "opposite_4ws": 2,
            "axle12": 3,
            "axle12_steer": 3,
        }.get(str(self.get_parameter("steering_mode").value), 3)

    def selected_velocity(self):
        if self.two_axis_velocity is not None:
            return self.two_axis_velocity
        return clamp(float(self.control.longitudinal.velocity), -self.max_velocity, self.max_velocity)

    def selected_acceleration(self):
        if self.two_axis_acceleration is not None:
            return self.two_axis_acceleration
        if self.control.longitudinal.is_defined_acceleration:
            return clamp(
                float(self.control.longitudinal.acceleration),
                -self.max_brake_decel,
                self.max_acceleration,
            )
        return 0.0

    def brake_command(self, acceleration):
        if acceleration >= 0.0:
            return 0.0
        return clamp(abs(acceleration) / max(1e-3, self.max_brake_decel), 0.0, 1.0)

    def encode_mode_frame(self, fresh):
        message_id = 0x180
        payload = bytearray(8)
        payload[0] = self.alive_counter & 0xFF
        payload[1] = 1 if (self.autonomous_enable and fresh and not self.shadow_mode) else 0
        payload[2] = self.chassis_gear() & 0xFF
        payload[3] = self.chassis_steering_mode() & 0xFF
        payload[7] = checksum(message_id, payload)
        return message_id, payload

    def encode_motion_frame(self, velocity, acceleration, fresh):
        message_id = 0x181
        if not fresh or self.shadow_mode:
            velocity = 0.0
            acceleration = -min(0.5, self.max_brake_decel)
        payload = bytearray(8)
        struct.pack_into(
            "<hhH",
            payload,
            0,
            int16_scaled(velocity, 0.001, -self.max_velocity, self.max_velocity),
            int16_scaled(acceleration, 0.001, -self.max_brake_decel, self.max_acceleration),
            uint16_scaled(self.brake_command(acceleration), 0.001, 0.0, 1.0),
        )
        payload[6] = self.alive_counter & 0xFF
        payload[7] = checksum(message_id, payload)
        return message_id, payload

    def encode_steer_frame(self, fresh):
        message_id = 0x182
        front = self.front_steer if fresh and not self.shadow_mode else 0.0
        rear = self.rear_steer if fresh and not self.shadow_mode else 0.0
        payload = bytearray(8)
        struct.pack_into(
            "<hhhh",
            payload,
            0,
            int16_scaled(front, 0.0001, -self.front_steer_limit, self.front_steer_limit),
            int16_scaled(rear, 0.0001, -self.rear_steer_limit, self.rear_steer_limit),
            0,
            0,
        )
        return message_id, payload

    def encode_safety_frame(self, fresh):
        message_id = 0x183
        requested_state = 1
        controlled_stop = 0
        if self.autonomous_enable and fresh and not self.shadow_mode:
            requested_state = 2
        if not fresh or self.shadow_mode:
            requested_state = 0
            controlled_stop = 1
        payload = bytearray(8)
        payload[0] = requested_state
        payload[1] = controlled_stop
        struct.pack_into("<H", payload, 3, uint16_scaled(self.max_velocity, 0.001, 0.0, 65.535))
        payload[6] = self.alive_counter & 0xFF
        payload[7] = checksum(message_id, payload)
        return message_id, payload

    def maybe_send(self, frames):
        for message_id, payload in frames:
            self.tx_debug_pub.publish(framed_debug_message(message_id, payload))
            if self.transport is not None and not self.shadow_mode:
                self.transport.send(message_id, payload)

    def feedback_age(self):
        if self.last_feedback_time is None:
            return None
        return (self.get_clock().now() - self.last_feedback_time).nanoseconds * 1e-9

    def checksum_ok(self, message_id, payload):
        return len(payload) >= 8 and checksum(message_id, payload) == payload[7]

    def poll_transport_feedback(self):
        if self.transport is None:
            return
        for message_id, payload in self.transport.receive():
            self.rx_debug_pub.publish(framed_debug_message(message_id, payload))
            self.decode_feedback_frame(message_id, payload)

    def decode_feedback_frame(self, message_id, payload):
        if len(payload) < 8:
            self.get_logger().warn(
                "Ignoring short chassis feedback frame 0x%03X len=%d" % (message_id, len(payload))
            )
            return
        if message_id == 0x280:
            self.decode_status_feedback(message_id, payload)
        elif message_id == 0x281:
            self.decode_motion_feedback(payload)
        elif message_id == 0x282:
            self.decode_steer_feedback(payload)
        elif message_id == 0x283:
            self.decode_fault_feedback(payload)

    def decode_status_feedback(self, message_id, payload):
        if not self.checksum_ok(message_id, payload):
            self.get_logger().warn("Ignoring AGV_FB_STATUS with invalid checksum")
            return
        stamp = stamp_now(self)
        flags = payload[0]
        self.manual_override = bool(flags & 0x02)
        self.emergency_stop_active = bool(flags & 0x04)
        self.fault_level = int(payload[3])
        self.last_feedback_time = self.get_clock().now()

        gear = GearReport()
        gear.stamp = stamp
        gear.report = gear_report_from_chassis(payload[1])
        self.gear_report_pub.publish(gear)

        control_mode = ControlModeReport()
        control_mode.stamp = stamp
        if self.manual_override or self.emergency_stop_active:
            control_mode.mode = ControlModeReport.MANUAL
        else:
            control_mode.mode = control_mode_from_chassis(payload[2])
        self.control_mode_pub.publish(control_mode)

        emergency_stop = Bool()
        emergency_stop.data = self.emergency_stop_active or self.fault_level >= 3
        self.emergency_stop_pub.publish(emergency_stop)

    def decode_motion_feedback(self, payload):
        stamp = stamp_now(self)
        velocity_raw, acceleration_raw, yaw_rate_raw, brake_raw = struct.unpack_from(
            "<hhhH", payload, 0
        )
        self.actual_velocity = velocity_raw * 0.001
        self.actual_acceleration = acceleration_raw * 0.001
        self.actual_yaw_rate = yaw_rate_raw * 0.0001
        self.brake_actual = brake_raw * 0.001
        self.last_feedback_time = self.get_clock().now()

        velocity_report = VelocityReport()
        velocity_report.header.stamp = stamp
        velocity_report.header.frame_id = "base_link"
        velocity_report.longitudinal_velocity = float(self.actual_velocity)
        velocity_report.lateral_velocity = 0.0
        velocity_report.heading_rate = float(self.actual_yaw_rate)
        self.velocity_pub.publish(velocity_report)

    def decode_steer_feedback(self, payload):
        stamp = stamp_now(self)
        axle1_raw, axle2_raw, _, _ = struct.unpack_from("<hhhh", payload, 0)
        self.front_steer = clamp(axle1_raw * 0.0001, -self.front_steer_limit, self.front_steer_limit)
        self.rear_steer = clamp(axle2_raw * 0.0001, -self.rear_steer_limit, self.rear_steer_limit)
        self.last_feedback_time = self.get_clock().now()

        front = Float32()
        front.data = float(self.front_steer)
        self.front_steer_pub.publish(front)

        rear = Float32()
        rear.data = float(self.rear_steer)
        self.rear_steer_pub.publish(rear)

        steering = SteeringReport()
        steering.stamp = stamp
        steering.steering_tire_angle = float(self.front_steer)
        self.steering_pub.publish(steering)

    def decode_fault_feedback(self, payload):
        self.fault_codes = list(struct.unpack_from("<HHHH", payload, 0))
        self.last_feedback_time = self.get_clock().now()

    def publish_optional_feedback(self, velocity):
        if not self.publish_feedback_from_commands:
            return
        stamp = stamp_now(self)

        front = Float32()
        front.data = float(self.front_steer)
        self.front_steer_pub.publish(front)

        rear = Float32()
        rear.data = float(self.rear_steer)
        self.rear_steer_pub.publish(rear)

        velocity_report = VelocityReport()
        velocity_report.header.stamp = stamp
        velocity_report.header.frame_id = "base_link"
        velocity_report.longitudinal_velocity = float(velocity)
        velocity_report.lateral_velocity = 0.0
        velocity_report.heading_rate = 0.0
        self.velocity_pub.publish(velocity_report)

        steering = SteeringReport()
        steering.stamp = stamp
        steering.steering_tire_angle = float(self.front_steer)
        self.steering_pub.publish(steering)

        gear = GearReport()
        gear.stamp = stamp
        gear.report = self.gear_command.command
        self.gear_report_pub.publish(gear)

        control_mode = ControlModeReport()
        control_mode.stamp = stamp
        control_mode.mode = (
            ControlModeReport.AUTONOMOUS
            if self.autonomous_enable and not self.shadow_mode
            else ControlModeReport.MANUAL
        )
        self.control_mode_pub.publish(control_mode)

    def publish_state(self, fresh, velocity, acceleration):
        feedback_age = self.feedback_age()
        feedback_age_text = "none" if feedback_age is None else "%.3f" % feedback_age
        msg = String()
        msg.data = (
            "transport=%s dry_run=%s shadow_mode=%s autonomous_enable=%s command_fresh=%s "
            "feedback_age=%s manual_override=%s estop=%s fault_level=%d faults=%s "
            "velocity=%.3f accel=%.3f actual_velocity=%.3f actual_yaw_rate=%.4f "
            "front=%.4f rear=%.4f"
            % (
                self.get_parameter("transport").value,
                self.dry_run,
                self.shadow_mode,
                self.autonomous_enable,
                fresh,
                feedback_age_text,
                self.manual_override,
                self.emergency_stop_active,
                self.fault_level,
                ",".join(str(code) for code in self.fault_codes),
                velocity,
                acceleration,
                self.actual_velocity,
                self.actual_yaw_rate,
                self.front_steer,
                self.rear_steer,
            )
        )
        self.state_pub.publish(msg)

    def on_timer(self):
        self.poll_transport_feedback()
        fresh = self.command_is_fresh()
        velocity = self.selected_velocity()
        acceleration = self.selected_acceleration()
        frames = (
            self.encode_mode_frame(fresh),
            self.encode_motion_frame(velocity, acceleration, fresh),
            self.encode_steer_frame(fresh),
            self.encode_safety_frame(fresh),
        )
        self.maybe_send(frames)
        self.publish_optional_feedback(velocity if fresh else 0.0)
        self.publish_state(fresh, velocity, acceleration)
        self.alive_counter = (self.alive_counter + 1) & 0xFF

    def destroy_node(self):
        if self.transport is not None:
            self.transport.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealVehicleInterfaceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
