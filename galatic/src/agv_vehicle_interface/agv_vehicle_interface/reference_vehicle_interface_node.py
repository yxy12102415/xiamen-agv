import struct

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import ControlModeReport, GearCommand, GearReport, SteeringReport, VelocityReport
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String, UInt8MultiArray


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def int16_scaled(value, scale, lower, upper):
    value = clamp(value, lower, upper)
    raw = int(round(value / scale))
    return clamp(raw, -32768, 32767)


def uint16_scaled(value, scale, lower, upper):
    value = clamp(value, lower, upper)
    raw = int(round(value / scale))
    return clamp(raw, 0, 65535)


def checksum(message_id, payload):
    total = (message_id & 0xFF) + ((message_id >> 8) & 0xFF) + sum(payload[:-1])
    return total & 0xFF


def frame_message(message_id, payload):
    msg = UInt8MultiArray()
    msg.data = [
        message_id & 0xFF,
        (message_id >> 8) & 0xFF,
        len(payload),
        *payload,
    ]
    return msg


class ReferenceVehicleInterfaceNode(Node):
    """Reference protocol bridge.

    This node intentionally does not touch real CAN hardware. It encodes the
    controller outputs into byte arrays matching docs/reference_agv_chassis.dbc
    and can mirror commands back as mock chassis feedback.
    """

    def __init__(self):
        super().__init__("reference_vehicle_interface")

        self.declare_parameter("control_topic", "/control/command/control_cmd")
        self.declare_parameter("two_axis_steer_topic", "/agv/control/two_axis_steer_cmd")
        self.declare_parameter("gear_topic", "/control/command/gear_cmd")
        self.declare_parameter("tx_frame_topic", "/agv/reference_can/tx")
        self.declare_parameter("state_topic", "/agv/vehicle_interface/state")
        self.declare_parameter("update_rate", 50.0)
        self.declare_parameter("mock_feedback", True)
        self.declare_parameter("shadow_mode", True)
        self.declare_parameter("autonomous_enable", False)
        self.declare_parameter("steering_mode", "axle12")
        self.declare_parameter("command_timeout", 0.1)
        self.declare_parameter("max_velocity", 5.0)
        self.declare_parameter("max_acceleration", 0.35)
        self.declare_parameter("max_brake_decel", 4.0)
        self.declare_parameter("max_steer", 1.745329)

        self.mock_feedback = bool(self.get_parameter("mock_feedback").value)
        self.shadow_mode = bool(self.get_parameter("shadow_mode").value)
        self.autonomous_enable = bool(self.get_parameter("autonomous_enable").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.max_brake_decel = float(self.get_parameter("max_brake_decel").value)
        self.max_steer = float(self.get_parameter("max_steer").value)

        self.control = Control()
        self.gear_command = GearCommand()
        self.gear_command.command = GearCommand.PARK
        self.axle1_steer = 0.0
        self.axle2_steer = 0.0
        self.axle3_steer = 0.0
        self.axle4_steer = 0.0
        self.two_axis_velocity = None
        self.two_axis_acceleration = None
        self.last_command_time = None
        self.alive_counter = 0

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

        self.tx_pub = self.create_publisher(
            UInt8MultiArray, self.get_parameter("tx_frame_topic").value, 10
        )
        self.state_pub = self.create_publisher(String, self.get_parameter("state_topic").value, 10)
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

        update_rate = float(self.get_parameter("update_rate").value)
        self.timer = self.create_timer(1.0 / max(1.0, update_rate), self.on_timer)

        mode = "shadow" if self.shadow_mode else "active"
        self.get_logger().info(
            "Reference vehicle interface started in %s mode. Encoded frames publish to %s"
            % (mode, self.get_parameter("tx_frame_topic").value)
        )

    def on_control(self, msg):
        self.control = msg
        self.last_command_time = self.get_clock().now()

    def on_two_axis_steer(self, msg):
        if len(msg.data) >= 1:
            self.axle1_steer = clamp(float(msg.data[0]), -self.max_steer, self.max_steer)
        if len(msg.data) >= 2:
            self.axle2_steer = clamp(float(msg.data[1]), -self.max_steer, self.max_steer)
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
        return (self.get_clock().now() - self.last_command_time).nanoseconds * 1e-9 <= self.command_timeout

    def reference_gear(self):
        if self.gear_command.command == GearCommand.DRIVE:
            return 1
        if self.gear_command.command == GearCommand.REVERSE:
            return 2
        if self.gear_command.command == GearCommand.NEUTRAL:
            return 3
        return 0

    def reference_steering_mode(self):
        value = str(self.get_parameter("steering_mode").value)
        return {
            "front_only": 0,
            "crab": 1,
            "opposite_4ws": 2,
            "axle12": 3,
            "axle12_steer": 3,
        }.get(value, 3)

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
        payload[2] = self.reference_gear() & 0xFF
        payload[3] = self.reference_steering_mode() & 0xFF
        payload[7] = checksum(message_id, payload)
        return message_id, payload

    def encode_motion_frame(self, velocity, acceleration, fresh):
        message_id = 0x181
        if not fresh:
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

    def encode_steer_frame(self):
        message_id = 0x182
        payload = bytearray(8)
        struct.pack_into(
            "<hhhh",
            payload,
            0,
            int16_scaled(self.axle1_steer, 0.0001, -self.max_steer, self.max_steer),
            int16_scaled(self.axle2_steer, 0.0001, -self.max_steer, self.max_steer),
            int16_scaled(self.axle3_steer, 0.0001, -self.max_steer, self.max_steer),
            int16_scaled(self.axle4_steer, 0.0001, -self.max_steer, self.max_steer),
        )
        return message_id, payload

    def encode_safety_frame(self, fresh):
        message_id = 0x183
        requested_state = 1
        controlled_stop = 0
        emergency_stop = 0
        if self.autonomous_enable and fresh and not self.shadow_mode:
            requested_state = 2
        if not fresh:
            requested_state = 0
            controlled_stop = 1
        payload = bytearray(8)
        payload[0] = requested_state
        payload[1] = controlled_stop | (emergency_stop << 1)
        struct.pack_into("<H", payload, 3, uint16_scaled(self.max_velocity, 0.001, 0.0, 65.535))
        payload[6] = self.alive_counter & 0xFF
        payload[7] = checksum(message_id, payload)
        return message_id, payload

    def publish_mock_feedback(self, velocity, acceleration):
        stamp = self.get_clock().now().to_msg()

        front = Float32()
        front.data = float(self.axle1_steer)
        self.front_steer_pub.publish(front)

        rear = Float32()
        rear.data = float(self.axle2_steer)
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
        steering.steering_tire_angle = float(self.axle1_steer)
        self.steering_pub.publish(steering)

        gear = GearReport()
        gear.stamp = stamp
        gear.report = self.gear_command.command
        self.gear_report_pub.publish(gear)

        control_mode = ControlModeReport()
        control_mode.stamp = stamp
        control_mode.mode = ControlModeReport.AUTONOMOUS if self.autonomous_enable else ControlModeReport.MANUAL
        self.control_mode_pub.publish(control_mode)

    def on_timer(self):
        fresh = self.command_is_fresh()
        velocity = self.selected_velocity()
        acceleration = self.selected_acceleration()

        for message_id, payload in (
            self.encode_mode_frame(fresh),
            self.encode_motion_frame(velocity, acceleration, fresh),
            self.encode_steer_frame(),
            self.encode_safety_frame(fresh),
        ):
            self.tx_pub.publish(frame_message(message_id, payload))

        if self.mock_feedback:
            self.publish_mock_feedback(velocity if fresh else 0.0, acceleration if fresh else 0.0)

        state = String()
        state.data = (
            "shadow_mode=%s autonomous_enable=%s command_fresh=%s velocity=%.3f accel=%.3f "
            "axle1=%.4f axle2=%.4f"
            % (
                self.shadow_mode,
                self.autonomous_enable,
                fresh,
                velocity,
                acceleration,
                self.axle1_steer,
                self.axle2_steer,
            )
        )
        self.state_pub.publish(state)
        self.alive_counter = (self.alive_counter + 1) & 0xFF


def main(args=None):
    rclpy.init(args=args)
    node = ReferenceVehicleInterfaceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
