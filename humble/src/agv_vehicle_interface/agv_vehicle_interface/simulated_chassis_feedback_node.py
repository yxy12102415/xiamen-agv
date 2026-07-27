import json
import math

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import ControlModeReport, GearCommand, GearReport
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class SimulatedChassisFeedbackNode(Node):
    """Publish real-chassis-like feedback from the current simulator state."""

    def __init__(self):
        super().__init__("simulated_chassis_feedback")

        self.declare_parameter("odom_topic", "/localization/kinematic_state")
        self.declare_parameter("control_topic", "/control/command/control_cmd")
        self.declare_parameter("front_steer_topic", "/agv/status/front_steer")
        self.declare_parameter("rear_steer_topic", "/agv/status/rear_steer")
        self.declare_parameter("gear_status_topic", "/vehicle/status/gear_status")
        self.declare_parameter("control_mode_topic", "/vehicle/status/control_mode")
        self.declare_parameter("emergency_stop_topic", "/agv/safety/emergency_stop")
        self.declare_parameter("update_rate", 20.0)
        self.declare_parameter("feedback_timeout", 0.5)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("wheel_radius", 0.66)
        self.declare_parameter("max_brake_pressure_bar", 160.0)
        self.declare_parameter("brake_pressure_deadzone_bar", 3.0)
        self.declare_parameter("brake_pressure_curve_exponent", 1.05)
        self.declare_parameter("max_brake_decel", 4.0)
        self.declare_parameter("max_drive_accel", 0.35)

        self.feedback_timeout = float(self.get_parameter("feedback_timeout").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.max_brake_pressure_bar = float(self.get_parameter("max_brake_pressure_bar").value)
        self.brake_pressure_deadzone_bar = float(
            self.get_parameter("brake_pressure_deadzone_bar").value
        )
        self.brake_pressure_curve_exponent = float(
            self.get_parameter("brake_pressure_curve_exponent").value
        )
        self.max_brake_decel = float(self.get_parameter("max_brake_decel").value)
        self.max_drive_accel = float(self.get_parameter("max_drive_accel").value)

        self.actual_velocity = 0.0
        self.actual_acceleration = 0.0
        self.actual_yaw_rate = 0.0
        self.front_steer = 0.0
        self.rear_steer = 0.0
        self.command_accel = 0.0
        self.gear_state = "UNKNOWN"
        self.control_mode_state = "UNKNOWN"
        self.emergency_stop = False
        self.last_odom_time = None
        self.last_command_time = None
        self.last_velocity = None
        self.last_velocity_time = None

        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.on_odom, 10
        )
        self.create_subscription(
            Control, self.get_parameter("control_topic").value, self.on_control, 10
        )
        self.create_subscription(
            Float32, self.get_parameter("front_steer_topic").value, self.on_front_steer, 10
        )
        self.create_subscription(
            Float32, self.get_parameter("rear_steer_topic").value, self.on_rear_steer, 10
        )
        self.create_subscription(
            GearReport, self.get_parameter("gear_status_topic").value, self.on_gear, 10
        )
        self.create_subscription(
            ControlModeReport,
            self.get_parameter("control_mode_topic").value,
            self.on_control_mode,
            10,
        )
        self.create_subscription(
            Bool, self.get_parameter("emergency_stop_topic").value, self.on_emergency_stop, 10
        )

        self.pub_velocity = self.create_publisher(Float32, "/agv/chassis/actual_velocity", 10)
        self.pub_acceleration = self.create_publisher(
            Float32, "/agv/chassis/actual_acceleration", 10
        )
        self.pub_front_steer = self.create_publisher(
            Float32, "/agv/chassis/front_steer_actual", 10
        )
        self.pub_rear_steer = self.create_publisher(
            Float32, "/agv/chassis/rear_steer_actual", 10
        )
        self.pub_wheel_speeds = self.create_publisher(
            Float32MultiArray, "/agv/chassis/wheel_speeds", 10
        )
        self.pub_brake_pressure = self.create_publisher(
            Float32, "/agv/chassis/brake_pressure_bar", 10
        )
        self.pub_drive_motor = self.create_publisher(
            String, "/agv/chassis/drive_motor_state", 10
        )
        self.pub_steering_motor = self.create_publisher(
            String, "/agv/chassis/steering_motor_state", 10
        )
        self.pub_gear = self.create_publisher(String, "/agv/chassis/gear_state", 10)
        self.pub_control_mode = self.create_publisher(
            String, "/agv/chassis/control_mode_state", 10
        )
        self.pub_emergency_stop = self.create_publisher(
            Bool, "/agv/chassis/emergency_stop", 10
        )
        self.pub_fault = self.create_publisher(String, "/agv/chassis/fault_state", 10)
        self.pub_communication = self.create_publisher(
            String, "/agv/chassis/communication_state", 10
        )
        self.pub_summary = self.create_publisher(String, "/agv/chassis/state_summary", 10)

        update_rate = float(self.get_parameter("update_rate").value)
        self.timer = self.create_timer(1.0 / max(1.0, update_rate), self.publish_feedback)
        self.get_logger().info("Simulated chassis feedback publisher started.")

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        now_s = self.now_seconds()
        velocity = float(msg.twist.twist.linear.x)
        self.actual_velocity = velocity
        self.actual_yaw_rate = float(msg.twist.twist.angular.z)
        if self.last_velocity is not None and self.last_velocity_time is not None:
            dt = max(1e-3, now_s - self.last_velocity_time)
            self.actual_acceleration = (velocity - self.last_velocity) / dt
        self.last_velocity = velocity
        self.last_velocity_time = now_s
        self.last_odom_time = now_s

    def on_control(self, msg):
        self.command_accel = float(msg.longitudinal.acceleration)
        self.last_command_time = self.now_seconds()

    def on_front_steer(self, msg):
        self.front_steer = float(msg.data)

    def on_rear_steer(self, msg):
        self.rear_steer = float(msg.data)

    def on_gear(self, msg):
        self.gear_state = {
            GearReport.PARK: "PARK",
            GearReport.REVERSE: "REVERSE",
            GearReport.NEUTRAL: "NEUTRAL",
            GearReport.DRIVE: "DRIVE",
            GearReport.LOW: "LOW",
        }.get(int(msg.report), "UNKNOWN")

    def on_control_mode(self, msg):
        self.control_mode_state = {
            ControlModeReport.NO_COMMAND: "NO_COMMAND",
            ControlModeReport.AUTONOMOUS: "AUTONOMOUS",
            ControlModeReport.AUTONOMOUS_STEER_ONLY: "AUTONOMOUS_STEER_ONLY",
            ControlModeReport.AUTONOMOUS_VELOCITY_ONLY: "AUTONOMOUS_VELOCITY_ONLY",
            ControlModeReport.MANUAL: "MANUAL",
        }.get(int(msg.mode), "UNKNOWN")

    def on_emergency_stop(self, msg):
        self.emergency_stop = bool(msg.data)

    def message_float(self, value):
        msg = Float32()
        msg.data = float(value)
        return msg

    def publish_feedback(self):
        now_s = self.now_seconds()
        odom_age = math.inf if self.last_odom_time is None else now_s - self.last_odom_time
        command_age = (
            math.inf if self.last_command_time is None else now_s - self.last_command_time
        )
        communication_state = "OK" if odom_age <= self.feedback_timeout else "STALE_FEEDBACK"
        command_state = "OK" if command_age <= self.command_timeout else "STALE_COMMAND"
        fault_state = "OK"
        if self.emergency_stop:
            fault_state = "EMERGENCY_STOP"
        elif communication_state != "OK":
            fault_state = "FEEDBACK_TIMEOUT"

        brake_ratio = clamp(-self.command_accel / max(1e-3, self.max_brake_decel), 0.0, 1.0)
        brake_pressure_bar = 0.0
        if brake_ratio > 0.0:
            usable_pressure = max(
                0.0, self.max_brake_pressure_bar - self.brake_pressure_deadzone_bar
            )
            brake_pressure_bar = self.brake_pressure_deadzone_bar + usable_pressure * (
                brake_ratio ** max(0.1, self.brake_pressure_curve_exponent)
            )
        wheel_angular_speed = self.actual_velocity / max(1e-3, self.wheel_radius)
        drive_motor_state = "BRAKING" if brake_ratio > 0.02 else (
            "DRIVING" if self.command_accel > 0.02 else "IDLE"
        )
        steering_motor_state = "ACTIVE" if abs(self.front_steer) + abs(self.rear_steer) > 1e-3 else "IDLE"

        self.pub_velocity.publish(self.message_float(self.actual_velocity))
        self.pub_acceleration.publish(self.message_float(self.actual_acceleration))
        self.pub_front_steer.publish(self.message_float(self.front_steer))
        self.pub_rear_steer.publish(self.message_float(self.rear_steer))
        self.pub_brake_pressure.publish(self.message_float(brake_pressure_bar))

        wheel_speeds = Float32MultiArray()
        wheel_speeds.data = [float(wheel_angular_speed)] * 8
        self.pub_wheel_speeds.publish(wheel_speeds)

        for publisher, value in (
            (self.pub_drive_motor, drive_motor_state),
            (self.pub_steering_motor, steering_motor_state),
            (self.pub_gear, self.gear_state),
            (self.pub_control_mode, self.control_mode_state),
            (self.pub_fault, fault_state),
            (self.pub_communication, f"{communication_state};{command_state}"),
        ):
            msg = String()
            msg.data = value
            publisher.publish(msg)

        estop = Bool()
        estop.data = self.emergency_stop
        self.pub_emergency_stop.publish(estop)

        stamp = self.get_clock().now().to_msg()
        summary = {
            "stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
            "timestamp_source": "ros_node_clock",
            "actual_velocity_mps": self.actual_velocity,
            "actual_acceleration_mps2": self.actual_acceleration,
            "yaw_rate_radps": self.actual_yaw_rate,
            "front_steer_rad": self.front_steer,
            "rear_steer_rad": self.rear_steer,
            "wheel_speeds_radps": list(wheel_speeds.data),
            "brake_pressure_bar": brake_pressure_bar,
            "drive_motor_state": drive_motor_state,
            "steering_motor_state": steering_motor_state,
            "gear_state": self.gear_state,
            "control_mode_state": self.control_mode_state,
            "emergency_stop": self.emergency_stop,
            "fault_state": fault_state,
            "communication_state": communication_state,
            "command_state": command_state,
        }
        summary_msg = String()
        summary_msg.data = json.dumps(summary, separators=(",", ":"))
        self.pub_summary.publish(summary_msg)


def main():
    rclpy.init()
    node = SimulatedChassisFeedbackNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
