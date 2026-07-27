from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import ControlModeReport, GearCommand, VelocityReport
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def stamp_now(node):
    return node.get_clock().now().to_msg()


class SafetySupervisorNode(Node):
    def __init__(self):
        super().__init__("agv_safety_supervisor")

        self.declare_parameter("raw_control_topic", "/control/command/control_cmd_raw")
        self.declare_parameter("raw_two_axis_steer_topic", "/agv/control/two_axis_steer_cmd_raw")
        self.declare_parameter("raw_gear_topic", "/control/command/gear_cmd_raw")
        self.declare_parameter("safe_control_topic", "/control/command/control_cmd")
        self.declare_parameter("safe_two_axis_steer_topic", "/agv/control/two_axis_steer_cmd")
        self.declare_parameter("safe_gear_topic", "/control/command/gear_cmd")
        self.declare_parameter("odom_topic", "/localization/kinematic_state")
        self.declare_parameter("velocity_status_topic", "/vehicle/status/velocity_status")
        self.declare_parameter("control_mode_topic", "/vehicle/status/control_mode")
        self.declare_parameter("front_steer_topic", "/agv/status/front_steer")
        self.declare_parameter("rear_steer_topic", "/agv/status/rear_steer")
        self.declare_parameter("emergency_stop_topic", "/agv/safety/emergency_stop")
        self.declare_parameter(
            "remote_emergency_stop_topic", "/remote/control/emergency_stop"
        )
        self.declare_parameter("state_topic", "/agv/safety/state")
        self.declare_parameter("update_rate", 50.0)
        self.declare_parameter("command_timeout", 0.2)
        self.declare_parameter("odom_timeout", 0.3)
        self.declare_parameter("feedback_timeout", 0.5)
        self.declare_parameter("require_autonomous_mode", True)
        self.declare_parameter("max_velocity", 5.0)
        self.declare_parameter("max_reverse_velocity", 0.5)
        self.declare_parameter("max_drive_accel", 0.35)
        self.declare_parameter("max_brake_decel", 1.0)
        self.declare_parameter("front_steer_limit", 0.47822)
        self.declare_parameter("rear_steer_limit", 0.39444)
        self.declare_parameter("stop_acceleration", -1.0)
        self.declare_parameter("stop_gear", "drive")

        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)
        self.feedback_timeout = float(self.get_parameter("feedback_timeout").value)
        self.require_autonomous_mode = bool(self.get_parameter("require_autonomous_mode").value)
        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_reverse_velocity = float(self.get_parameter("max_reverse_velocity").value)
        self.max_drive_accel = float(self.get_parameter("max_drive_accel").value)
        self.max_brake_decel = float(self.get_parameter("max_brake_decel").value)
        self.front_steer_limit = float(self.get_parameter("front_steer_limit").value)
        self.rear_steer_limit = float(self.get_parameter("rear_steer_limit").value)
        self.stop_acceleration = -abs(float(self.get_parameter("stop_acceleration").value))

        self.raw_control = None
        self.raw_two_axis = None
        self.raw_gear = None
        self.odom = None
        self.velocity_status = None
        self.control_mode = None
        self.front_steer = 0.0
        self.rear_steer = 0.0
        self.emergency_stop = False
        self.remote_emergency_stop = False

        self.raw_control_time = None
        self.raw_two_axis_time = None
        self.raw_gear_time = None
        self.odom_time = None
        self.velocity_status_time = None
        self.control_mode_time = None
        self.front_steer_time = None
        self.rear_steer_time = None

        self.create_subscription(
            Control, self.get_parameter("raw_control_topic").value, self.on_raw_control, 1
        )
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter("raw_two_axis_steer_topic").value,
            self.on_raw_two_axis,
            1,
        )
        self.create_subscription(
            GearCommand, self.get_parameter("raw_gear_topic").value, self.on_raw_gear, 1
        )
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.on_odom, 1)
        self.create_subscription(
            VelocityReport,
            self.get_parameter("velocity_status_topic").value,
            self.on_velocity_status,
            1,
        )
        self.create_subscription(
            ControlModeReport,
            self.get_parameter("control_mode_topic").value,
            self.on_control_mode,
            1,
        )
        self.create_subscription(
            Float32, self.get_parameter("front_steer_topic").value, self.on_front_steer, 1
        )
        self.create_subscription(
            Float32, self.get_parameter("rear_steer_topic").value, self.on_rear_steer, 1
        )
        self.create_subscription(
            Bool, self.get_parameter("emergency_stop_topic").value, self.on_emergency_stop, 1
        )
        self.create_subscription(
            Bool,
            self.get_parameter("remote_emergency_stop_topic").value,
            self.on_remote_emergency_stop,
            1,
        )

        self.control_pub = self.create_publisher(
            Control, self.get_parameter("safe_control_topic").value, 1
        )
        self.two_axis_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter("safe_two_axis_steer_topic").value, 1
        )
        self.gear_pub = self.create_publisher(
            GearCommand, self.get_parameter("safe_gear_topic").value, 1
        )
        self.state_pub = self.create_publisher(String, self.get_parameter("state_topic").value, 10)

        update_rate = float(self.get_parameter("update_rate").value)
        self.timer = self.create_timer(1.0 / max(1.0, update_rate), self.on_timer)
        self.get_logger().info("AGV safety supervisor started")

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def age_ok(self, timestamp, timeout):
        if timestamp is None:
            return False
        return self.now_seconds() - timestamp <= timeout

    def remember_time(self):
        return self.now_seconds()

    def on_raw_control(self, msg):
        self.raw_control = msg
        self.raw_control_time = self.remember_time()

    def on_raw_two_axis(self, msg):
        self.raw_two_axis = msg
        self.raw_two_axis_time = self.remember_time()

    def on_raw_gear(self, msg):
        self.raw_gear = msg
        self.raw_gear_time = self.remember_time()

    def on_odom(self, msg):
        self.odom = msg
        self.odom_time = self.remember_time()

    def on_velocity_status(self, msg):
        self.velocity_status = msg
        self.velocity_status_time = self.remember_time()

    def on_control_mode(self, msg):
        self.control_mode = msg
        self.control_mode_time = self.remember_time()

    def on_front_steer(self, msg):
        self.front_steer = float(msg.data)
        self.front_steer_time = self.remember_time()

    def on_rear_steer(self, msg):
        self.rear_steer = float(msg.data)
        self.rear_steer_time = self.remember_time()

    def on_emergency_stop(self, msg):
        self.emergency_stop = bool(msg.data)

    def on_remote_emergency_stop(self, msg):
        self.remote_emergency_stop = bool(msg.data)

    def control_mode_is_safe(self):
        if not self.require_autonomous_mode:
            return True
        if not self.age_ok(self.control_mode_time, self.feedback_timeout):
            return False
        return self.control_mode.mode in (
            ControlModeReport.AUTONOMOUS,
            ControlModeReport.AUTONOMOUS_STEER_ONLY,
            ControlModeReport.AUTONOMOUS_VELOCITY_ONLY,
        )

    def raw_command_is_fresh(self):
        return self.age_ok(self.raw_control_time, self.command_timeout) or self.age_ok(
            self.raw_two_axis_time, self.command_timeout
        )

    def collect_faults(self):
        faults = []
        if self.emergency_stop or self.remote_emergency_stop:
            faults.append("emergency_stop")
        if not self.raw_command_is_fresh():
            faults.append("command_timeout")
        if not self.age_ok(self.odom_time, self.odom_timeout):
            faults.append("odom_timeout")
        if not self.control_mode_is_safe():
            faults.append("control_mode_not_autonomous")
        if self.velocity_status is not None and abs(self.velocity_status.longitudinal_velocity) > self.max_velocity:
            faults.append("velocity_over_limit")
        return faults

    def stop_gear_command(self):
        gear = GearCommand()
        gear.stamp = stamp_now(self)
        stop_gear = str(self.get_parameter("stop_gear").value).lower()
        gear.command = GearCommand.PARK if stop_gear == "park" else GearCommand.DRIVE
        return gear

    def publish_stop(self, faults):
        stamp = stamp_now(self)
        actual_velocity = 0.0
        if self.velocity_status is not None:
            actual_velocity = float(self.velocity_status.longitudinal_velocity)
        elif self.odom is not None:
            actual_velocity = float(self.odom.twist.twist.linear.x)
        if actual_velocity > 0.02:
            stop_acceleration = self.stop_acceleration
        elif actual_velocity < -0.02:
            stop_acceleration = min(abs(self.stop_acceleration), self.max_drive_accel)
        else:
            stop_acceleration = 0.0

        control = Control()
        control.stamp = stamp
        control.longitudinal.stamp = stamp
        control.longitudinal.velocity = 0.0
        control.longitudinal.acceleration = stop_acceleration
        control.longitudinal.is_defined_acceleration = True
        control.lateral.stamp = stamp
        control.lateral.steering_tire_angle = clamp(
            self.front_steer, -self.front_steer_limit, self.front_steer_limit
        )
        control.lateral.steering_tire_rotation_rate = 0.0
        control.lateral.is_defined_steering_tire_rotation_rate = True
        self.control_pub.publish(control)

        two_axis = Float32MultiArray()
        two_axis.data = [
            float(clamp(self.front_steer, -self.front_steer_limit, self.front_steer_limit)),
            float(clamp(self.rear_steer, -self.rear_steer_limit, self.rear_steer_limit)),
            0.0,
            float(stop_acceleration),
        ]
        self.two_axis_pub.publish(two_axis)
        self.gear_pub.publish(self.stop_gear_command())
        self.publish_state("STOP", faults)

    def sanitized_control(self):
        stamp = stamp_now(self)
        source = self.raw_control if self.raw_control is not None else Control()
        control = Control()
        control.stamp = stamp
        control.longitudinal.stamp = stamp
        velocity = clamp(
            float(source.longitudinal.velocity),
            -self.max_reverse_velocity,
            self.max_velocity,
        )
        acceleration = 0.0
        if source.longitudinal.is_defined_acceleration:
            acceleration = clamp(
                float(source.longitudinal.acceleration),
                -self.max_brake_decel,
                self.max_drive_accel,
            )
        control.longitudinal.velocity = velocity
        control.longitudinal.acceleration = acceleration
        control.longitudinal.is_defined_acceleration = True
        control.lateral.stamp = stamp
        control.lateral.steering_tire_angle = clamp(
            float(source.lateral.steering_tire_angle),
            -self.front_steer_limit,
            self.front_steer_limit,
        )
        control.lateral.steering_tire_rotation_rate = float(source.lateral.steering_tire_rotation_rate)
        control.lateral.is_defined_steering_tire_rotation_rate = source.lateral.is_defined_steering_tire_rotation_rate
        return control

    def sanitized_two_axis(self, control):
        two_axis = Float32MultiArray()
        if self.raw_two_axis is not None and len(self.raw_two_axis.data) >= 2:
            front = float(self.raw_two_axis.data[0])
            rear = float(self.raw_two_axis.data[1])
            velocity = float(self.raw_two_axis.data[2]) if len(self.raw_two_axis.data) >= 3 else control.longitudinal.velocity
            acceleration = (
                float(self.raw_two_axis.data[3])
                if len(self.raw_two_axis.data) >= 4
                else control.longitudinal.acceleration
            )
        else:
            front = control.lateral.steering_tire_angle
            rear = 0.0
            velocity = control.longitudinal.velocity
            acceleration = control.longitudinal.acceleration
        two_axis.data = [
            float(clamp(front, -self.front_steer_limit, self.front_steer_limit)),
            float(clamp(rear, -self.rear_steer_limit, self.rear_steer_limit)),
            float(clamp(velocity, -self.max_reverse_velocity, self.max_velocity)),
            float(clamp(acceleration, -self.max_brake_decel, self.max_drive_accel)),
        ]
        return two_axis

    def publish_safe_command(self):
        control = self.sanitized_control()
        self.control_pub.publish(control)
        self.two_axis_pub.publish(self.sanitized_two_axis(control))
        if self.raw_gear is not None and self.age_ok(self.raw_gear_time, self.command_timeout):
            self.gear_pub.publish(self.raw_gear)
        else:
            self.gear_pub.publish(self.stop_gear_command())
        self.publish_state("PASS", [])

    def publish_state(self, state, faults):
        msg = String()
        msg.data = "%s%s" % (state, "" if not faults else ":" + ",".join(faults))
        self.state_pub.publish(msg)

    def on_timer(self):
        faults = self.collect_faults()
        if faults:
            self.publish_stop(faults)
        else:
            self.publish_safe_command()


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
