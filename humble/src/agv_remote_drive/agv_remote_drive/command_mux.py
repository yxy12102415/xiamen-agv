import json

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class RemoteDriveCommandMux(Node):
    """Select exactly one control source before the safety supervisor."""

    def __init__(self):
        super().__init__("remote_drive_command_mux")

        self.declare_parameter("auto_control_topic", "/control/command/auto/control_cmd")
        self.declare_parameter(
            "auto_two_axis_topic", "/agv/control/auto/two_axis_steer_cmd"
        )
        self.declare_parameter("auto_gear_topic", "/control/command/auto/gear_cmd")
        self.declare_parameter("remote_control_topic", "/remote/control/control_cmd")
        self.declare_parameter(
            "remote_two_axis_topic", "/remote/control/two_axis_steer_cmd"
        )
        self.declare_parameter("remote_gear_topic", "/remote/control/gear_cmd")
        self.declare_parameter("remote_deadman_topic", "/remote/control/deadman")
        self.declare_parameter("remote_estop_topic", "/remote/control/emergency_stop")
        self.declare_parameter("remote_video_state_topic", "/remote/network/video_state")
        self.declare_parameter("mode_request_topic", "/agv/control/authority/request")
        self.declare_parameter("mode_state_topic", "/agv/control/authority/state")
        self.declare_parameter("output_control_topic", "/control/command/control_cmd_raw")
        self.declare_parameter(
            "output_two_axis_topic", "/agv/control/two_axis_steer_cmd_raw"
        )
        self.declare_parameter("output_gear_topic", "/control/command/gear_cmd_raw")
        self.declare_parameter("odom_topic", "/localization/kinematic_state")
        self.declare_parameter("default_mode", "AUTO")
        self.declare_parameter("update_rate", 30.0)
        self.declare_parameter("input_timeout", 0.25)
        self.declare_parameter("deadman_timeout", 0.35)
        self.declare_parameter("switch_speed_limit", 0.05)
        self.declare_parameter("remote_max_velocity", 3.0)
        self.declare_parameter("remote_max_reverse_velocity", 0.5)
        self.declare_parameter("remote_max_acceleration", 0.35)
        self.declare_parameter("remote_max_brake_decel", 4.0)
        self.declare_parameter("front_steer_limit", 0.47822)
        self.declare_parameter("rear_steer_limit", 0.39444)
        self.declare_parameter("require_remote_video", False)
        self.declare_parameter("required_video_cameras", ["front"])
        self.declare_parameter("remote_video_timeout", 1.5)

        default_mode = str(self.get_parameter("default_mode").value).upper()
        self.mode = default_mode if default_mode in ("AUTO", "REMOTE") else "AUTO"
        self.input_timeout = float(self.get_parameter("input_timeout").value)
        self.deadman_timeout = float(self.get_parameter("deadman_timeout").value)
        self.switch_speed_limit = float(self.get_parameter("switch_speed_limit").value)
        self.remote_max_velocity = float(self.get_parameter("remote_max_velocity").value)
        self.remote_max_reverse_velocity = float(
            self.get_parameter("remote_max_reverse_velocity").value
        )
        self.remote_max_acceleration = float(
            self.get_parameter("remote_max_acceleration").value
        )
        self.remote_max_brake_decel = float(
            self.get_parameter("remote_max_brake_decel").value
        )
        self.front_steer_limit = float(self.get_parameter("front_steer_limit").value)
        self.rear_steer_limit = float(self.get_parameter("rear_steer_limit").value)
        self.require_remote_video = bool(
            self.get_parameter("require_remote_video").value
        )
        self.required_video_cameras = tuple(
            str(name) for name in self.get_parameter("required_video_cameras").value
        )
        self.remote_video_timeout = float(
            self.get_parameter("remote_video_timeout").value
        )

        self.auto_control = None
        self.auto_two_axis = None
        self.auto_gear = None
        self.remote_control = None
        self.remote_two_axis = None
        self.remote_gear = None
        self.deadman = False
        self.remote_estop = False
        self.vehicle_speed = 0.0
        self.times = {}
        self.video_times = {}
        self.last_state = None
        self.last_rejection = ""

        self.create_subscription(
            Control,
            self.get_parameter("auto_control_topic").value,
            lambda msg: self.remember("auto_control", msg),
            1,
        )
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter("auto_two_axis_topic").value,
            lambda msg: self.remember("auto_two_axis", msg),
            1,
        )
        self.create_subscription(
            GearCommand,
            self.get_parameter("auto_gear_topic").value,
            lambda msg: self.remember("auto_gear", msg),
            1,
        )
        self.create_subscription(
            Control,
            self.get_parameter("remote_control_topic").value,
            lambda msg: self.remember("remote_control", msg),
            1,
        )
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter("remote_two_axis_topic").value,
            lambda msg: self.remember("remote_two_axis", msg),
            1,
        )
        self.create_subscription(
            GearCommand,
            self.get_parameter("remote_gear_topic").value,
            lambda msg: self.remember("remote_gear", msg),
            1,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("remote_deadman_topic").value,
            self.on_deadman,
            1,
        )
        self.create_subscription(
            String,
            self.get_parameter("remote_video_state_topic").value,
            self.on_video_state,
            20,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("remote_estop_topic").value,
            self.on_remote_estop,
            1,
        )
        self.create_subscription(
            String,
            self.get_parameter("mode_request_topic").value,
            self.on_mode_request,
            10,
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.on_odom, 1
        )

        self.control_pub = self.create_publisher(
            Control, self.get_parameter("output_control_topic").value, 1
        )
        self.two_axis_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter("output_two_axis_topic").value, 1
        )
        self.gear_pub = self.create_publisher(
            GearCommand, self.get_parameter("output_gear_topic").value, 1
        )
        self.state_pub = self.create_publisher(
            String, self.get_parameter("mode_state_topic").value, 10
        )

        update_rate = float(self.get_parameter("update_rate").value)
        self.timer = self.create_timer(1.0 / max(1.0, update_rate), self.on_timer)
        self.get_logger().info(
            "Remote-drive command mux started in %s mode; switching requires speed <= %.2f m/s"
            % (self.mode, self.switch_speed_limit)
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def remember(self, name, msg):
        setattr(self, name, msg)
        self.times[name] = self.now_seconds()

    def fresh(self, name, timeout=None):
        timestamp = self.times.get(name)
        if timestamp is None:
            return False
        return self.now_seconds() - timestamp <= (
            self.input_timeout if timeout is None else timeout
        )

    def on_deadman(self, msg):
        self.deadman = bool(msg.data)
        self.times["deadman"] = self.now_seconds()

    def on_remote_estop(self, msg):
        self.remote_estop = bool(msg.data)
        self.times["remote_estop"] = self.now_seconds()

    def on_video_state(self, msg):
        try:
            state = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        # H.264 sender diagnostics only prove that packets left the vehicle side.
        # The receiver (or the raw loopback relay) must confirm usable video.
        if state.get("role") == "sender":
            return
        now = self.now_seconds()
        for camera_name, camera in state.get("cameras", {}).items():
            if camera.get("state") == "LIVE":
                self.video_times[str(camera_name)] = now
            else:
                self.video_times.pop(str(camera_name), None)

    def video_ready(self):
        if not self.require_remote_video:
            return True
        now = self.now_seconds()
        return bool(self.required_video_cameras) and all(
            camera_name in self.video_times
            and now - self.video_times[camera_name] <= self.remote_video_timeout
            for camera_name in self.required_video_cameras
        )

    def on_odom(self, msg):
        self.vehicle_speed = float(msg.twist.twist.linear.x)

    def on_mode_request(self, msg):
        requested = str(msg.data).strip().upper()
        if requested not in ("AUTO", "REMOTE"):
            self.last_rejection = "invalid_request=%s" % requested
            self.get_logger().warning("Rejected unknown control mode '%s'" % requested)
            return
        if requested == self.mode:
            return
        if abs(self.vehicle_speed) > self.switch_speed_limit:
            self.last_rejection = "switch_rejected_vehicle_moving"
            self.get_logger().warning(
                "Rejected %s mode: vehicle speed %.3f m/s exceeds %.3f m/s"
                % (requested, self.vehicle_speed, self.switch_speed_limit)
            )
            return
        self.mode = requested
        self.last_rejection = ""
        self.get_logger().info("Control authority changed to %s" % self.mode)

    def stop_messages(self):
        stamp = self.get_clock().now().to_msg()
        stop_acceleration = self.stopping_acceleration()
        control = Control()
        control.stamp = stamp
        control.longitudinal.stamp = stamp
        control.longitudinal.velocity = 0.0
        control.longitudinal.acceleration = stop_acceleration
        control.longitudinal.is_defined_acceleration = True
        control.lateral.stamp = stamp
        control.lateral.steering_tire_angle = 0.0
        control.lateral.steering_tire_rotation_rate = 0.0
        control.lateral.is_defined_steering_tire_rotation_rate = True

        two_axis = Float32MultiArray()
        two_axis.data = [0.0, 0.0, 0.0, float(stop_acceleration)]

        gear = GearCommand()
        gear.stamp = stamp
        gear.command = GearCommand.DRIVE
        return control, two_axis, gear

    def stopping_acceleration(self):
        if self.vehicle_speed > 0.02:
            return -self.remote_max_brake_decel
        if self.vehicle_speed < -0.02:
            return self.remote_max_acceleration
        return 0.0

    def sanitized_remote_messages(self):
        stamp = self.get_clock().now().to_msg()
        source = self.remote_control
        control = Control()
        control.stamp = stamp
        control.longitudinal.stamp = stamp
        control.longitudinal.velocity = clamp(
            float(source.longitudinal.velocity),
            -self.remote_max_reverse_velocity,
            self.remote_max_velocity,
        )
        acceleration = 0.0
        if source.longitudinal.is_defined_acceleration:
            acceleration = clamp(
                float(source.longitudinal.acceleration),
                -self.remote_max_brake_decel,
                self.remote_max_acceleration,
            )
        if abs(control.longitudinal.velocity) < 1.0e-3:
            acceleration = self.stopping_acceleration()
        control.longitudinal.acceleration = acceleration
        control.longitudinal.is_defined_acceleration = True
        control.lateral.stamp = stamp
        control.lateral.steering_tire_angle = clamp(
            float(source.lateral.steering_tire_angle),
            -self.front_steer_limit,
            self.front_steer_limit,
        )
        control.lateral.steering_tire_rotation_rate = float(
            source.lateral.steering_tire_rotation_rate
        )
        control.lateral.is_defined_steering_tire_rotation_rate = True

        two_axis = Float32MultiArray()
        source_two_axis = self.remote_two_axis.data
        front = source_two_axis[0] if len(source_two_axis) > 0 else control.lateral.steering_tire_angle
        rear = source_two_axis[1] if len(source_two_axis) > 1 else 0.0
        two_axis.data = [
            float(clamp(front, -self.front_steer_limit, self.front_steer_limit)),
            float(clamp(rear, -self.rear_steer_limit, self.rear_steer_limit)),
            float(control.longitudinal.velocity),
            float(control.longitudinal.acceleration),
        ]

        gear = self.remote_gear
        gear.stamp = stamp
        return control, two_axis, gear

    def publish(self, control, two_axis, gear):
        self.control_pub.publish(control)
        self.two_axis_pub.publish(two_axis)
        self.gear_pub.publish(gear)

    def publish_state(self, state):
        detail = "%s:%s" % (self.mode, state)
        if self.last_rejection:
            detail += ":" + self.last_rejection
        if detail == self.last_state:
            return
        self.last_state = detail
        msg = String()
        msg.data = detail
        self.state_pub.publish(msg)
        self.get_logger().info("Authority state: %s" % detail)

    def on_timer(self):
        if self.mode == "AUTO":
            if self.fresh("auto_control") and self.fresh("auto_two_axis"):
                gear = self.auto_gear if self.fresh("auto_gear") else self.stop_messages()[2]
                self.publish(self.auto_control, self.auto_two_axis, gear)
                self.publish_state("ACTIVE")
            else:
                self.publish(*self.stop_messages())
                self.publish_state("STOPPED_AUTO_STALE")
            return

        remote_ready = (
            self.fresh("remote_control")
            and self.fresh("remote_two_axis")
            and self.fresh("remote_gear")
            and self.fresh("deadman", self.deadman_timeout)
            and self.deadman
            and not self.remote_estop
            and self.video_ready()
        )
        if remote_ready:
            self.publish(*self.sanitized_remote_messages())
            self.publish_state("ACTIVE")
        else:
            self.publish(*self.stop_messages())
            if self.remote_estop:
                reason = "ESTOP"
            elif not self.video_ready():
                reason = "VIDEO_STALE"
            else:
                reason = "DEADMAN_OR_INPUT_STALE"
            self.publish_state("STOPPED_" + reason)


def main(args=None):
    rclpy.init(args=args)
    node = RemoteDriveCommandMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
