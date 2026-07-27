import csv
import math
import os

from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import Trajectory
from autoware_vehicle_msgs.msg import GearCommand, SteeringReport
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class Pid:
    def __init__(self, kp, ki, kd, integral_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.integral = 0.0
        self.previous_error = None

    def reset(self):
        self.integral = 0.0
        self.previous_error = None

    def update(self, error, dt):
        if dt <= 0.0:
            return self.kp * error
        derivative = 0.0
        if self.previous_error is not None:
            derivative = (error - self.previous_error) / dt
        self.integral = clamp(
            self.integral + error * dt,
            -self.integral_limit,
            self.integral_limit,
        )
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class PidTrajectoryController(Node):
    def __init__(self):
        super().__init__("pid_trajectory_controller")
        self.declare_parameter(
            "trajectory_topic", "/planning/scenario_planning/trajectory"
        )
        self.declare_parameter("odom_topic", "/localization/kinematic_state")
        self.declare_parameter(
            "steering_topic", "/vehicle/status/steering_status"
        )
        self.declare_parameter("control_topic", "/control/command/control_cmd")
        self.declare_parameter(
            "two_axis_steer_topic", "/agv/control/two_axis_steer_cmd"
        )
        self.declare_parameter("gear_topic", "/control/command/gear_cmd")
        self.declare_parameter("steering_mode_topic", "/agv/steering_mode_cmd")
        self.declare_parameter("control_rate", 30.0)
        self.declare_parameter("target_speed", 4.0)
        self.declare_parameter("max_drive_accel", 0.35)
        self.declare_parameter("max_brake_decel", 0.35)
        self.declare_parameter("stop_brake_decel", 1.0)
        self.declare_parameter("steer_lim", 0.5229795759)
        self.declare_parameter("steer_rate_lim", 0.261799)
        self.declare_parameter("wheelbase_f", 4.65)
        self.declare_parameter("wheelbase_r", 4.65)
        self.declare_parameter("axle2_x", 3.0)
        self.declare_parameter("steering_mode", "independent_4ws")
        self.declare_parameter("asymmetric_rear_steer_ratio", -0.5)
        self.declare_parameter("rear_steer_lim", 0.4505604976)
        self.declare_parameter("independent_beta_limit", 0.22)
        self.declare_parameter("independent_curvature_feedback_limit", 0.045)
        self.declare_parameter("independent_rear_steer_scale", 0.75)
        self.declare_parameter("curvature_preview_distance", 5.0)
        self.declare_parameter("feedforward_steer_limit", 0.42)
        self.declare_parameter("lateral_kp", 0.12)
        self.declare_parameter("lateral_ki", 0.003)
        self.declare_parameter("lateral_kd", 0.06)
        self.declare_parameter("lateral_integral_limit", 2.0)
        self.declare_parameter("heading_correction_limit", 0.35)
        self.declare_parameter("heading_kp", 0.75)
        self.declare_parameter("heading_ki", 0.01)
        self.declare_parameter("heading_kd", 0.05)
        self.declare_parameter("heading_integral_limit", 0.5)
        self.declare_parameter("speed_kp", 0.8)
        self.declare_parameter("speed_ki", 0.08)
        self.declare_parameter("speed_kd", 0.02)
        self.declare_parameter("speed_integral_limit", 2.0)
        self.declare_parameter("closed_trajectory", True)
        self.declare_parameter("enable_loop_stop", True)
        self.declare_parameter("loop_stop_duration", 3.0)
        self.declare_parameter("loop_stop_speed_threshold", 0.08)
        self.declare_parameter("loop_stop_enter_speed_threshold", 0.30)
        self.declare_parameter("loop_stop_approach_distance", 30.0)
        self.declare_parameter("loop_stop_position_tolerance", 0.35)
        self.declare_parameter("loop_stop_delay_compensation", 0.5)
        self.declare_parameter("loop_stop_steer_decay_rate", 0.45)
        self.declare_parameter("loop_restart_lock_distance", 8.0)
        self.declare_parameter("enable_error_log", True)
        self.declare_parameter("error_log_path", "/tmp/agv_pid_error.csv")

        self.control_rate = float(self.get_parameter("control_rate").value)
        self.dt = 1.0 / max(1.0, self.control_rate)
        self.target_speed = float(self.get_parameter("target_speed").value)
        self.max_drive_accel = float(
            self.get_parameter("max_drive_accel").value
        )
        self.max_brake_decel = float(
            self.get_parameter("max_brake_decel").value
        )
        self.stop_brake_decel = float(
            self.get_parameter("stop_brake_decel").value
        )
        self.steer_lim = float(self.get_parameter("steer_lim").value)
        self.rear_steer_lim = float(
            self.get_parameter("rear_steer_lim").value
        )
        self.steer_rate_lim = float(
            self.get_parameter("steer_rate_lim").value
        )
        self.wheelbase_f = float(self.get_parameter("wheelbase_f").value)
        self.wheelbase_r = float(self.get_parameter("wheelbase_r").value)
        self.axle2_x = float(self.get_parameter("axle2_x").value)
        self.steering_mode = str(self.get_parameter("steering_mode").value)
        self.asymmetric_rear_steer_ratio = float(
            self.get_parameter("asymmetric_rear_steer_ratio").value
        )
        self.independent_beta_limit = float(
            self.get_parameter("independent_beta_limit").value
        )
        self.independent_curvature_feedback_limit = float(
            self.get_parameter("independent_curvature_feedback_limit").value
        )
        self.independent_rear_steer_scale = float(
            self.get_parameter("independent_rear_steer_scale").value
        )
        self.curvature_preview_distance = float(
            self.get_parameter("curvature_preview_distance").value
        )
        self.feedforward_steer_limit = float(
            self.get_parameter("feedforward_steer_limit").value
        )
        self.heading_correction_limit = float(
            self.get_parameter("heading_correction_limit").value
        )
        self.closed_trajectory = bool(
            self.get_parameter("closed_trajectory").value
        )
        self.enable_loop_stop = bool(
            self.get_parameter("enable_loop_stop").value
        )
        self.loop_stop_duration = float(
            self.get_parameter("loop_stop_duration").value
        )
        self.loop_stop_speed_threshold = float(
            self.get_parameter("loop_stop_speed_threshold").value
        )
        self.loop_stop_enter_speed_threshold = float(
            self.get_parameter("loop_stop_enter_speed_threshold").value
        )
        self.loop_stop_approach_distance = float(
            self.get_parameter("loop_stop_approach_distance").value
        )
        self.loop_stop_position_tolerance = float(
            self.get_parameter("loop_stop_position_tolerance").value
        )
        self.loop_stop_delay_compensation = float(
            self.get_parameter("loop_stop_delay_compensation").value
        )
        self.loop_stop_steer_decay_rate = float(
            self.get_parameter("loop_stop_steer_decay_rate").value
        )
        self.loop_restart_lock_distance = float(
            self.get_parameter("loop_restart_lock_distance").value
        )

        self.lateral_pid = Pid(
            self.get_parameter("lateral_kp").value,
            self.get_parameter("lateral_ki").value,
            self.get_parameter("lateral_kd").value,
            self.get_parameter("lateral_integral_limit").value,
        )
        self.heading_pid = Pid(
            self.get_parameter("heading_kp").value,
            self.get_parameter("heading_ki").value,
            self.get_parameter("heading_kd").value,
            self.get_parameter("heading_integral_limit").value,
        )
        self.speed_pid = Pid(
            self.get_parameter("speed_kp").value,
            self.get_parameter("speed_ki").value,
            self.get_parameter("speed_kd").value,
            self.get_parameter("speed_integral_limit").value,
        )

        self.trajectory = []
        self.trajectory_s = []
        self.trajectory_length = 0.0
        self.odom = None
        self.current_steer = 0.0
        self.last_steer_cmd = 0.0
        self.last_rear_steer_cmd = 0.0
        self.last_accel_cmd = 0.0
        self.last_ref_index = None
        self.loop_stop_state = None
        self.loop_stop_until = None
        self.loop_stop_armed = True
        self.loop_restart_lock_s = None

        self.error_log_file = None
        self.error_log_writer = None
        if bool(self.get_parameter("enable_error_log").value):
            path = str(self.get_parameter("error_log_path").value)
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            self.error_log_file = open(
                path, "w", newline="", encoding="utf-8"
            )
            self.error_log_writer = csv.writer(self.error_log_file)
            self.error_log_writer.writerow(
                [
                    "time",
                    "x",
                    "y",
                    "yaw",
                    "vx",
                    "ref_index",
                    "ref_x",
                    "ref_y",
                    "ref_yaw",
                    "ref_speed",
                    "lateral_error",
                    "heading_error",
                    "feedforward_or_curvature",
                    "front_steer_command",
                    "rear_steer_command",
                    "accel_command",
                ]
            )

        self.create_subscription(
            Trajectory,
            self.get_parameter("trajectory_topic").value,
            self.on_trajectory,
            1,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.on_odom,
            1,
        )
        self.create_subscription(
            SteeringReport,
            self.get_parameter("steering_topic").value,
            self.on_steering,
            1,
        )
        self.control_pub = self.create_publisher(
            Control, self.get_parameter("control_topic").value, 1
        )
        self.two_axis_steer_pub = self.create_publisher(
            Float32MultiArray,
            self.get_parameter("two_axis_steer_topic").value,
            1,
        )
        self.gear_pub = self.create_publisher(
            GearCommand, self.get_parameter("gear_topic").value, 1
        )
        self.steering_mode_pub = self.create_publisher(
            String, self.get_parameter("steering_mode_topic").value, 1
        )
        self.timer = self.create_timer(self.dt, self.on_timer)

        mode = String()
        mode.data = self.steering_mode
        self.steering_mode_pub.publish(mode)
        self.get_logger().info(
            "Trajectory PID started: rate=%.1f Hz steering_mode=%s"
            % (self.control_rate, self.steering_mode)
        )

    def on_trajectory(self, msg):
        points = []
        for point in msg.points:
            points.append(
                (
                    point.pose.position.x,
                    point.pose.position.y,
                    yaw_from_quaternion(point.pose.orientation),
                    point.longitudinal_velocity_mps,
                    0.0,
                )
            )
        curvatures = [0.0] * len(points)
        for index in range(1, len(points) - 1):
            x0, y0, yaw0, _, _ = points[index - 1]
            x1, y1, yaw1, _, _ = points[index + 1]
            distance = max(1e-3, math.hypot(x1 - x0, y1 - y0))
            curvatures[index] = normalize_angle(yaw1 - yaw0) / distance
        if len(points) > 1:
            curvatures[0] = curvatures[1]
            curvatures[-1] = curvatures[-2]
        for _ in range(2):
            smoothed = curvatures[:]
            for index in range(1, len(curvatures) - 1):
                smoothed[index] = (
                    0.25 * curvatures[index - 1]
                    + 0.5 * curvatures[index]
                    + 0.25 * curvatures[index + 1]
                )
            curvatures = smoothed
        self.trajectory = [
            (x, y, yaw, speed, curvatures[index])
            for index, (x, y, yaw, speed, _) in enumerate(points)
        ]
        self.trajectory_s = [0.0]
        for index in range(1, len(self.trajectory)):
            x0, y0, *_ = self.trajectory[index - 1]
            x1, y1, *_ = self.trajectory[index]
            self.trajectory_s.append(
                self.trajectory_s[-1] + math.hypot(x1 - x0, y1 - y0)
            )
        self.trajectory_length = (
            self.trajectory_s[-1] if self.trajectory_s else 0.0
        )

    def on_odom(self, msg):
        self.odom = msg

    def on_steering(self, msg):
        self.current_steer = msg.steering_tire_angle

    def sample_reference(self, target_s):
        if len(self.trajectory) < 2:
            return None
        if self.closed_trajectory and self.trajectory_length > 1e-6:
            target_s %= self.trajectory_length
        else:
            target_s = clamp(target_s, 0.0, self.trajectory_length)
        index = 0
        while (
            index + 1 < len(self.trajectory_s)
            and self.trajectory_s[index + 1] < target_s
        ):
            index += 1
        index = min(index, len(self.trajectory) - 2)
        s0 = self.trajectory_s[index]
        s1 = self.trajectory_s[index + 1]
        ratio = 0.0 if s1 <= s0 else (target_s - s0) / (s1 - s0)
        p0 = self.trajectory[index]
        p1 = self.trajectory[index + 1]
        return {
            "index": index,
            "s": target_s,
            "x": p0[0] + ratio * (p1[0] - p0[0]),
            "y": p0[1] + ratio * (p1[1] - p0[1]),
            "yaw": normalize_angle(
                p0[2] + ratio * normalize_angle(p1[2] - p0[2])
            ),
            "speed": p0[3] + ratio * (p1[3] - p0[3]),
            "curvature": p0[4] + ratio * (p1[4] - p0[4]),
        }

    def nearest_reference(self, x, y):
        segment_count = len(self.trajectory) - 1
        if self.loop_restart_lock_s is not None:
            reference = self.sample_reference(self.loop_restart_lock_s)
            self.loop_restart_lock_s += max(
                0.2,
                reference["speed"] * self.dt,
            )
            if self.loop_restart_lock_s >= self.loop_restart_lock_distance:
                self.loop_restart_lock_s = None
                self.last_ref_index = reference["index"]
            return reference

        if self.last_ref_index is None:
            indices = range(segment_count)
        elif self.closed_trajectory:
            start = (self.last_ref_index - 8) % segment_count
            count = min(segment_count, 100)
            indices = [(start + offset) % segment_count for offset in range(count)]
        else:
            start = max(0, self.last_ref_index - 8)
            indices = range(start, min(segment_count, start + 100))

        best_distance = float("inf")
        best_s = 0.0
        for index in indices:
            x0, y0, *_ = self.trajectory[index]
            x1, y1, *_ = self.trajectory[index + 1]
            dx = x1 - x0
            dy = y1 - y0
            length_squared = dx * dx + dy * dy
            ratio = 0.0
            if length_squared > 1e-9:
                ratio = clamp(
                    ((x - x0) * dx + (y - y0) * dy) / length_squared,
                    0.0,
                    1.0,
                )
            px = x0 + ratio * dx
            py = y0 + ratio * dy
            distance = (x - px) ** 2 + (y - py) ** 2
            if distance < best_distance:
                best_distance = distance
                best_s = (
                    self.trajectory_s[index]
                    + ratio * math.sqrt(length_squared)
                )
        reference = self.sample_reference(best_s)
        previous_index = self.last_ref_index
        self.last_ref_index = reference["index"]
        self.update_loop_stop(previous_index, reference["index"])
        return reference

    def update_loop_stop(self, previous_index, current_index):
        if (
            not self.enable_loop_stop
            or self.loop_stop_state is not None
            or previous_index is None
            or len(self.trajectory) < 4
        ):
            return
        segment_count = len(self.trajectory) - 1
        rearm_index = max(20, int(0.2 * segment_count))
        if not self.loop_stop_armed:
            if current_index >= rearm_index:
                self.loop_stop_armed = True
            return
        if (
            previous_index >= int(0.85 * segment_count)
            and current_index <= int(0.15 * segment_count)
        ):
            self.loop_stop_state = "braking"
            self.loop_stop_until = None
            self.loop_stop_armed = False
            self.reset_pid()
            self.get_logger().info(
                "Completed trajectory loop; stopping for %.1f s"
                % self.loop_stop_duration
            )

    def handle_loop_stop(self, vx):
        if self.loop_stop_state is None:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.loop_stop_state == "braking":
            self.publish_stop()
            if abs(vx) <= self.loop_stop_speed_threshold:
                self.loop_stop_state = "holding"
                self.loop_stop_until = now + self.loop_stop_duration
            return True
        if now >= self.loop_stop_until:
            self.loop_stop_state = None
            self.loop_stop_until = None
            self.last_ref_index = None
            self.loop_restart_lock_s = 0.0
            self.reset_pid()
            return False
        self.publish_stop()
        return True

    def rear_steer_ratio(self):
        if self.steering_mode == "front_only":
            return 0.0
        if self.steering_mode == "crab":
            return 1.0
        if self.steering_mode == "asymmetric_4ws":
            return self.asymmetric_rear_steer_ratio
        if self.steering_mode == "independent_4ws":
            return self.asymmetric_rear_steer_ratio
        if self.steering_mode == "axle12_steer":
            return 0.0
        return -1.0

    def loop_stop_velocity_limit(self, reference, vx):
        if (
            not self.enable_loop_stop
            or not self.closed_trajectory
            or not self.loop_stop_armed
            or self.loop_stop_state is not None
            or self.trajectory_length <= 1e-6
        ):
            return None

        segment_count = len(self.trajectory) - 1
        if reference["index"] < int(0.70 * segment_count):
            return None

        remaining_distance = max(
            0.0, self.trajectory_length - reference["s"]
        )
        if remaining_distance > self.loop_stop_approach_distance:
            return None

        delay_distance = max(0.0, vx) * self.loop_stop_delay_compensation
        usable_distance = max(
            0.0,
            remaining_distance
            - self.loop_stop_position_tolerance
            - delay_distance,
        )
        velocity_limit = math.sqrt(
            2.0 * max(1e-3, self.max_brake_decel) * usable_distance
        )

        if (
            remaining_distance <= self.loop_stop_position_tolerance
            or velocity_limit <= self.loop_stop_enter_speed_threshold
        ):
            self.loop_stop_state = "braking"
            self.loop_stop_until = None
            self.loop_stop_armed = False
            self.reset_pid()
            self.get_logger().info(
                "Reached loop stop point with %.2f m remaining; braking and holding for %.1f s"
                % (remaining_distance, self.loop_stop_duration)
            )
            return 0.0
        return velocity_limit

    def curvature_from_steer(self, steer):
        rear_steer = clamp(
            steer * self.rear_steer_ratio(), -self.rear_steer_lim, self.rear_steer_lim
        )
        beta_numerator = self.wheelbase_f * math.tan(
            rear_steer
        ) + self.wheelbase_r * math.tan(steer)
        beta_denominator = self.wheelbase_f + self.wheelbase_r
        numerator = math.tan(steer) - math.tan(rear_steer)
        denominator = math.sqrt(
            beta_numerator * beta_numerator
            + beta_denominator * beta_denominator
        )
        return numerator / max(1e-6, denominator)

    def steer_from_curvature(self, curvature):
        if self.steering_mode == "crab" or abs(curvature) < 1e-6:
            return 0.0
        sign = 1.0 if curvature > 0.0 else -1.0
        lower = 0.0
        upper = min(self.steer_lim, self.feedforward_steer_limit)
        for _ in range(24):
            middle = 0.5 * (lower + upper)
            if abs(self.curvature_from_steer(sign * middle)) < abs(curvature):
                lower = middle
            else:
                upper = middle
        return sign * 0.5 * (lower + upper)

    def preview_curvature(self, reference, vx):
        distance = max(
            self.curvature_preview_distance,
            min(8.0, abs(vx) * 1.2),
        )
        strongest = reference["curvature"]
        for sample in range(1, 9):
            future = self.sample_reference(
                reference["s"] + distance * sample / 8.0
            )
            if abs(future["curvature"]) > abs(strongest):
                strongest = future["curvature"]
        return strongest

    def preview_feedforward(self, reference, vx):
        strongest = self.preview_curvature(reference, vx)
        return self.steer_from_curvature(strongest)

    def reset_pid(self):
        self.lateral_pid.reset()
        self.heading_pid.reset()
        self.speed_pid.reset()

    def publish_control(self, velocity, acceleration, steer):
        stamp = self.get_clock().now().to_msg()
        command = Control()
        command.stamp = stamp
        command.longitudinal.stamp = stamp
        command.longitudinal.velocity = float(velocity)
        command.longitudinal.acceleration = float(acceleration)
        command.longitudinal.is_defined_acceleration = True
        command.lateral.stamp = stamp
        command.lateral.steering_tire_angle = float(steer)
        command.lateral.steering_tire_rotation_rate = float(
            (steer - self.current_steer) / self.dt
        )
        command.lateral.is_defined_steering_tire_rotation_rate = True
        self.control_pub.publish(command)

        gear = GearCommand()
        gear.stamp = stamp
        gear.command = GearCommand.DRIVE
        self.gear_pub.publish(gear)

    def publish_two_axis_steer(
        self, front_steer, rear_steer, velocity, acceleration
    ):
        if self.steering_mode not in ("independent_4ws", "axle12_steer"):
            return
        msg = Float32MultiArray()
        msg.data = [
            float(front_steer),
            float(rear_steer),
            float(velocity),
            float(acceleration),
        ]
        self.two_axis_steer_pub.publish(msg)

    def independent_steer_commands(
        self, reference, lateral_error, heading_error, vx
    ):
        length = max(1e-3, self.wheelbase_f + self.wheelbase_r)

        beta_cmd = clamp(
            heading_error - self.lateral_pid.update(lateral_error, self.dt),
            -self.independent_beta_limit,
            self.independent_beta_limit,
        )
        curvature_feedback = clamp(
            self.heading_pid.update(heading_error, self.dt) / length,
            -self.independent_curvature_feedback_limit,
            self.independent_curvature_feedback_limit,
        )
        curvature_cmd = self.preview_curvature(reference, vx) + curvature_feedback

        if self.steering_mode == "axle12_steer":
            beta_tan = math.tan(beta_cmd)
            front_steer = math.atan(beta_tan + self.wheelbase_f * curvature_cmd)
            rear_steer = math.atan(beta_tan + self.axle2_x * curvature_cmd)
        else:
            front_steer = math.atan(beta_cmd + self.wheelbase_f * curvature_cmd)
            rear_steer = math.atan(beta_cmd - self.wheelbase_r * curvature_cmd)
            rear_steer *= clamp(self.independent_rear_steer_scale, 0.0, 1.0)
        front_steer = clamp(front_steer, -self.steer_lim, self.steer_lim)
        rear_steer = clamp(rear_steer, -self.rear_steer_lim, self.rear_steer_lim)

        front_step = self.steer_rate_lim * self.dt
        rear_step = self.steer_rate_lim * self.dt
        front_steer = clamp(
            front_steer,
            self.last_steer_cmd - front_step,
            self.last_steer_cmd + front_step,
        )
        rear_steer = clamp(
            rear_steer,
            self.last_rear_steer_cmd - rear_step,
            self.last_rear_steer_cmd + rear_step,
        )
        return front_steer, rear_steer, curvature_cmd

    def publish_stop(self):
        vx = 0.0 if self.odom is None else self.odom.twist.twist.linear.x
        acceleration = 0.0
        if abs(vx) > self.loop_stop_speed_threshold:
            acceleration = -math.copysign(self.stop_brake_decel, vx)

        steer_step = max(self.steer_rate_lim, self.loop_stop_steer_decay_rate) * self.dt
        front_steer = clamp(0.0, self.last_steer_cmd - steer_step, self.last_steer_cmd + steer_step)
        rear_steer = clamp(
            0.0,
            self.last_rear_steer_cmd - steer_step,
            self.last_rear_steer_cmd + steer_step,
        )
        self.publish_control(0.0, acceleration, front_steer)
        self.publish_two_axis_steer(
            front_steer,
            rear_steer,
            0.0,
            acceleration,
        )
        self.last_steer_cmd = front_steer
        self.last_rear_steer_cmd = rear_steer
        self.last_accel_cmd = acceleration

    def on_timer(self):
        if self.odom is None or len(self.trajectory) < 2:
            self.publish_stop()
            return

        pose = self.odom.pose.pose
        vx = self.odom.twist.twist.linear.x
        x = pose.position.x
        y = pose.position.y
        yaw = yaw_from_quaternion(pose.orientation)
        reference = self.nearest_reference(x, y)
        if self.handle_loop_stop(vx):
            return

        dx = x - reference["x"]
        dy = y - reference["y"]
        lateral_error = (
            -math.sin(reference["yaw"]) * dx
            + math.cos(reference["yaw"]) * dy
        )
        path_heading_error = normalize_angle(reference["yaw"] - yaw)
        feedforward_steer = self.preview_feedforward(reference, vx)
        if self.steering_mode in ("independent_4ws", "axle12_steer"):
            steer_command, rear_steer_command, curvature_cmd = (
                self.independent_steer_commands(
                    reference, lateral_error, path_heading_error, vx
                )
            )
            heading_error = path_heading_error
            feedforward_steer = curvature_cmd
        else:
            heading_correction = clamp(
                -self.lateral_pid.update(lateral_error, self.dt),
                -self.heading_correction_limit,
                self.heading_correction_limit,
            )
            desired_yaw = normalize_angle(reference["yaw"] + heading_correction)
            heading_error = normalize_angle(desired_yaw - yaw)
            target_steer = clamp(
                feedforward_steer
                + self.heading_pid.update(heading_error, self.dt),
                -self.steer_lim,
                self.steer_lim,
            )
            steer_step = self.steer_rate_lim * self.dt
            steer_command = clamp(
                target_steer,
                self.last_steer_cmd - steer_step,
                self.last_steer_cmd + steer_step,
            )
            rear_steer_command = steer_command * self.rear_steer_ratio()

        target_velocity = clamp(reference["speed"], 0.0, self.target_speed)
        loop_velocity_limit = self.loop_stop_velocity_limit(reference, vx)
        if loop_velocity_limit is not None:
            target_velocity = min(target_velocity, loop_velocity_limit)
        acceleration = clamp(
            self.speed_pid.update(target_velocity - vx, self.dt),
            -self.max_brake_decel,
            self.max_drive_accel,
        )
        accel_step = 0.7 * self.dt
        acceleration = clamp(
            acceleration,
            self.last_accel_cmd - accel_step,
            self.last_accel_cmd + accel_step,
        )
        self.publish_control(target_velocity, acceleration, steer_command)
        self.publish_two_axis_steer(
            steer_command,
            rear_steer_command,
            target_velocity,
            acceleration,
        )
        self.last_steer_cmd = steer_command
        self.last_rear_steer_cmd = rear_steer_command
        self.last_accel_cmd = acceleration

        if self.error_log_writer is not None:
            self.error_log_writer.writerow(
                [
                    self.get_clock().now().nanoseconds * 1e-9,
                    x,
                    y,
                    yaw,
                    vx,
                    reference["index"],
                    reference["x"],
                    reference["y"],
                    reference["yaw"],
                    target_velocity,
                    lateral_error,
                    heading_error,
                    feedforward_steer,
                    steer_command,
                    rear_steer_command,
                    acceleration,
                ]
            )
            self.error_log_file.flush()

    def destroy_node(self):
        if self.error_log_file is not None:
            self.error_log_file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PidTrajectoryController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
