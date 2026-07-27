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
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

from agv_mpc_controller.steering_geometry import front_two_axle_kinematics


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def stamp_to_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class SamplingMpcController(Node):
    def __init__(self):
        super().__init__("sampling_mpc_controller")
        self.declare_parameter("trajectory_topic", "/planning/scenario_planning/trajectory")
        self.declare_parameter("odom_topic", "/localization/kinematic_state")
        self.declare_parameter("steering_topic", "/vehicle/status/steering_status")
        self.declare_parameter("front_steer_topic", "/agv/status/front_steer")
        self.declare_parameter("rear_steer_topic", "/agv/status/rear_steer")
        self.declare_parameter("steering_mode_topic", "/agv/steering_mode_cmd")
        self.declare_parameter("control_topic", "/control/command/control_cmd")
        self.declare_parameter("two_axis_steer_topic", "/agv/control/two_axis_steer_cmd")
        self.declare_parameter("gear_topic", "/control/command/gear_cmd")
        self.declare_parameter("control_rate", 30.0)
        self.declare_parameter("prediction_dt", 0.2)
        self.declare_parameter("horizon_steps", 12)
        self.declare_parameter("target_speed", 4.0)
        self.declare_parameter("vx_lim", 5.0)
        self.declare_parameter("vx_rate_lim", 0.35)
        self.declare_parameter("steer_lim", 0.5229795759)
        self.declare_parameter("steer_rate_lim", 0.261799)
        self.declare_parameter("command_steer_rate_lim", 0.261799)
        self.declare_parameter("prediction_steer_rate_lim", 0.261799)
        self.declare_parameter("prediction_vx_time_constant", 0.2)
        self.declare_parameter("prediction_steer_time_constant", 0.05)
        self.declare_parameter("prediction_drive_delay", 0.05)
        self.declare_parameter("prediction_brake_delay", 0.05)
        self.declare_parameter("prediction_drive_time_constant", 0.18)
        self.declare_parameter("prediction_brake_time_constant", 0.18)
        self.declare_parameter("prediction_drive_accel_dead_band", 0.02)
        self.declare_parameter("prediction_brake_decel_dead_band", 0.05)
        self.declare_parameter("prediction_steer_delay", 0.02)
        self.declare_parameter("prediction_steer_backlash", 0.003491)
        self.declare_parameter("steer_lim_front", 0.5229795759)
        self.declare_parameter("steer_lim_rear", 0.4505604976)
        self.declare_parameter("wheelbase_f", 4.65)
        self.declare_parameter("wheelbase_r", 4.65)
        self.declare_parameter("axle2_x", 3.0)
        self.declare_parameter("axle3_x", -3.0)
        self.declare_parameter("axle4_x", -4.65)
        self.declare_parameter("wheel_tread", 2.381384)
        self.declare_parameter("ackermann_axle1_wheelbase", 6.116611371515338)
        self.declare_parameter("ackermann_axle2_wheelbase", 3.5527407651152627)
        self.declare_parameter("steering_mode", "axle12_steer")
        self.declare_parameter("two_axis_beta_limit", 0.22)
        self.declare_parameter("two_axis_lateral_gain", 0.12)
        self.declare_parameter("enable_lane_change_mode", False)
        self.declare_parameter("lane_change_steering_mode", "crab")
        self.declare_parameter("fixed_lane_change_windows", "111.5:133.0,239.5:261.0")
        self.declare_parameter("lane_change_preview_distance", 8.0)
        self.declare_parameter("lane_change_lateral_threshold", 0.8)
        self.declare_parameter("lane_change_exit_lateral_threshold", 0.4)
        self.declare_parameter("lane_change_heading_threshold", 0.6)
        self.declare_parameter("normal_lane_change_guard_distance", 12.0)
        self.declare_parameter("normal_lane_change_guard_epsilon", 0.5)
        self.declare_parameter("curvature_preview_distance", 3.0)
        self.declare_parameter("feedforward_steer_limit", 0.45)
        self.declare_parameter("crab_reference_yaw_lookback", 2.0)
        self.declare_parameter("crab_preview_time", 2.2)
        self.declare_parameter("stop_distance_to_end", 1.0)
        self.declare_parameter("loop_stop_duration", 3.0)
        self.declare_parameter("loop_stop_speed_threshold", 0.08)
        self.declare_parameter("enable_loop_stop", True)
        self.declare_parameter("stiction_speed_threshold", 0.15)
        self.declare_parameter("stiction_speed_error_threshold", 0.4)
        self.declare_parameter("stiction_accel", 0.2)
        self.declare_parameter("closed_trajectory", True)
        self.declare_parameter("enable_error_log", True)
        self.declare_parameter("error_log_path", "/tmp/agv_mpc_error.csv")

        self.prediction_dt = self.get_parameter("prediction_dt").value
        self.horizon_steps = int(self.get_parameter("horizon_steps").value)
        self.target_speed = self.get_parameter("target_speed").value
        self.vx_lim = self.get_parameter("vx_lim").value
        self.vx_rate_lim = self.get_parameter("vx_rate_lim").value
        self.steer_lim = self.get_parameter("steer_lim").value
        self.steer_rate_lim = self.get_parameter("steer_rate_lim").value
        self.command_steer_rate_lim = self.get_parameter("command_steer_rate_lim").value
        self.prediction_steer_rate_lim = self.get_parameter("prediction_steer_rate_lim").value
        self.prediction_vx_time_constant = max(
            1e-3, self.get_parameter("prediction_vx_time_constant").value
        )
        self.prediction_steer_time_constant = max(
            1e-3, self.get_parameter("prediction_steer_time_constant").value
        )
        self.prediction_drive_delay = max(
            0.0, self.get_parameter("prediction_drive_delay").value
        )
        self.prediction_brake_delay = max(
            0.0, self.get_parameter("prediction_brake_delay").value
        )
        self.prediction_drive_time_constant = max(
            1e-3, self.get_parameter("prediction_drive_time_constant").value
        )
        self.prediction_brake_time_constant = max(
            1e-3, self.get_parameter("prediction_brake_time_constant").value
        )
        self.prediction_drive_accel_dead_band = max(
            0.0, self.get_parameter("prediction_drive_accel_dead_band").value
        )
        self.prediction_brake_decel_dead_band = max(
            0.0, self.get_parameter("prediction_brake_decel_dead_band").value
        )
        self.prediction_steer_delay = max(
            0.0, self.get_parameter("prediction_steer_delay").value
        )
        self.prediction_steer_backlash = max(
            0.0, self.get_parameter("prediction_steer_backlash").value
        )
        self.steer_lim_front = self.get_parameter("steer_lim_front").value
        self.steer_lim_rear = self.get_parameter("steer_lim_rear").value
        self.wheelbase_f = self.get_parameter("wheelbase_f").value
        self.wheelbase_r = self.get_parameter("wheelbase_r").value
        self.axle2_x = self.get_parameter("axle2_x").value
        self.axle3_x = self.get_parameter("axle3_x").value
        self.axle4_x = self.get_parameter("axle4_x").value
        self.wheel_tread = self.get_parameter("wheel_tread").value
        self.ackermann_axle1_wheelbase = self.get_parameter(
            "ackermann_axle1_wheelbase"
        ).value
        self.ackermann_axle2_wheelbase = self.get_parameter(
            "ackermann_axle2_wheelbase"
        ).value
        self.normal_steering_mode = self.get_parameter("steering_mode").value
        self.two_axis_beta_limit = self.get_parameter("two_axis_beta_limit").value
        self.two_axis_lateral_gain = self.get_parameter("two_axis_lateral_gain").value
        self.enable_lane_change_mode = bool(self.get_parameter("enable_lane_change_mode").value)
        self.lane_change_steering_mode = self.get_parameter("lane_change_steering_mode").value
        self.fixed_lane_change_windows = self.parse_lane_change_windows(
            self.get_parameter("fixed_lane_change_windows").value
        )
        self.active_steering_mode = self.normal_steering_mode
        self.lane_change_preview_distance = self.get_parameter("lane_change_preview_distance").value
        self.lane_change_lateral_threshold = self.get_parameter("lane_change_lateral_threshold").value
        self.lane_change_exit_lateral_threshold = self.get_parameter(
            "lane_change_exit_lateral_threshold"
        ).value
        self.lane_change_heading_threshold = self.get_parameter("lane_change_heading_threshold").value
        self.normal_lane_change_guard_distance = self.get_parameter(
            "normal_lane_change_guard_distance"
        ).value
        self.normal_lane_change_guard_epsilon = self.get_parameter(
            "normal_lane_change_guard_epsilon"
        ).value
        self.curvature_preview_distance = self.get_parameter("curvature_preview_distance").value
        self.feedforward_steer_limit = self.get_parameter("feedforward_steer_limit").value
        self.crab_reference_yaw_lookback = self.get_parameter("crab_reference_yaw_lookback").value
        self.crab_preview_time = self.get_parameter("crab_preview_time").value
        self.stop_distance_to_end = self.get_parameter("stop_distance_to_end").value
        self.loop_stop_duration = max(0.0, self.get_parameter("loop_stop_duration").value)
        self.loop_stop_speed_threshold = max(
            0.0, self.get_parameter("loop_stop_speed_threshold").value
        )
        self.enable_loop_stop = bool(self.get_parameter("enable_loop_stop").value)
        self.stiction_speed_threshold = self.get_parameter("stiction_speed_threshold").value
        self.stiction_speed_error_threshold = self.get_parameter(
            "stiction_speed_error_threshold"
        ).value
        self.stiction_accel = self.get_parameter("stiction_accel").value
        self.closed_trajectory = bool(self.get_parameter("closed_trajectory").value)
        self.enable_error_log = bool(self.get_parameter("enable_error_log").value)
        self.error_log_path = self.get_parameter("error_log_path").value

        self.trajectory = []
        self.trajectory_s = []
        self.trajectory_length = 0.0
        self.odom = None
        self.current_steer = 0.0
        self.current_front_steer = 0.0
        self.current_rear_steer = 0.0
        self.current_ax = 0.0
        self.previous_odom_vx = None
        self.previous_odom_time = None
        self.last_steer_cmd = 0.0
        self.last_accel = 0.0
        self.last_ref_index = None
        self.loop_stop_state = None
        self.loop_stop_until = None
        self.loop_stop_armed = True
        self.error_log_file = None
        self.error_log_writer = None
        self.active_lane_change_window = None

        if self.enable_error_log:
            os.makedirs(os.path.dirname(self.error_log_path) or ".", exist_ok=True)
            self.error_log_file = open(self.error_log_path, "w", newline="", encoding="utf-8")
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
                    "ref_curvature",
                    "lateral_error",
                    "longitudinal_error",
                    "yaw_error",
                    "speed_error",
                    "current_steer",
                    "feedforward_steer",
                    "ref_steer",
                    "target_steer",
                    "command_steer",
                    "accel",
                    "command_velocity",
                    "best_cost",
                    "preview_distance",
                    "preview_index",
                    "steering_mode",
                    "lane_change_lateral_shift",
                    "lane_change_heading_delta",
                ]
            )
            self.get_logger().info("Logging MPC tracking errors to %s" % self.error_log_path)

        self.create_subscription(Trajectory, self.get_parameter("trajectory_topic").value, self.on_traj, 1)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.on_odom, 1)
        self.create_subscription(
            SteeringReport, self.get_parameter("steering_topic").value, self.on_steering, 1
        )
        self.create_subscription(
            Float32, self.get_parameter("front_steer_topic").value, self.on_front_steer, 1
        )
        self.create_subscription(
            Float32, self.get_parameter("rear_steer_topic").value, self.on_rear_steer, 1
        )
        self.control_pub = self.create_publisher(Control, self.get_parameter("control_topic").value, 1)
        self.two_axis_steer_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter("two_axis_steer_topic").value, 1
        )
        self.gear_pub = self.create_publisher(GearCommand, self.get_parameter("gear_topic").value, 1)
        self.steering_mode_pub = self.create_publisher(
            String, self.get_parameter("steering_mode_topic").value, 1
        )

        control_rate = self.get_parameter("control_rate").value
        self.timer = self.create_timer(1.0 / max(1.0, control_rate), self.on_timer)
        self.get_logger().info(
            "Sampling MPC started: horizon=%d dt=%.2f steering_mode=%s lane_change_mode=%s"
            % (
                self.horizon_steps,
                self.prediction_dt,
                self.normal_steering_mode,
                self.lane_change_steering_mode,
            )
        )

    def parse_lane_change_windows(self, value):
        windows = []
        for item in str(value).split(","):
            item = item.strip()
            if not item:
                continue
            start, end = item.split(":", 1)
            windows.append((float(start), float(end)))
        return windows

    def on_traj(self, msg):
        trajectory = []
        for point in msg.points:
            x = point.pose.position.x
            y = point.pose.position.y
            yaw = yaw_from_quaternion(point.pose.orientation)
            speed = point.longitudinal_velocity_mps
            trajectory.append((x, y, yaw, speed, 0.0))
        curvatures = [0.0 for _ in trajectory]
        for i in range(1, len(trajectory) - 1):
            prev_x, prev_y, prev_yaw, _, _ = trajectory[i - 1]
            next_x, next_y, next_yaw, _, _ = trajectory[i + 1]
            arc = max(1e-3, math.hypot(next_x - prev_x, next_y - prev_y))
            curvatures[i] = normalize_angle(next_yaw - prev_yaw) / arc
        if len(trajectory) > 1:
            curvatures[0] = curvatures[1]
            curvatures[-1] = curvatures[-2]

        smoothed_curvatures = curvatures[:]
        for _ in range(2):
            for i in range(1, len(curvatures) - 1):
                smoothed_curvatures[i] = (
                    0.25 * curvatures[i - 1] + 0.5 * curvatures[i] + 0.25 * curvatures[i + 1]
                )
            curvatures = smoothed_curvatures[:]

        self.trajectory = [
            (x, y, yaw, speed, curvatures[i]) for i, (x, y, yaw, speed, _) in enumerate(trajectory)
        ]
        self.trajectory_s = [0.0]
        for i in range(1, len(self.trajectory)):
            x0, y0, *_ = self.trajectory[i - 1]
            x1, y1, *_ = self.trajectory[i]
            self.trajectory_s.append(self.trajectory_s[-1] + math.hypot(x1 - x0, y1 - y0))
        self.trajectory_length = self.trajectory_s[-1] if self.trajectory_s else 0.0

    def on_odom(self, msg):
        self.odom = msg
        vx = msg.twist.twist.linear.x
        stamp = stamp_to_seconds(msg.header.stamp)
        if self.previous_odom_vx is not None and self.previous_odom_time is not None:
            dt = stamp - self.previous_odom_time
            if dt > 1e-4:
                measured_ax = clamp(
                    (vx - self.previous_odom_vx) / dt,
                    -self.vx_rate_lim,
                    self.vx_rate_lim,
                )
                self.current_ax = 0.8 * self.current_ax + 0.2 * measured_ax
        self.previous_odom_vx = vx
        self.previous_odom_time = stamp

    def on_steering(self, msg):
        self.current_steer = msg.steering_tire_angle
        self.current_front_steer = msg.steering_tire_angle

    def on_front_steer(self, msg):
        self.current_front_steer = msg.data
        self.current_steer = msg.data

    def on_rear_steer(self, msg):
        self.current_rear_steer = msg.data

    def nearest_index(self, x, y):
        if not self.trajectory:
            return 0
        return min(
            range(len(self.trajectory)),
            key=lambda i: (self.trajectory[i][0] - x) ** 2 + (self.trajectory[i][1] - y) ** 2,
        )

    def sample_reference(self, target_s):
        if not self.trajectory:
            return None
        if len(self.trajectory) == 1:
            x, y, yaw, speed, curvature = self.trajectory[0]
            return {
                "index": 0,
                "s": 0.0,
                "x": x,
                "y": y,
                "yaw": yaw,
                "speed": speed,
                "curvature": curvature,
            }

        if self.closed_trajectory and self.trajectory_length > 1e-6:
            target_s = target_s % self.trajectory_length
        else:
            target_s = clamp(target_s, self.trajectory_s[0], self.trajectory_s[-1])
        index = 0
        while index + 1 < len(self.trajectory_s) and self.trajectory_s[index + 1] < target_s:
            index += 1
        if index + 1 >= len(self.trajectory):
            index = len(self.trajectory) - 2

        s0 = self.trajectory_s[index]
        s1 = self.trajectory_s[index + 1]
        ratio = 0.0 if s1 <= s0 else (target_s - s0) / (s1 - s0)
        x0, y0, yaw0, speed0, curvature0 = self.trajectory[index]
        x1, y1, yaw1, speed1, curvature1 = self.trajectory[index + 1]
        yaw = normalize_angle(yaw0 + normalize_angle(yaw1 - yaw0) * ratio)
        return {
            "index": index,
            "s": target_s,
            "x": x0 + (x1 - x0) * ratio,
            "y": y0 + (y1 - y0) * ratio,
            "yaw": yaw,
            "speed": speed0 + (speed1 - speed0) * ratio,
            "curvature": curvature0 + (curvature1 - curvature0) * ratio,
        }

    def nearest_reference(self, x, y, start_index=0, search_segments=None):
        if len(self.trajectory) < 2:
            return self.sample_reference(0.0)

        segment_count = len(self.trajectory) - 1
        if self.closed_trajectory and self.trajectory_length > 1e-6:
            first = start_index % segment_count
            count = segment_count if search_segments is None else min(segment_count, max(1, search_segments))
            segment_indices = [(first + offset) % segment_count for offset in range(count)]
        else:
            first = max(0, start_index)
            last = len(self.trajectory) - 1
            if search_segments is not None:
                last = min(last, first + search_segments)
            if first >= last:
                first = max(0, last - 1)
            segment_indices = range(first, last)

        best_dist2 = float("inf")
        best_s = self.trajectory_s[segment_indices[0]]
        for i in segment_indices:
            x0, y0, *_ = self.trajectory[i]
            x1, y1, *_ = self.trajectory[i + 1]
            seg_x = x1 - x0
            seg_y = y1 - y0
            seg_len2 = seg_x * seg_x + seg_y * seg_y
            if seg_len2 <= 1e-9:
                ratio = 0.0
            else:
                ratio = clamp(((x - x0) * seg_x + (y - y0) * seg_y) / seg_len2, 0.0, 1.0)
            proj_x = x0 + seg_x * ratio
            proj_y = y0 + seg_y * ratio
            dist2 = (x - proj_x) ** 2 + (y - proj_y) ** 2
            if dist2 < best_dist2:
                best_dist2 = dist2
                best_s = self.trajectory_s[i] + math.sqrt(seg_len2) * ratio
        return self.sample_reference(best_s)

    def current_reference(self, x, y):
        previous_index = self.last_ref_index
        if self.last_ref_index is None:
            reference = self.nearest_reference(x, y)
        else:
            search_start = self.last_ref_index - 8
            reference = self.nearest_reference(x, y, search_start, 90)
        self.last_ref_index = reference["index"]
        self.update_loop_stop_trigger(previous_index, reference["index"])
        return reference

    def update_loop_stop_trigger(self, previous_index, current_index):
        if (
            not self.enable_loop_stop
            or not self.closed_trajectory
            or self.loop_stop_duration <= 0.0
            or self.loop_stop_state is not None
            or len(self.trajectory) < 4
        ):
            return

        segment_count = len(self.trajectory) - 1
        if segment_count <= 0:
            return

        rearm_index = max(20, int(0.2 * segment_count))
        if not self.loop_stop_armed:
            if current_index >= rearm_index:
                self.loop_stop_armed = True
            return

        if previous_index is None:
            return

        near_end_index = max(1, int(0.85 * segment_count))
        near_start_index = max(1, int(0.15 * segment_count))
        crossed_loop_seam = previous_index >= near_end_index and current_index <= near_start_index
        if crossed_loop_seam:
            self.loop_stop_state = "braking"
            self.loop_stop_until = None
            self.loop_stop_armed = False
            self.last_accel = 0.0
            self.last_steer_cmd = 0.0
            self.get_logger().info(
                "Completed closed trajectory loop at index %d -> %d; stopping for %.1f s"
                % (previous_index, current_index, self.loop_stop_duration)
            )

    def handle_loop_stop(self, vx):
        if self.loop_stop_state is None:
            return False

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.loop_stop_state == "braking":
            self.publish_stop()
            self.last_accel = 0.0
            self.last_steer_cmd = 0.0
            if abs(vx) <= self.loop_stop_speed_threshold:
                self.loop_stop_state = "holding"
                self.loop_stop_until = now + self.loop_stop_duration
                self.get_logger().info("Loop stop reached zero speed; holding for %.1f s" % self.loop_stop_duration)
            return True

        if self.loop_stop_state == "holding":
            if self.loop_stop_until is not None and now >= self.loop_stop_until:
                self.loop_stop_state = None
                self.loop_stop_until = None
                self.last_ref_index = 0
                self.get_logger().info("Loop stop complete; resuming trajectory")
                return False
            self.publish_stop()
            self.last_accel = 0.0
            self.last_steer_cmd = 0.0
            return True

        self.loop_stop_state = None
        self.loop_stop_until = None
        return False

    def two_axis_beta_command(self, x, y, yaw, reference):
        if self.active_steering_mode not in ("independent_4ws", "axle12_steer"):
            return 0.0
        dx = x - reference["x"]
        dy = y - reference["y"]
        lateral_error = -math.sin(reference["yaw"]) * dx + math.cos(reference["yaw"]) * dy
        heading_error = normalize_angle(reference["yaw"] - yaw)
        return clamp(
            heading_error - self.two_axis_lateral_gain * lateral_error,
            -self.two_axis_beta_limit,
            self.two_axis_beta_limit,
        )

    def split_steer(self, steer, beta_cmd=0.0):
        if self.active_steering_mode == "front_only":
            steer_f, steer_r = steer, 0.0
        elif self.active_steering_mode == "crab":
            steer_f, steer_r = steer, steer
        elif self.active_steering_mode == "axle12_steer":
            curvature_cmd = math.tan(steer) / max(1e-3, self.wheelbase_f)
            beta_tan = math.tan(beta_cmd)
            steer_f = math.atan(beta_tan + self.wheelbase_f * curvature_cmd)
            steer_r = math.atan(beta_tan + self.axle2_x * curvature_cmd)
        elif self.active_steering_mode == "independent_4ws":
            curvature_cmd = math.tan(steer) / max(1e-3, self.wheelbase_f)
            steer_f = math.atan(beta_cmd + self.wheelbase_f * curvature_cmd)
            steer_r = math.atan(beta_cmd - self.wheelbase_r * curvature_cmd)
        elif self.active_steering_mode == "asymmetric_4ws":
            steer_f, steer_r = steer, -steer / 2.0
        else:
            steer_f, steer_r = steer, -steer
        return (
            clamp(steer_f, -self.steer_lim_front, self.steer_lim_front),
            clamp(steer_r, -self.steer_lim_rear, self.steer_lim_rear),
        )

    def axle12_kinematics(self, vx, steer_f, steer_r):
        axle_positions = [self.wheelbase_f, self.axle2_x, self.axle3_x, self.axle4_x]
        return front_two_axle_kinematics(
            vx,
            steer_f,
            steer_r,
            axle_positions,
            self.wheel_tread,
            self.ackermann_axle1_wheelbase,
            self.ackermann_axle2_wheelbase,
        )

    def lane_change_metrics(self, reference):
        future = self.sample_reference(reference["s"] + self.lane_change_preview_distance)
        dx = future["x"] - reference["x"]
        dy = future["y"] - reference["y"]
        lateral_shift = -math.sin(reference["yaw"]) * dx + math.cos(reference["yaw"]) * dy
        heading_delta = normalize_angle(future["yaw"] - reference["yaw"])
        return lateral_shift, heading_delta

    def select_steering_mode(self, reference):
        lateral_shift, heading_delta = self.lane_change_metrics(reference)
        self.active_lane_change_window = self.lane_change_window(reference["s"])
        in_lane_change_window = self.active_lane_change_window is not None

        next_mode = (
            self.lane_change_steering_mode if in_lane_change_window else self.normal_steering_mode
        )

        if next_mode != self.active_steering_mode:
            self.active_steering_mode = next_mode
            msg = String()
            msg.data = self.active_steering_mode
            self.steering_mode_pub.publish(msg)
            self.get_logger().info(
                "Changed MPC/vehicle steering_mode to %s at s=%.1f: lateral_shift=%.2f heading_delta=%.1f deg"
                % (
                    self.active_steering_mode,
                    reference["s"],
                    lateral_shift,
                    math.degrees(heading_delta),
                )
            )
        return lateral_shift, heading_delta

    def lane_change_window(self, reference_s):
        if not self.enable_lane_change_mode:
            return None
        for start, end in self.fixed_lane_change_windows:
            if start <= reference_s <= end:
                return start, end
        return None

    def upcoming_lane_change_window(self, reference_s):
        if not self.enable_lane_change_mode:
            return None
        guard_distance = max(0.0, self.normal_lane_change_guard_distance)
        for start, end in self.fixed_lane_change_windows:
            distance_to_start = start - reference_s
            if 0.0 < distance_to_start <= guard_distance:
                return start, end
        return None

    def normal_mode_reference(self, reference, guard_window):
        if guard_window is None:
            return reference
        window_start, _ = guard_window
        if reference["s"] < window_start:
            return reference

        anchor_s = max(
            self.trajectory_s[0],
            window_start - max(0.0, self.normal_lane_change_guard_epsilon),
        )
        anchor = self.sample_reference(anchor_s)
        road_yaw = anchor["yaw"]
        ds = reference["s"] - anchor["s"]
        guarded = reference.copy()
        guarded["x"] = anchor["x"] + math.cos(road_yaw) * ds
        guarded["y"] = anchor["y"] + math.sin(road_yaw) * ds
        guarded["yaw"] = road_yaw
        guarded["curvature"] = 0.0
        return guarded

    def crab_reference_yaw(self):
        if self.active_lane_change_window is None:
            return None
        window_start, _ = self.active_lane_change_window
        yaw_reference = self.sample_reference(window_start - max(0.0, self.crab_reference_yaw_lookback))
        return yaw_reference["yaw"]

    def curvature_from_steer(self, steer):
        steer_f, steer_r = self.split_steer(steer)
        if self.active_steering_mode == "axle12_steer":
            _, omega = self.axle12_kinematics(1.0, steer_f, steer_r)
            return omega
        beta_num = self.wheelbase_f * math.tan(steer_r) + self.wheelbase_r * math.tan(steer_f)
        beta_den = self.wheelbase_f + self.wheelbase_r
        kappa_num = math.tan(steer_f) - math.tan(steer_r)
        kappa_den = math.sqrt(beta_num * beta_num + beta_den * beta_den)
        return kappa_num / max(1e-6, kappa_den)

    def steer_from_curvature(self, curvature):
        if self.active_steering_mode == "crab":
            return 0.0
        limit = min(self.steer_lim, self.feedforward_steer_limit)
        if abs(curvature) < 1e-5:
            return 0.0
        sign = 1.0 if curvature > 0.0 else -1.0
        lo = 0.0
        hi = limit
        target = abs(curvature)
        for _ in range(24):
            mid = 0.5 * (lo + hi)
            value = abs(self.curvature_from_steer(sign * mid))
            if value < target:
                lo = mid
            else:
                hi = mid
        return sign * 0.5 * (lo + hi)

    def preview_feedforward_steer(self, start_s, preview_distance):
        sample_count = 8
        best_steer = 0.0
        for sample_index in range(sample_count + 1):
            ratio = sample_index / sample_count
            reference = self.sample_reference(start_s + preview_distance * ratio)
            candidate = self.steer_from_curvature(reference["curvature"])
            if abs(candidate) > abs(best_steer):
                best_steer = candidate
        return best_steer

    def tracking_errors(self, x, y, yaw, vx, reference, ref_yaw_override=None, ref_steer_override=None):
        ref_x = reference["x"]
        ref_y = reference["y"]
        ref_yaw = reference["yaw"] if ref_yaw_override is None else ref_yaw_override
        ref_speed = reference["speed"]
        ref_curvature = reference["curvature"]
        dx = x - ref_x
        dy = y - ref_y
        longitudinal_error = math.cos(ref_yaw) * dx + math.sin(ref_yaw) * dy
        lateral_error = -math.sin(ref_yaw) * dx + math.cos(ref_yaw) * dy
        yaw_error = normalize_angle(ref_yaw - yaw)
        speed_target = min(max(0.0, ref_speed), self.target_speed)
        speed_error = speed_target - vx
        ref_steer = self.steer_from_curvature(ref_curvature) if ref_steer_override is None else ref_steer_override
        return {
            "ref_x": ref_x,
            "ref_y": ref_y,
            "ref_yaw": ref_yaw,
            "ref_speed": ref_speed,
            "ref_curvature": ref_curvature,
            "lateral_error": lateral_error,
            "longitudinal_error": longitudinal_error,
            "yaw_error": yaw_error,
            "speed_error": speed_error,
            "ref_steer": ref_steer,
        }

    def step_model(self, state, accel, target_steer, elapsed, command_reference=None):
        x, y, yaw, vx, ax, steer_f, steer_r = state
        dt = self.prediction_dt

        def delayed_value(previous_value, requested_value, delay):
            step_start = elapsed - dt
            if elapsed <= delay:
                return previous_value
            if step_start >= delay:
                return requested_value
            active_ratio = clamp((elapsed - delay) / dt, 0.0, 1.0)
            return previous_value + active_ratio * (requested_value - previous_value)

        requested_accel = clamp(accel, -self.vx_rate_lim, self.vx_rate_lim)
        if 0.0 < requested_accel < self.prediction_drive_accel_dead_band:
            requested_accel = 0.0
        elif -self.prediction_brake_decel_dead_band < requested_accel < 0.0:
            requested_accel = 0.0
        accel_delay = (
            self.prediction_drive_delay
            if requested_accel >= ax
            else self.prediction_brake_delay
        )
        delayed_accel = delayed_value(self.last_accel, requested_accel, accel_delay)
        accel_time_constant = (
            self.prediction_drive_time_constant
            if delayed_accel >= ax
            else self.prediction_brake_time_constant
        )
        ax_rate = (delayed_accel - ax) / accel_time_constant
        ax = clamp(
            ax + ax_rate * dt,
            -self.vx_rate_lim,
            self.vx_rate_lim,
        )
        vx = clamp(vx + ax * dt, -self.vx_lim, self.vx_lim)

        delayed_steer = delayed_value(
            self.last_steer_cmd, target_steer, self.prediction_steer_delay
        )
        steer_delta = delayed_steer - self.last_steer_cmd
        if abs(steer_delta) <= self.prediction_steer_backlash:
            delayed_steer = self.last_steer_cmd
        else:
            delayed_steer -= math.copysign(self.prediction_steer_backlash, steer_delta)
        beta_cmd = 0.0
        if command_reference is not None:
            beta_cmd = self.two_axis_beta_command(x, y, yaw, command_reference)
        steer_f_des, steer_r_des = self.split_steer(
            clamp(delayed_steer, -self.steer_lim_front, self.steer_lim_front),
            beta_cmd,
        )
        steer_f_rate = clamp(
            (steer_f_des - steer_f) / self.prediction_steer_time_constant,
            -self.prediction_steer_rate_lim,
            self.prediction_steer_rate_lim,
        )
        steer_r_rate = clamp(
            (steer_r_des - steer_r) / self.prediction_steer_time_constant,
            -self.prediction_steer_rate_lim,
            self.prediction_steer_rate_lim,
        )
        steer_f = clamp(
            steer_f + steer_f_rate * dt, -self.steer_lim_front, self.steer_lim_front
        )
        steer_r = clamp(
            steer_r + steer_r_rate * dt, -self.steer_lim_rear, self.steer_lim_rear
        )

        if self.active_steering_mode == "axle12_steer":
            vy, omega = self.axle12_kinematics(vx, steer_f, steer_r)
        else:
            beta_num = self.wheelbase_f * math.tan(steer_r) + self.wheelbase_r * math.tan(steer_f)
            beta_den = self.wheelbase_f + self.wheelbase_r
            beta = math.atan2(beta_num, beta_den)
            kappa_num = math.tan(steer_f) - math.tan(steer_r)
            kappa_den = math.sqrt(beta_num * beta_num + beta_den * beta_den)
            kappa = kappa_num / max(1e-6, kappa_den)
            vy = vx * beta
            speed = vx * math.sqrt(1.0 + beta * beta)
            omega = speed * kappa

        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        yaw = normalize_angle(yaw + omega * dt)
        return x, y, yaw, vx, ax, steer_f, steer_r

    def rollout_cost(self, initial_state, accel, target_steer, start_reference, guard_window=None):
        state = initial_state
        cost = 0.0
        ref_index = start_reference["index"]
        for step in range(1, self.horizon_steps + 1):
            command_reference = self.nearest_reference(*state[:2], ref_index, 30)
            command_reference = self.normal_mode_reference(command_reference, guard_window)
            state = self.step_model(
                state, accel, target_steer, step * self.prediction_dt, command_reference
            )
            x, y, yaw, vx, ax, steer_f, steer_r = state

            reference = self.nearest_reference(x, y, ref_index, 30)
            ref_index = reference["index"]
            reference = self.normal_mode_reference(reference, guard_window)
            errors = self.tracking_errors(x, y, yaw, vx, reference)
            lateral_error = errors["lateral_error"]
            longitudinal_error = errors["longitudinal_error"]
            yaw_error = errors["yaw_error"]
            speed_error = errors["speed_error"]
            ref_steer = errors["ref_steer"]

            cost += 20.0 * lateral_error * lateral_error
            cost += 0.2 * longitudinal_error * longitudinal_error
            cost += 9.0 * yaw_error * yaw_error
            cost += 3.0 * speed_error * speed_error
            steer_energy = 0.5 * (steer_f * steer_f + steer_r * steer_r)
            cost += 2.2 * steer_energy
            cost += 0.65 * steer_energy * vx * vx
            cost += 0.08 * accel * accel
            beta_cmd = self.two_axis_beta_command(x, y, yaw, reference)
            target_steer_f, target_steer_r = self.split_steer(target_steer, beta_cmd)
            steer_tracking = 0.5 * (
                (target_steer_f - steer_f) * (target_steer_f - steer_f)
                + (target_steer_r - steer_r) * (target_steer_r - steer_r)
            )
            cost += 2.8 * steer_tracking
            cost += 2.8 * (target_steer - ref_steer) * (target_steer - ref_steer)
            if abs(ref_steer) > 0.03 and target_steer * ref_steer < 0.0:
                cost += 4.0 * target_steer * target_steer
            if target_steer * self.last_steer_cmd < 0.0:
                cost += 5.0 * (target_steer - self.last_steer_cmd) ** 2
                cost += 1.5 * target_steer * target_steer
            cost += 0.01 * step
        cost += 0.2 * (accel - self.last_accel) ** 2
        cost += 6.0 * (target_steer - self.last_steer_cmd) ** 2
        return cost

    def rollout_cost_crab(self, initial_state, accel, target_steer, start_reference, road_yaw):
        state = initial_state
        cost = 0.0
        for step in range(1, self.horizon_steps + 1):
            state = self.step_model(
                state, accel, target_steer, step * self.prediction_dt
            )
            x, y, yaw, vx, ax, steer_f, steer_r = state

            preview_time = min(max(0.1, self.crab_preview_time), self.prediction_dt * step)
            target_s = start_reference["s"] + max(0.2, abs(vx)) * preview_time
            reference = self.sample_reference(target_s)
            errors = self.tracking_errors(
                x, y, yaw, vx, reference, ref_yaw_override=road_yaw, ref_steer_override=0.0
            )
            lateral_error = errors["lateral_error"]
            longitudinal_error = errors["longitudinal_error"]
            yaw_error = errors["yaw_error"]
            speed_error = errors["speed_error"]

            cost += 28.0 * lateral_error * lateral_error
            cost += 0.4 * longitudinal_error * longitudinal_error
            cost += 12.0 * yaw_error * yaw_error
            cost += 0.6 * speed_error * speed_error
            cost += 0.35 * (steer_f * steer_f + steer_r * steer_r)
            cost += 0.05 * accel * accel
            cost += 0.7 * (
                (target_steer - steer_f) * (target_steer - steer_f)
                + (target_steer - steer_r) * (target_steer - steer_r)
            )
            cost += 0.01 * step
        cost += 0.2 * (accel - self.last_accel) ** 2
        cost += 2.0 * (target_steer - self.last_steer_cmd) ** 2
        return cost

    def solve_normal_mpc(self, initial_state, start_reference, feedforward_steer, guard_window=None):
        accel_candidates = [
            -self.vx_rate_lim,
            -0.5 * self.vx_rate_lim,
            0.0,
            0.5 * self.vx_rate_lim,
            self.vx_rate_lim,
        ]
        x, y, yaw, vx, _, _, _ = initial_state
        errors = self.tracking_errors(x, y, yaw, vx, start_reference)
        feedback_steer = clamp(
            -0.10 * errors["lateral_error"] + 0.45 * errors["yaw_error"],
            -0.18,
            0.18,
        )
        steer_center = clamp(
            feedforward_steer + feedback_steer,
            -self.steer_lim,
            self.steer_lim,
        )
        if abs(feedforward_steer) > 0.05:
            steer_offsets = [-0.16, -0.12, -0.08, -0.04, 0.0, 0.04, 0.08, 0.12, 0.16]
        else:
            steer_offsets = [-0.10, -0.07, -0.04, -0.02, 0.0, 0.02, 0.04, 0.07, 0.10]

        best_cost = float("inf")
        best = (0.0, 0.0)
        for accel in accel_candidates:
            for steer_offset in steer_offsets:
                target_steer = clamp(steer_center + steer_offset, -self.steer_lim, self.steer_lim)
                if abs(feedforward_steer) > 0.05 and target_steer * feedforward_steer < 0.0:
                    continue
                cost = self.rollout_cost(
                    initial_state, accel, target_steer, start_reference, guard_window
                )
                if cost < best_cost:
                    best_cost = cost
                    best = (accel, target_steer)
        return best[0], best[1], best_cost

    def solve_crab_mpc(self, initial_state, start_reference):
        road_yaw = self.crab_reference_yaw()
        if road_yaw is None:
            road_yaw = start_reference["yaw"]

        accel_candidates = [
            -0.5 * self.vx_rate_lim,
            0.0,
            0.5 * self.vx_rate_lim,
            self.vx_rate_lim,
        ]
        steer_candidates = [
            -0.45,
            -0.35,
            -0.25,
            -0.16,
            -0.08,
            0.0,
            0.08,
            0.16,
            0.25,
            0.35,
            0.45,
        ]

        best_cost = float("inf")
        best = (0.0, 0.0)
        for accel in accel_candidates:
            for target_steer in steer_candidates:
                cost = self.rollout_cost_crab(initial_state, accel, target_steer, start_reference, road_yaw)
                if cost < best_cost:
                    best_cost = cost
                    best = (accel, target_steer)
        return best[0], best[1], best_cost

    def publish_stop(self):
        stamp = self.get_clock().now().to_msg()
        current_vx = 0.0
        if self.odom is not None:
            current_vx = self.odom.twist.twist.linear.x
        if current_vx > 0.05:
            stop_accel = -self.vx_rate_lim
        elif current_vx < -0.05:
            stop_accel = self.vx_rate_lim
        else:
            stop_accel = 0.0
        cmd = Control()
        cmd.stamp = stamp
        cmd.longitudinal.stamp = stamp
        cmd.longitudinal.velocity = 0.0
        cmd.longitudinal.acceleration = float(stop_accel)
        cmd.longitudinal.is_defined_acceleration = True
        cmd.lateral.stamp = stamp
        cmd.lateral.steering_tire_angle = float(clamp(self.current_steer, -self.steer_lim, self.steer_lim))
        cmd.lateral.steering_tire_rotation_rate = 0.0
        cmd.lateral.is_defined_steering_tire_rotation_rate = True
        self.control_pub.publish(cmd)
        self.publish_two_axis_steer(self.current_front_steer, self.current_rear_steer, 0.0, stop_accel)

    def publish_two_axis_steer(self, front_steer, rear_steer, velocity, acceleration):
        if self.active_steering_mode not in ("independent_4ws", "axle12_steer"):
            return
        msg = Float32MultiArray()
        msg.data = [
            float(clamp(front_steer, -self.steer_lim_front, self.steer_lim_front)),
            float(clamp(rear_steer, -self.steer_lim_rear, self.steer_lim_rear)),
            float(clamp(velocity, -self.vx_lim, self.vx_lim)),
            float(clamp(acceleration, -self.vx_rate_lim, self.vx_rate_lim)),
        ]
        self.two_axis_steer_pub.publish(msg)

    def on_timer(self):
        if self.odom is None or len(self.trajectory) < 2:
            self.publish_stop()
            return

        pose = self.odom.pose.pose
        twist = self.odom.twist.twist
        x = pose.position.x
        y = pose.position.y
        yaw = yaw_from_quaternion(pose.orientation)
        vx = twist.linear.x
        start_reference = self.current_reference(x, y)
        if self.handle_loop_stop(vx):
            return
        lane_change_lateral_shift, lane_change_heading_delta = self.select_steering_mode(
            start_reference
        )
        guard_window = None
        if self.active_steering_mode != "crab":
            guard_window = self.upcoming_lane_change_window(start_reference["s"])
        preview_distance = max(self.curvature_preview_distance, abs(vx) * 1.2)
        preview_reference = self.sample_reference(start_reference["s"] + preview_distance)
        preview_index = preview_reference["index"]
        normal_start_reference = self.normal_mode_reference(start_reference, guard_window)
        normal_preview_reference = self.normal_mode_reference(preview_reference, guard_window)
        current_ref_steer = self.steer_from_curvature(normal_start_reference["curvature"])
        feedforward_steer = self.preview_feedforward_steer(
            normal_start_reference["s"], preview_distance
        )
        if abs(current_ref_steer) > abs(feedforward_steer):
            feedforward_steer = current_ref_steer

        if not self.closed_trajectory and len(self.trajectory) - start_reference["index"] < 3:
            self.publish_stop()
            return

        initial_state = (
            x,
            y,
            yaw,
            vx,
            self.current_ax,
            self.current_front_steer,
            self.current_rear_steer,
        )
        if self.active_steering_mode == "crab":
            feedforward_steer = 0.0
            accel, target_steer, best_cost = self.solve_crab_mpc(initial_state, start_reference)
        else:
            accel, target_steer, best_cost = self.solve_normal_mpc(
                initial_state, start_reference, feedforward_steer, guard_window
            )

        control_rate = max(1.0, self.get_parameter("control_rate").value)
        speed_target = min(max(0.0, start_reference["speed"]), self.target_speed)
        speed_error = speed_target - vx
        current_errors = self.tracking_errors(x, y, yaw, vx, start_reference)
        if (
            abs(vx) < self.stiction_speed_threshold
            and speed_error > self.stiction_speed_error_threshold
            and accel <= 0.0
        ):
            accel = min(self.vx_rate_lim, max(0.0, self.stiction_accel))
        elif (
            speed_error > 0.6
            and abs(current_errors["lateral_error"]) < 1.5
            and accel <= 0.0
        ):
            accel = min(self.vx_rate_lim, 0.5 * speed_error)

        max_steer_step = self.command_steer_rate_lim / control_rate
        next_steer = clamp(
            self.last_steer_cmd + clamp(target_steer - self.last_steer_cmd, -max_steer_step, max_steer_step),
            -0.55,
            0.55,
        )
        next_velocity = clamp(vx + accel / control_rate, -self.vx_lim, self.vx_lim)
        if next_velocity < 0.0:
            next_velocity = 0.0

        stamp = self.get_clock().now().to_msg()
        control = Control()
        control.stamp = stamp
        control.longitudinal.stamp = stamp
        control.longitudinal.velocity = float(next_velocity)
        control.longitudinal.acceleration = float(accel)
        control.longitudinal.is_defined_acceleration = True
        control.lateral.stamp = stamp
        control.lateral.steering_tire_angle = float(next_steer)
        control.lateral.steering_tire_rotation_rate = float(
            clamp((next_steer - self.last_steer_cmd) * control_rate, -self.command_steer_rate_lim, self.command_steer_rate_lim)
        )
        control.lateral.is_defined_steering_tire_rotation_rate = True
        self.control_pub.publish(control)
        beta_cmd = self.two_axis_beta_command(x, y, yaw, start_reference)
        front_steer, rear_steer = self.split_steer(next_steer, beta_cmd)
        self.publish_two_axis_steer(front_steer, rear_steer, next_velocity, accel)

        gear = GearCommand()
        gear.stamp = stamp
        gear.command = GearCommand.DRIVE
        self.gear_pub.publish(gear)
        self.write_error_log(
            stamp,
            x,
            y,
            yaw,
            vx,
            start_reference,
            preview_reference,
            preview_distance,
            feedforward_steer,
            target_steer,
            next_steer,
            accel,
            next_velocity,
            best_cost,
            lane_change_lateral_shift,
            lane_change_heading_delta,
        )
        self.last_accel = accel
        self.last_steer_cmd = next_steer

    def write_error_log(
        self,
        stamp,
        x,
        y,
        yaw,
        vx,
        reference,
        preview_reference,
        preview_distance,
        feedforward_steer,
        target_steer,
        command_steer,
        accel,
        command_velocity,
        best_cost,
        lane_change_lateral_shift,
        lane_change_heading_delta,
    ):
        if self.error_log_writer is None:
            return
        ref_yaw_override = self.crab_reference_yaw() if self.active_steering_mode == "crab" else None
        ref_steer_override = 0.0 if self.active_steering_mode == "crab" else None
        errors = self.tracking_errors(
            x, y, yaw, vx, reference, ref_yaw_override=ref_yaw_override,
            ref_steer_override=ref_steer_override
        )
        self.error_log_writer.writerow(
            [
                "%.9f" % stamp_to_seconds(stamp),
                "%.6f" % x,
                "%.6f" % y,
                "%.6f" % yaw,
                "%.6f" % vx,
                reference["index"],
                "%.6f" % errors["ref_x"],
                "%.6f" % errors["ref_y"],
                "%.6f" % errors["ref_yaw"],
                "%.6f" % errors["ref_speed"],
                "%.8f" % errors["ref_curvature"],
                "%.6f" % errors["lateral_error"],
                "%.6f" % errors["longitudinal_error"],
                "%.6f" % errors["yaw_error"],
                "%.6f" % errors["speed_error"],
                "%.6f" % self.current_steer,
                "%.6f" % feedforward_steer,
                "%.6f" % errors["ref_steer"],
                "%.6f" % target_steer,
                "%.6f" % command_steer,
                "%.6f" % accel,
                "%.6f" % command_velocity,
                "%.6f" % best_cost,
                "%.6f" % preview_distance,
                preview_reference["index"],
                self.active_steering_mode,
                "%.6f" % lane_change_lateral_shift,
                "%.6f" % lane_change_heading_delta,
            ]
        )
        self.error_log_file.flush()

    def destroy_node(self):
        if self.error_log_file is not None:
            self.error_log_file.close()
            self.error_log_file = None
            self.error_log_writer = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SamplingMpcController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
