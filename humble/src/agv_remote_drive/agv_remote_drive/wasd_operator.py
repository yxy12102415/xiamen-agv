import json
import math
import threading
import time
import tkinter as tk

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand
from cv_bridge import CvBridge
from PIL import Image as PilImage
from PIL import ImageTk
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray, String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def approach(value, target, max_step):
    if value < target:
        return min(value + max_step, target)
    return max(value - max_step, target)


class WasdOperator(Node):
    """GTA-style remote operator with key-down and key-up safety semantics."""

    CAMERA_TOPICS = {
        "front": "/remote/video/front/image",
        "rear": "/remote/video/rear/image",
        "left_front": "/remote/video/left_front/image",
        "left_rear": "/remote/video/left_rear/image",
        "right_front": "/remote/video/right_front/image",
        "right_rear": "/remote/video/right_rear/image",
    }
    CAMERA_TITLES = {
        "front": "FRONT",
        "rear": "REAR",
        "left_front": "LEFT FRONT",
        "left_rear": "LEFT REAR",
        "right_front": "RIGHT FRONT",
        "right_rear": "RIGHT REAR",
    }

    def __init__(self, root):
        super().__init__("remote_drive_wasd")
        self.root = root
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("camera_render_rate", 15.0)
        self.declare_parameter("max_velocity", 3.0)
        self.declare_parameter("max_reverse_velocity", 0.3)
        self.declare_parameter("speed_step", 0.10)
        self.declare_parameter("max_drive_acceleration", 0.35)
        self.declare_parameter("brake_deceleration", 4.0)
        self.declare_parameter("front_steer_limit", 0.47822)
        self.declare_parameter("rear_steer_limit", 0.39444)
        self.declare_parameter("steer_rate", 0.9)
        self.declare_parameter("steer_return_rate", 1.2)

        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.camera_render_rate = float(
            self.get_parameter("camera_render_rate").value
        )
        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_reverse_velocity = float(
            self.get_parameter("max_reverse_velocity").value
        )
        self.speed_step = float(self.get_parameter("speed_step").value)
        self.max_drive_acceleration = float(
            self.get_parameter("max_drive_acceleration").value
        )
        self.brake_deceleration = float(
            self.get_parameter("brake_deceleration").value
        )
        self.front_steer_limit = float(self.get_parameter("front_steer_limit").value)
        self.rear_steer_limit = float(self.get_parameter("rear_steer_limit").value)
        self.steer_rate = float(self.get_parameter("steer_rate").value)
        self.steer_return_rate = float(self.get_parameter("steer_return_rate").value)

        self.velocity = 0.0
        self.front_steer = 0.0
        self.rear_steer = 0.0
        self.gear = GearCommand.DRIVE
        self.estop = False
        self.focus_active = False
        self.control_armed = False
        self.keys_down = set()
        self.pending_releases = {}
        self.last_update = time.monotonic()
        self.command_acceleration = 0.0
        self.closed = False
        self.bridge = CvBridge()
        self.camera_lock = threading.Lock()
        self.camera_messages = {name: None for name in self.CAMERA_TOPICS}
        self.camera_versions = {name: 0 for name in self.CAMERA_TOPICS}
        self.rendered_versions = {name: -1 for name in self.CAMERA_TOPICS}
        self.camera_received_at = {name: 0.0 for name in self.CAMERA_TOPICS}
        self.camera_receive_counts = {name: 0 for name in self.CAMERA_TOPICS}
        self.camera_render_counts = {name: 0 for name in self.CAMERA_TOPICS}
        self.camera_receive_fps = {name: 0.0 for name in self.CAMERA_TOPICS}
        self.camera_render_fps = {name: 0.0 for name in self.CAMERA_TOPICS}
        self.camera_metric_counts = {name: (0, 0) for name in self.CAMERA_TOPICS}
        self.camera_metric_time = time.monotonic()
        self.next_camera_render = self.camera_metric_time
        self.camera_labels = {}
        self.camera_caption_labels = {}
        self.camera_photos = {}
        self.camera_errors_logged = set()
        self.main_camera = "front"
        self.protocol_link = "WAITING"
        self.protocol_diagnostic = {}
        self.video_network = {}
        self.vehicle_state = {}
        self.vehicle_state_received_at = 0.0
        self.chassis_summary = {}
        self.chassis_received_at = 0.0

        self.control_pub = self.create_publisher(
            Control, "/remote/operator/control_cmd", 1
        )
        self.two_axis_pub = self.create_publisher(
            Float32MultiArray, "/remote/operator/two_axis_steer_cmd", 1
        )
        self.gear_pub = self.create_publisher(
            GearCommand, "/remote/operator/gear_cmd", 1
        )
        self.deadman_pub = self.create_publisher(Bool, "/remote/operator/deadman", 1)
        self.estop_pub = self.create_publisher(
            Bool, "/remote/operator/emergency_stop", 1
        )
        self.mode_pub = self.create_publisher(
            String, "/agv/control/authority/request", 10
        )
        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        for camera_name, topic in self.CAMERA_TOPICS.items():
            self.create_subscription(
                Image,
                topic,
                lambda msg, name=camera_name: self.on_camera_image(name, msg),
                camera_qos,
            )
        self.create_subscription(
            String,
            "/remote/protocol/domain_state",
            self.on_protocol_diagnostic,
            10,
        )
        self.create_subscription(
            String,
            "/remote/protocol/vehicle_state",
            self.on_vehicle_state,
            10,
        )
        self.create_subscription(
            String,
            "/agv/chassis/state_summary",
            self.on_chassis_summary,
            10,
        )
        self.create_subscription(
            String,
            "/remote/network/video_state",
            self.on_video_network,
            10,
        )

        self.build_window()
        self.root.after(500, self.activate_window)

    def build_window(self):
        self.root.title("AGV Remote Driving Console")
        self.root.geometry("1440x860")
        self.root.minsize(1280, 760)
        self.root.configure(bg="#111417")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.root.bind("<FocusIn>", self.on_focus_in)
        self.root.bind("<FocusOut>", self.on_focus_out)

        header = tk.Frame(self.root, bg="#1b2024", height=54)
        header.pack(fill="x", padx=12, pady=(12, 0))
        header.pack_propagate(False)
        tk.Label(
            header,
            text="AGV REMOTE DRIVE",
            font=("Sans", 17, "bold"),
            fg="#f4f6f8",
            bg="#1b2024",
        ).pack(side="left", padx=18)
        self.header_status_label = tk.Label(
            header,
            text="CONTROL INITIALIZING",
            font=("Sans", 10, "bold"),
            fg="#ffca5c",
            bg="#1b2024",
        )
        self.header_status_label.pack(side="right", padx=18)

        outer = tk.Frame(self.root, bg="#111417")
        outer.pack(fill="both", expand=True, padx=12, pady=12)
        outer.grid_columnconfigure(1, weight=1)
        outer.grid_rowconfigure(0, weight=1)

        control_panel = tk.Frame(outer, bg="#20262b", width=300)
        control_panel.grid(row=0, column=0, sticky="ns", padx=(0, 12))
        control_panel.grid_propagate(False)

        video_panel = tk.Frame(outer, bg="#0b0e10")
        video_panel.grid(row=0, column=1, sticky="nsew")
        video_panel.grid_columnconfigure(0, weight=1)

        self.focus_label = tk.Label(
            control_panel,
            text="CONTROL ARMING",
            font=("Sans", 10, "bold"),
            fg="#ffca5c",
            bg="#20262b",
            pady=16,
        )
        self.focus_label.pack(fill="x")

        mode_actions = tk.Frame(control_panel, bg="#20262b")
        mode_actions.pack(fill="x", padx=14, pady=(0, 10))
        self.make_action_button(
            mode_actions, "REMOTE", "#24734f", self.enable_remote
        ).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.make_action_button(
            mode_actions, "AUTO", "#3e586d", self.enable_auto
        ).grid(row=0, column=1, sticky="ew", padx=(4, 0))
        mode_actions.grid_columnconfigure(0, weight=1)
        mode_actions.grid_columnconfigure(1, weight=1)

        safety_actions = tk.Frame(control_panel, bg="#20262b")
        safety_actions.pack(fill="x", padx=14, pady=(0, 14))
        self.make_action_button(
            safety_actions, "E-STOP", "#a93636", self.activate_estop
        ).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.make_action_button(
            safety_actions, "RESET", "#6a5632", self.reset_estop
        ).grid(row=0, column=1, sticky="ew", padx=(4, 0))
        safety_actions.grid_columnconfigure(0, weight=1)
        safety_actions.grid_columnconfigure(1, weight=1)

        self.make_section_label(control_panel, "DRIVING").pack(
            fill="x", padx=14, pady=(0, 4)
        )
        self.telemetry_label = tk.Label(
            control_panel,
            text="COMMAND  +0.00 m/s\nACTUAL   +0.00 m/s\nFRONT    +0.0 deg\nREAR     +0.0 deg",
            font=("Monospace", 13, "bold"),
            fg="#72d7a5",
            bg="#171c20",
            justify="left",
            anchor="w",
            padx=12,
            pady=10,
        )
        self.telemetry_label.pack(fill="x", padx=14)

        self.make_section_label(control_panel, "CHASSIS").pack(
            fill="x", padx=14, pady=(14, 4)
        )
        self.chassis_label = tk.Label(
            control_panel,
            text="GEAR   WAITING\nBRAKE  -- bar\nMOTORS WAITING\nFAULT  WAITING",
            font=("Monospace", 10, "bold"),
            fg="#c2cbd0",
            bg="#171c20",
            justify="left",
            anchor="w",
            padx=12,
            pady=9,
        )
        self.chassis_label.pack(fill="x", padx=14)

        self.make_section_label(control_panel, "LINK & SAFETY").pack(
            fill="x", padx=14, pady=(14, 4)
        )
        self.state_label = tk.Label(
            control_panel,
            text="CONTROL WAITING\nVIDEO WAITING\nFEEDBACK WAITING",
            font=("Monospace", 10, "bold"),
            fg="#aeb7bc",
            bg="#171c20",
            justify="left",
            anchor="w",
            padx=12,
            pady=9,
        )
        self.state_label.pack(fill="x", padx=14)

        tk.Label(
            control_panel,
            text="W/S  SPEED     A/D  STEER     SPACE  BRAKE\nE  E-STOP      Z  RESET      Q  QUIT",
            font=("Sans", 9, "bold"),
            fg="#9ca6ad",
            bg="#20262b",
            justify="center",
            pady=14,
        ).pack(side="bottom", fill="x", padx=12)

        front_caption = self.make_camera_caption(video_panel, "front")
        front_caption.pack(fill="x", padx=8, pady=(8, 0))
        front_label = tk.Label(
            video_panel,
            text="FRONT CAMERA\nWAITING FOR VIDEO",
            font=("Sans", 16, "bold"),
            fg="#6f787e",
            bg="#0b0d0e",
        )
        front_label.pack(fill="both", expand=True, padx=8)
        self.camera_labels["front"] = front_label

        thumbnails = tk.Frame(video_panel, bg="#101214")
        thumbnails.pack(fill="x", padx=8, pady=(8, 8))
        thumbnail_names = (
            "rear",
            "left_front",
            "left_rear",
            "right_front",
            "right_rear",
        )
        for column, camera_name in enumerate(thumbnail_names):
            cell = tk.Frame(thumbnails, bg="#101214")
            cell.grid(row=0, column=column, sticky="nsew", padx=(0 if column == 0 else 4, 0))
            thumbnails.grid_columnconfigure(column, weight=1, uniform="camera")
            self.make_camera_caption(cell, camera_name).pack(fill="x")
            label = tk.Label(
                cell,
                text="WAITING",
                font=("Sans", 9, "bold"),
                fg="#6f787e",
                bg="#0b0d0e",
            )
            label.pack(fill="both", expand=True)
            self.camera_labels[camera_name] = label
        self.bind_camera_selection()

    @staticmethod
    def make_section_label(parent, text):
        return tk.Label(
            parent,
            text=text,
            font=("Sans", 9, "bold"),
            fg="#8ea0ab",
            bg="#20262b",
            anchor="w",
        )

    def make_action_button(self, parent, text, color, command):
        return tk.Button(
            parent,
            text=text,
            font=("Sans", 10, "bold"),
            fg="#f7f8fa",
            bg=color,
            activebackground=color,
            activeforeground="#ffffff",
            relief="flat",
            bd=0,
            padx=8,
            pady=9,
            command=command,
        )

    def make_camera_caption(self, parent, camera_name):
        label = tk.Label(
            parent,
            text=self.CAMERA_TITLES[camera_name] + " | WAITING",
            font=("Sans", 9, "bold"),
            fg="#aeb7bc",
            bg="#1c2023",
            anchor="w",
            padx=8,
            pady=4,
        )
        self.camera_caption_labels[camera_name] = label
        return label

    def bind_camera_selection(self):
        for camera_name, label in self.camera_labels.items():
            label.bind(
                "<Button-1>",
                lambda _event, name=camera_name: self.select_main_camera(name),
            )
        for camera_name, label in self.camera_caption_labels.items():
            label.bind(
                "<Button-1>",
                lambda _event, name=camera_name: self.select_main_camera(name),
            )

    def select_main_camera(self, camera_name):
        if camera_name == self.main_camera or camera_name not in self.camera_labels:
            return
        old_main = self.main_camera
        self.camera_labels[old_main], self.camera_labels[camera_name] = (
            self.camera_labels[camera_name],
            self.camera_labels[old_main],
        )
        self.camera_caption_labels[old_main], self.camera_caption_labels[camera_name] = (
            self.camera_caption_labels[camera_name],
            self.camera_caption_labels[old_main],
        )
        self.main_camera = camera_name
        self.rendered_versions[old_main] = -1
        self.rendered_versions[camera_name] = -1
        self.bind_camera_selection()

    def activate_window(self):
        self.root.lift()
        self.root.focus_force()

    def normalized_key(self, event):
        key = event.keysym.lower()
        return "space" if key == "space" else key

    def on_key_press(self, event):
        key = self.normalized_key(event)
        if key == "e":
            self.activate_estop()
        elif key == "z" and abs(self.velocity) < 0.02:
            self.reset_estop()
        elif key == "r":
            self.enable_remote()
        elif key == "m":
            self.enable_auto()
        elif key in ("q", "escape"):
            self.close()
        elif key == "w" and self.control_armed:
            self.velocity = min(self.max_velocity, self.velocity + self.speed_step)
        elif key == "s" and self.control_armed:
            self.velocity = max(
                -self.max_reverse_velocity, self.velocity - self.speed_step
            )
        elif key == "space" and self.control_armed:
            self.velocity = 0.0
            self.command_acceleration = -self.brake_deceleration
        elif key in ("a", "d") and self.control_armed:
            release_job = self.pending_releases.pop(key, None)
            if release_job is not None:
                self.root.after_cancel(release_job)
            self.keys_down.add(key)

    def on_key_release(self, event):
        key = self.normalized_key(event)
        old_job = self.pending_releases.pop(key, None)
        if old_job is not None:
            self.root.after_cancel(old_job)
        self.pending_releases[key] = self.root.after(
            25, lambda released_key=key: self.finish_key_release(released_key)
        )

    def finish_key_release(self, key):
        self.pending_releases.pop(key, None)
        self.keys_down.discard(key)

    def on_focus_in(self, _event):
        self.focus_active = True

    def on_focus_out(self, _event):
        self.focus_active = False
        self.disarm_control()

    def disarm_control(self):
        self.control_armed = False
        self.keys_down.clear()
        for job in self.pending_releases.values():
            self.root.after_cancel(job)
        self.pending_releases.clear()

    def request_mode(self, mode):
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    def enable_remote(self):
        self.disarm_control()
        self.velocity = 0.0
        self.front_steer = 0.0
        self.rear_steer = 0.0
        self.control_armed = True
        self.request_mode("REMOTE")
        self.activate_window()

    def enable_auto(self):
        self.disarm_control()
        self.request_mode("AUTO")
        self.activate_window()

    def activate_estop(self):
        self.disarm_control()
        self.estop = True
        self.velocity = 0.0
        self.activate_window()

    def reset_estop(self):
        if abs(self.velocity) < 0.02:
            self.estop = False
        self.activate_window()

    def on_camera_image(self, camera_name, msg):
        with self.camera_lock:
            self.camera_messages[camera_name] = msg
            self.camera_versions[camera_name] += 1
            self.camera_receive_counts[camera_name] += 1
            self.camera_received_at[camera_name] = time.monotonic()

    def on_protocol_diagnostic(self, msg):
        try:
            self.protocol_diagnostic = json.loads(msg.data)
            self.protocol_link = str(self.protocol_diagnostic.get("link", "UNKNOWN"))
        except (TypeError, ValueError):
            self.protocol_diagnostic = {}
            self.protocol_link = "ERROR"

    def on_video_network(self, msg):
        try:
            incoming = json.loads(msg.data)
            incoming_cameras = incoming.get("cameras", {})
            if len(incoming_cameras) == 1:
                current = self.video_network
                if current.get("transport") != incoming.get("transport"):
                    current = {}
                merged_cameras = dict(current.get("cameras", {}))
                for camera_name, camera_update in incoming_cameras.items():
                    merged_camera = dict(merged_cameras.get(camera_name, {}))
                    merged_camera.update(camera_update)
                    merged_cameras[camera_name] = merged_camera
                current.update(incoming)
                current["cameras"] = merged_cameras
                current["link"] = (
                    "LIVE"
                    if any(
                        camera.get("state") == "LIVE"
                        for camera in merged_cameras.values()
                    )
                    else "WAITING"
                )
                self.video_network = current
            else:
                self.video_network = incoming
        except (TypeError, ValueError):
            self.video_network = {}

    def on_vehicle_state(self, msg):
        try:
            self.vehicle_state = json.loads(msg.data)
            self.vehicle_state_received_at = time.monotonic()
        except (TypeError, ValueError):
            self.vehicle_state = {}

    def on_chassis_summary(self, msg):
        try:
            self.chassis_summary = json.loads(msg.data)
            self.chassis_received_at = time.monotonic()
        except (TypeError, ValueError):
            self.chassis_summary = {}

    def update_targets(self, dt):
        if self.estop or not self.focus_active or not self.control_armed:
            self.velocity = approach(
                self.velocity, 0.0, self.brake_deceleration * dt
            )

        steer_direction = 0.0
        if "a" in self.keys_down and "d" not in self.keys_down:
            steer_direction = 1.0
        elif "d" in self.keys_down and "a" not in self.keys_down:
            steer_direction = -1.0
        steer_target = steer_direction * self.front_steer_limit
        steer_speed = self.steer_rate if steer_direction else self.steer_return_rate
        self.front_steer = approach(
            self.front_steer, steer_target, steer_speed * dt
        )
        rear_ratio = self.rear_steer_limit / max(1.0e-6, self.front_steer_limit)
        self.rear_steer = clamp(
            self.front_steer * rear_ratio,
            -self.rear_steer_limit,
            self.rear_steer_limit,
        )
        if self.velocity > 0.01:
            self.gear = GearCommand.DRIVE
        elif self.velocity < -0.01:
            self.gear = GearCommand.REVERSE

    def feedback_velocity(self):
        if (
            self.chassis_received_at > 0.0
            and time.monotonic() - self.chassis_received_at <= 1.0
        ):
            return float(self.chassis_summary.get("actual_velocity_mps", 0.0))
        if (
            self.vehicle_state_received_at > 0.0
            and time.monotonic() - self.vehicle_state_received_at <= 1.0
        ):
            return float(self.vehicle_state.get("speed_feedback", 0.0))
        return 0.0

    def publish_commands(self):
        stamp = self.get_clock().now().to_msg()
        acceleration = self.command_acceleration

        control = Control()
        control.stamp = stamp
        control.longitudinal.stamp = stamp
        control.longitudinal.velocity = float(self.velocity)
        control.longitudinal.acceleration = float(acceleration)
        control.longitudinal.is_defined_acceleration = True
        control.lateral.stamp = stamp
        control.lateral.steering_tire_angle = float(self.front_steer)
        control.lateral.steering_tire_rotation_rate = float(self.steer_rate)
        control.lateral.is_defined_steering_tire_rotation_rate = True
        self.control_pub.publish(control)

        two_axis = Float32MultiArray()
        two_axis.data = [
            float(self.front_steer),
            float(self.rear_steer),
            float(self.velocity),
            float(acceleration),
        ]
        self.two_axis_pub.publish(two_axis)

        gear = GearCommand()
        gear.stamp = stamp
        gear.command = self.gear
        self.gear_pub.publish(gear)

        deadman = Bool()
        deadman.data = self.focus_active and self.control_armed and not self.estop
        self.deadman_pub.publish(deadman)

        estop = Bool()
        estop.data = self.estop
        self.estop_pub.publish(estop)

    def update_labels(self):
        vehicle_feedback_fresh = (
            self.vehicle_state_received_at > 0.0
            and time.monotonic() - self.vehicle_state_received_at <= 1.0
        )
        chassis_fresh = (
            self.chassis_received_at > 0.0
            and time.monotonic() - self.chassis_received_at <= 1.0
        )
        actual_speed = (
            float(self.chassis_summary.get("actual_velocity_mps", 0.0))
            if chassis_fresh
            else float(self.vehicle_state.get("speed_feedback", 0.0))
            if vehicle_feedback_fresh
            else 0.0
        )
        front_steer = (
            math.degrees(float(self.chassis_summary.get("front_steer_rad", 0.0)))
            if chassis_fresh
            else math.degrees(self.front_steer)
        )
        rear_steer = (
            math.degrees(float(self.chassis_summary.get("rear_steer_rad", 0.0)))
            if chassis_fresh
            else math.degrees(self.rear_steer)
        )
        self.telemetry_label.configure(
            text=(
                "COMMAND  %+.2f m/s\nACTUAL   %+.2f m/s\n"
                "FRONT    %+.1f deg\nREAR     %+.1f deg"
            )
            % (self.velocity, actual_speed, front_steer, rear_steer)
        )
        control_delay = self.protocol_diagnostic.get("network", {}).get(
            "delay_ms", 0.0
        )
        feedback_latency = self.protocol_diagnostic.get(
            "last_vehicle_state_latency_ms"
        )
        video_delay = self.video_network.get("configured_delay_ms", 0.0)
        video_link = self.video_network.get("link", "WAITING")
        video_transport = self.video_network.get("transport", "RAW ROS")
        live_camera_count = sum(
            camera.get("state") == "LIVE"
            for camera in self.video_network.get("cameras", {}).values()
        )
        feedback_text = (
            "%.0f ms" % float(feedback_latency)
            if feedback_latency is not None
            else "waiting"
        )
        brake_pressure = self.chassis_summary.get("brake_pressure_bar")
        brake_text = (
            "%.0f bar" % float(brake_pressure)
            if brake_pressure is not None and chassis_fresh
            else "-- bar"
        )
        battery_level = self.vehicle_state.get("battery_level")
        battery_text = (
            "%.0f%%" % float(battery_level)
            if battery_level is not None and vehicle_feedback_fresh
            else "--"
        )
        fault_text = self.chassis_summary.get("fault_state", "WAITING")
        if not chassis_fresh:
            fault_text = "STALE"
        self.chassis_label.configure(
            text=(
                "GEAR   %-8s BAT %s\nBRAKE  %-12s\nMOTORS D:%s S:%s\nFAULT  %s"
                % (
                    str(self.chassis_summary.get("gear_state", "WAITING")),
                    battery_text,
                    brake_text,
                    str(self.chassis_summary.get("drive_motor_state", "WAITING")),
                    str(self.chassis_summary.get("steering_motor_state", "WAITING")),
                    str(fault_text),
                )
            )
        )
        link_text = (
            "CONTROL  %-10s  TX %3.0f ms\n"
            "VIDEO    %-10s  %d/6 LIVE\n"
            "FEEDBACK %-10s  %s\n"
            "TRANSPORT %s / %3.0f ms"
            % (
                self.protocol_link,
                float(control_delay),
                video_link,
                live_camera_count,
                "LIVE" if chassis_fresh else "STALE",
                feedback_text,
                video_transport,
                float(video_delay),
            )
        )
        if self.estop:
            self.focus_label.configure(text="EMERGENCY STOP", fg="#ff6b6b")
            self.header_status_label.configure(text="E-STOP ACTIVE", fg="#ff6b6b")
            self.state_label.configure(text=link_text)
        elif self.focus_active and self.control_armed:
            self.focus_label.configure(text="CONTROL ACTIVE", fg="#70d6a5")
            self.header_status_label.configure(
                text="REMOTE CONTROL ACTIVE", fg="#70d6a5"
            )
            self.state_label.configure(text=link_text)
        elif self.focus_active:
            self.focus_label.configure(text="REMOTE READY", fg="#ffca5c")
            self.header_status_label.configure(
                text="CONTROL NOT ARMED", fg="#ffca5c"
            )
            self.state_label.configure(text=link_text)
        else:
            self.focus_label.configure(
                text="SAFE STOP: WINDOW NOT FOCUSED", fg="#ffca5c"
            )
            self.header_status_label.configure(
                text="CONTROL NOT ARMED", fg="#ffca5c"
            )
            self.state_label.configure(text=link_text)

    def update_camera_metrics(self, now):
        elapsed = now - self.camera_metric_time
        if elapsed < 1.0:
            return
        with self.camera_lock:
            current = {
                name: (
                    self.camera_receive_counts[name],
                    self.camera_render_counts[name],
                )
                for name in self.CAMERA_TOPICS
            }
        for name, (receive_count, render_count) in current.items():
            old_receive, old_render = self.camera_metric_counts[name]
            self.camera_receive_fps[name] = (receive_count - old_receive) / elapsed
            self.camera_render_fps[name] = (render_count - old_render) / elapsed
        self.camera_metric_counts = current
        self.camera_metric_time = now

    def update_camera_views(self, now):
        self.update_camera_metrics(now)
        with self.camera_lock:
            snapshots = {
                name: (
                    self.camera_messages[name],
                    self.camera_versions[name],
                    self.camera_received_at[name],
                )
                for name in self.CAMERA_TOPICS
            }

        for camera_name, (msg, version, received_at) in snapshots.items():
            caption = self.camera_caption_labels[camera_name]
            age = now - received_at if received_at else float("inf")
            state = "LIVE" if age <= 1.0 else ("STALE" if msg is not None else "WAITING")
            network_camera = self.video_network.get("cameras", {}).get(
                camera_name, {}
            )
            encoder_error = network_camera.get("last_error")
            if msg is None and encoder_error:
                state = "ERROR"
            delivery_delay_ms = network_camera.get("last_delivery_delay_ms")
            bitrate_mbps = network_camera.get("bitrate_mbps")
            estimated_age_ms = (
                None
                if delivery_delay_ms is None or received_at == 0.0
                else float(delivery_delay_ms) + age * 1000.0
            )
            state_color = {
                "LIVE": "#70d6a5",
                "STALE": "#ffca5c",
                "WAITING": "#aeb7bc",
                "ERROR": "#ff6b6b",
            }[state]
            caption.configure(
                text="%s\n%s | %.0f/%.0f FPS | AGE %s%s"
                % (
                    self.CAMERA_TITLES[camera_name],
                    state,
                    self.camera_receive_fps[camera_name],
                    self.camera_render_fps[camera_name],
                    "--" if estimated_age_ms is None else "%.0f ms" % estimated_age_ms,
                    ""
                    if bitrate_mbps is None
                    else " | %.1f Mb/s" % float(bitrate_mbps),
                ),
                fg=state_color,
            )
            if msg is None or version == self.rendered_versions[camera_name]:
                continue
            try:
                rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                size = (960, 540) if camera_name == self.main_camera else (180, 102)
                resampling = getattr(PilImage, "Resampling", PilImage)
                image = PilImage.fromarray(rgb).resize(size, resampling.BILINEAR)
                photo = ImageTk.PhotoImage(image=image)
                self.camera_photos[camera_name] = photo
                self.camera_labels[camera_name].configure(image=photo, text="")
                self.rendered_versions[camera_name] = version
                with self.camera_lock:
                    self.camera_render_counts[camera_name] += 1
            except Exception as exc:
                caption.configure(
                    text="%s | IMAGE ERROR" % self.CAMERA_TITLES[camera_name],
                    fg="#ff6b6b",
                )
                if camera_name not in self.camera_errors_logged:
                    self.camera_errors_logged.add(camera_name)
                    self.get_logger().warning(
                        "Failed to render %s camera: %s" % (camera_name, exc)
                    )

    def camera_tick(self):
        if self.closed or not rclpy.ok():
            return
        now = time.monotonic()
        self.update_camera_views(now)

        interval = 1.0 / max(1.0, self.camera_render_rate)
        self.next_camera_render += interval
        finished = time.monotonic()
        if self.next_camera_render <= finished:
            missed = int((finished - self.next_camera_render) / interval) + 1
            self.next_camera_render += missed * interval
        delay_ms = max(1, int(round((self.next_camera_render - finished) * 1000.0)))
        self.root.after(delay_ms, self.camera_tick)

    def tick(self):
        if self.closed:
            return
        if not rclpy.ok():
            self.close()
            return
        now = time.monotonic()
        dt = clamp(now - self.last_update, 0.0, 0.1)
        self.last_update = now
        self.update_targets(dt)
        speed_error = self.velocity - self.feedback_velocity()
        self.command_acceleration = clamp(
            speed_error / 0.3,
            -self.brake_deceleration,
            self.max_drive_acceleration,
        )
        self.publish_commands()
        self.update_labels()
        self.root.after(max(1, int(1000.0 / self.publish_rate)), self.tick)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.focus_active = False
        self.disarm_control()
        self.velocity = 0.0
        self.command_acceleration = 0.0
        if rclpy.ok():
            self.publish_commands()
        self.root.quit()


def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    node = WasdOperator(root)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin_executor():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    executor_thread = threading.Thread(target=spin_executor, daemon=True)
    executor_thread.start()
    root.after(0, node.tick)
    root.after(0, node.camera_tick)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        node.close()
    finally:
        executor.shutdown(timeout_sec=1.0)
        executor_thread.join(timeout=1.0)
        root.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
