import os
import select
import sys
import termios
import time
import tty

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class TerminalRawMode:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.original = None

    def __enter__(self):
        if os.isatty(self.fd):
            self.original = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, _exc_type, _exc, _traceback):
        if self.original is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original)

    def read_available(self):
        keys = []
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1)
            if not key:
                break
            keys.append(key)
        return keys


class RemoteDriveKeyboard(Node):
    """Terminal operator for low-speed simulation remote driving."""

    def __init__(self, terminal):
        super().__init__("remote_drive_keyboard")
        self.terminal = terminal
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("deadman_hold_time", 0.35)
        self.declare_parameter("velocity_step", 0.1)
        self.declare_parameter("steer_step", 0.03)
        self.declare_parameter("max_velocity", 0.5)
        self.declare_parameter("max_reverse_velocity", 0.5)
        self.declare_parameter("front_steer_limit", 0.47822)
        self.declare_parameter("rear_steer_limit", 0.39444)

        self.deadman_hold_time = float(self.get_parameter("deadman_hold_time").value)
        self.velocity_step = float(self.get_parameter("velocity_step").value)
        self.steer_step = float(self.get_parameter("steer_step").value)
        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_reverse_velocity = float(
            self.get_parameter("max_reverse_velocity").value
        )
        self.front_steer_limit = float(self.get_parameter("front_steer_limit").value)
        self.rear_steer_limit = float(self.get_parameter("rear_steer_limit").value)

        self.velocity = 0.0
        self.front_steer = 0.0
        self.rear_steer = 0.0
        self.gear = GearCommand.DRIVE
        self.estop = False
        self.last_deadman_key = 0.0
        self.running = True

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

        publish_rate = float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(1.0 / max(1.0, publish_rate), self.on_timer)
        self.print_help()

    def print_help(self):
        print(
            """
AGV remote-drive keyboard (simulation)
--------------------------------------
r / m     : request REMOTE / AUTO authority (vehicle must be stopped)
w / s     : increase / decrease target velocity
a / d     : front axle left / right
j / l     : rear axle left / right
c         : center both steering axles
f / v / n / p : DRIVE / REVERSE / NEUTRAL / PARK
g (hold)  : deadman; release it to stop within 0.35 s
x         : set target speed to zero
!         : latch remote emergency stop
z         : reset remote emergency stop while stopped
q         : quit (deadman becomes false)
""".strip()
        )

    def request_mode(self, mode):
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    def handle_key(self, key):
        if key in ("r", "R"):
            self.request_mode("REMOTE")
        elif key in ("m", "M"):
            self.request_mode("AUTO")
        elif key in ("w", "W"):
            self.velocity = clamp(
                self.velocity + self.velocity_step,
                -self.max_reverse_velocity,
                self.max_velocity,
            )
        elif key in ("s", "S"):
            self.velocity = clamp(
                self.velocity - self.velocity_step,
                -self.max_reverse_velocity,
                self.max_velocity,
            )
        elif key in ("a", "A"):
            self.front_steer = clamp(
                self.front_steer + self.steer_step,
                -self.front_steer_limit,
                self.front_steer_limit,
            )
        elif key in ("d", "D"):
            self.front_steer = clamp(
                self.front_steer - self.steer_step,
                -self.front_steer_limit,
                self.front_steer_limit,
            )
        elif key in ("j", "J"):
            self.rear_steer = clamp(
                self.rear_steer + self.steer_step,
                -self.rear_steer_limit,
                self.rear_steer_limit,
            )
        elif key in ("l", "L"):
            self.rear_steer = clamp(
                self.rear_steer - self.steer_step,
                -self.rear_steer_limit,
                self.rear_steer_limit,
            )
        elif key in ("c", "C"):
            self.front_steer = 0.0
            self.rear_steer = 0.0
        elif key in ("f", "F"):
            self.gear = GearCommand.DRIVE
        elif key in ("v", "V"):
            self.gear = GearCommand.REVERSE
        elif key in ("n", "N"):
            self.velocity = 0.0
            self.gear = GearCommand.NEUTRAL
        elif key in ("p", "P"):
            self.velocity = 0.0
            self.gear = GearCommand.PARK
        elif key in ("g", "G"):
            self.last_deadman_key = time.monotonic()
        elif key in ("x", "X", " "):
            self.velocity = 0.0
        elif key == "!":
            self.estop = True
            self.velocity = 0.0
        elif key in ("z", "Z") and abs(self.velocity) < 1e-6:
            self.estop = False
        elif key in ("q", "Q"):
            self.running = False
        self.print_status()

    def deadman_active(self):
        return time.monotonic() - self.last_deadman_key <= self.deadman_hold_time

    def print_status(self):
        gear_names = {
            GearCommand.PARK: "PARK",
            GearCommand.REVERSE: "REVERSE",
            GearCommand.NEUTRAL: "NEUTRAL",
            GearCommand.DRIVE: "DRIVE",
        }
        print(
            "\rvelocity=%+.2f m/s front=%+.3f rear=%+.3f gear=%s deadman=%s estop=%s   "
            % (
                self.velocity,
                self.front_steer,
                self.rear_steer,
                gear_names.get(self.gear, "UNKNOWN"),
                "ON" if self.deadman_active() else "OFF",
                "ON" if self.estop else "OFF",
            ),
            end="",
            flush=True,
        )

    def publish_commands(self):
        stamp = self.get_clock().now().to_msg()
        if self.velocity > 1e-6:
            acceleration = 0.2
        elif self.velocity < -1e-6:
            acceleration = -0.2
        else:
            acceleration = 0.0

        control = Control()
        control.stamp = stamp
        control.longitudinal.stamp = stamp
        control.longitudinal.velocity = float(self.velocity)
        control.longitudinal.acceleration = float(acceleration)
        control.longitudinal.is_defined_acceleration = True
        control.lateral.stamp = stamp
        control.lateral.steering_tire_angle = float(self.front_steer)
        control.lateral.steering_tire_rotation_rate = 0.261799
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
        deadman.data = self.deadman_active() and not self.estop
        self.deadman_pub.publish(deadman)

        estop = Bool()
        estop.data = self.estop
        self.estop_pub.publish(estop)

    def publish_shutdown(self):
        self.velocity = 0.0
        self.last_deadman_key = 0.0
        self.publish_commands()

    def on_timer(self):
        for key in self.terminal.read_available():
            self.handle_key(key)
        self.publish_commands()
        if not self.running:
            self.publish_shutdown()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    with TerminalRawMode() as terminal:
        node = RemoteDriveKeyboard(terminal)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            if rclpy.ok():
                node.publish_shutdown()
        except ExternalShutdownException:
            pass
        finally:
            print()
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
