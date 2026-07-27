#!/usr/bin/env python3

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class RemoteDriveCameraViewer(Node):
    def __init__(self):
        super().__init__("remote_drive_camera_viewer")
        self.bridge = CvBridge()
        self.window_name = self.declare_parameter(
            "window_name", "AGV Remote Driving Cameras"
        ).value
        self.front_size = (
            int(self.declare_parameter("front_width", 960).value),
            int(self.declare_parameter("front_height", 540).value),
        )
        self.small_size = (
            int(self.declare_parameter("small_width", 320).value),
            int(self.declare_parameter("small_height", 180).value),
        )
        self.stale_after_s = float(self.declare_parameter("stale_after_s", 1.0).value)

        self.cameras = [
            ("front", "/remote/video/front/image"),
            ("left_front", "/remote/video/left_front/image"),
            ("right_front", "/remote/video/right_front/image"),
            ("left_rear", "/remote/video/left_rear/image"),
            ("right_rear", "/remote/video/right_rear/image"),
            ("rear", "/remote/video/rear/image"),
        ]
        self.frames = {}
        self.last_stamp = {}

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        for name, topic in self.cameras:
            self.create_subscription(
                Image,
                topic,
                lambda msg, camera_name=name: self.image_callback(camera_name, msg),
                qos,
            )

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.get_logger().info(
            "Remote driving camera viewer started. Press q or Esc in the image window to exit."
        )

    def image_callback(self, camera_name, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert {camera_name}: {exc}")
            return
        self.frames[camera_name] = frame
        self.last_stamp[camera_name] = time.monotonic()

    def tile(self, camera_name, size):
        width, height = size
        frame = self.frames.get(camera_name)
        now = time.monotonic()
        stale = camera_name not in self.last_stamp or (
            now - self.last_stamp[camera_name] > self.stale_after_s
        )

        if frame is None:
            image = np.full((height, width, 3), 32, dtype=np.uint8)
            status = "waiting"
        else:
            image = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
            status = "stale" if stale else "live"
            if stale:
                image = (image * 0.35).astype(np.uint8)

        color = (0, 220, 0) if status == "live" else (0, 180, 255)
        cv2.rectangle(image, (0, 0), (width - 1, height - 1), color, 2)
        cv2.rectangle(image, (0, 0), (width, 34), (0, 0, 0), -1)
        cv2.putText(
            image,
            f"{camera_name}  {status}",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
            cv2.LINE_AA,
        )
        return image

    def render(self):
        front = self.tile("front", self.front_size)
        small_names = ["left_front", "right_front", "left_rear", "right_rear", "rear"]
        small_tiles = [self.tile(name, self.small_size) for name in small_names]
        blank = np.full((self.small_size[1], self.small_size[0], 3), 24, dtype=np.uint8)
        small_tiles.append(blank)
        right_panel = np.vstack(
            [
                np.hstack(small_tiles[0:2]),
                np.hstack(small_tiles[2:4]),
                np.hstack(small_tiles[4:6]),
            ]
        )
        cv2.imshow(self.window_name, np.hstack([front, right_panel]))
        return cv2.waitKey(1) & 0xFF


def main():
    rclpy.init()
    node = RemoteDriveCameraViewer()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if node.render() in (ord("q"), 27):
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
