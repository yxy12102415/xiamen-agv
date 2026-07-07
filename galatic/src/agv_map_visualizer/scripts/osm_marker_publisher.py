#!/usr/bin/env python3

import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def tag_map(element):
    return {tag.attrib.get("k", ""): tag.attrib.get("v", "") for tag in element.findall("tag")}


class OsmMarkerPublisher(Node):
    def __init__(self):
        super().__init__("osm_marker_publisher")
        self.declare_parameter("map_path", "")
        self.declare_parameter("output_topic", "/map/vector_map_marker")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("line_width", 0.12)

        self.map_path = self.get_parameter("map_path").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.line_width = self.get_parameter("line_width").get_parameter_value().double_value

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(MarkerArray, self.output_topic, qos)
        self.markers = self.load_markers()
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.publish_markers()

        self.get_logger().info(
            f"Publishing standalone OSM markers from {self.map_path} on {self.output_topic}"
        )

    def load_markers(self):
        if not self.map_path:
            raise RuntimeError("map_path parameter is required")

        root = ET.parse(self.map_path).getroot()
        nodes = {}
        for node in root.findall("node"):
            tags = tag_map(node)
            if "local_x" not in tags or "local_y" not in tags:
                continue
            point = Point()
            point.x = float(tags["local_x"])
            point.y = float(tags["local_y"])
            point.z = float(tags.get("ele", "0.0"))
            nodes[node.attrib["id"]] = point

        boundary = self.make_line_marker("lanelet_boundaries", 0, 0.25, 0.85, 1.0, 0.95)
        centerline = self.make_line_marker("lanelet_centerlines", 1, 1.0, 0.85, 0.15, 0.95)

        for way in root.findall("way"):
            tags = tag_map(way)
            marker = centerline if tags.get("type") == "centerline" else boundary
            refs = [nd.attrib["ref"] for nd in way.findall("nd")]
            for first, second in zip(refs, refs[1:]):
                if first in nodes and second in nodes:
                    marker.points.append(nodes[first])
                    marker.points.append(nodes[second])

        markers = MarkerArray()
        markers.markers.append(boundary)
        markers.markers.append(centerline)
        return markers

    def make_line_marker(self, namespace, marker_id, r, g, b, a):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_width
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        return marker

    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        for marker in self.markers.markers:
            marker.header.stamp = stamp
        self.publisher.publish(self.markers)


def main():
    rclpy.init()
    node = OsmMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
