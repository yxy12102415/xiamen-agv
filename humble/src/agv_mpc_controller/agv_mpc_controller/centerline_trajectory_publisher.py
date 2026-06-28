import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Quaternion
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def resample_polyline(points, count):
    if len(points) == count:
        return points
    if len(points) < 2:
        return points * count

    lengths = [0.0]
    for i in range(1, len(points)):
        lengths.append(lengths[-1] + distance(points[i - 1], points[i]))
    total = lengths[-1]
    if total <= 1e-6:
        return [points[0]] * count

    result = []
    for i in range(count):
        target = total * i / max(1, count - 1)
        seg = 1
        while seg < len(lengths) and lengths[seg] < target:
            seg += 1
        if seg >= len(lengths):
            result.append(points[-1])
            continue
        span = max(1e-6, lengths[seg] - lengths[seg - 1])
        ratio = (target - lengths[seg - 1]) / span
        x = points[seg - 1][0] + ratio * (points[seg][0] - points[seg - 1][0])
        y = points[seg - 1][1] + ratio * (points[seg][1] - points[seg - 1][1])
        z = points[seg - 1][2] + ratio * (points[seg][2] - points[seg - 1][2])
        result.append((x, y, z))
    return result


def resample_by_spacing(points, spacing):
    if len(points) < 2:
        return points

    result = [points[0]]
    carry = 0.0
    prev = points[0]
    for current in points[1:]:
        seg_len = distance(prev, current)
        if seg_len <= 1e-6:
            prev = current
            continue
        direction = (
            (current[0] - prev[0]) / seg_len,
            (current[1] - prev[1]) / seg_len,
            (current[2] - prev[2]) / seg_len,
        )
        traveled = spacing - carry
        while traveled <= seg_len:
            result.append(
                (
                    prev[0] + direction[0] * traveled,
                    prev[1] + direction[1] * traveled,
                    prev[2] + direction[2] * traveled,
                )
            )
            traveled += spacing
        carry = seg_len - (traveled - spacing)
        prev = current
    if distance(result[-1], points[-1]) > spacing * 0.5:
        result.append(points[-1])
    return result


def interpolate_gap(start, end, spacing):
    gap = distance(start, end)
    if gap <= spacing * 1.5:
        return []

    direction = (
        (end[0] - start[0]) / gap,
        (end[1] - start[1]) / gap,
        (end[2] - start[2]) / gap,
    )
    result = []
    traveled = spacing
    while traveled < gap - spacing * 0.5:
        result.append(
            (
                start[0] + direction[0] * traveled,
                start[1] + direction[1] * traveled,
                start[2] + direction[2] * traveled,
            )
        )
        traveled += spacing
    return result


def smooth_path_points(points, passes):
    smoothed = list(points)
    for _ in range(max(0, int(passes))):
        if len(smoothed) < 3:
            break
        next_points = smoothed[:]
        for i in range(1, len(smoothed) - 1):
            prev_point = smoothed[i - 1]
            point = smoothed[i]
            next_point = smoothed[i + 1]
            next_points[i] = (
                0.25 * prev_point[0] + 0.5 * point[0] + 0.25 * next_point[0],
                0.25 * prev_point[1] + 0.5 * point[1] + 0.25 * next_point[1],
                0.25 * prev_point[2] + 0.5 * point[2] + 0.25 * next_point[2],
            )
        smoothed = next_points
    return smoothed


def cumulative_lengths(points):
    lengths = [0.0]
    for i in range(1, len(points)):
        lengths.append(lengths[-1] + distance(points[i - 1], points[i]))
    return lengths


def sample_by_s(points, lengths, target_s, wrap=True):
    if not points:
        raise RuntimeError("Cannot sample an empty path")
    if len(points) == 1:
        return points[0]

    total = lengths[-1]
    if wrap and total > 1e-6:
        target_s = target_s % total
    else:
        target_s = max(0.0, min(total, target_s))

    seg = 1
    while seg < len(lengths) and lengths[seg] < target_s:
        seg += 1
    if seg >= len(lengths):
        return points[-1]

    span = max(1e-6, lengths[seg] - lengths[seg - 1])
    ratio = (target_s - lengths[seg - 1]) / span
    return (
        points[seg - 1][0] + ratio * (points[seg][0] - points[seg - 1][0]),
        points[seg - 1][1] + ratio * (points[seg][1] - points[seg - 1][1]),
        points[seg - 1][2] + ratio * (points[seg][2] - points[seg - 1][2]),
    )


def slice_ring(points, lengths, start_s, end_s, spacing):
    total = lengths[-1]
    if total <= 1e-6:
        return list(points)
    if end_s <= start_s:
        end_s += total

    result = []
    current_s = start_s
    while current_s < end_s:
        result.append(sample_by_s(points, lengths, current_s, wrap=True))
        current_s += spacing
    result.append(sample_by_s(points, lengths, end_s, wrap=True))
    return result


def lane_change_curve(start, end, spacing):
    gap = distance(start, end)
    steps = max(2, int(math.ceil(gap / max(0.1, spacing))))
    result = []
    for i in range(steps + 1):
        ratio = i / steps
        smooth = ratio * ratio * (3.0 - 2.0 * ratio)
        result.append(
            (
                start[0] + (end[0] - start[0]) * smooth,
                start[1] + (end[1] - start[1]) * smooth,
                start[2] + (end[2] - start[2]) * smooth,
            )
        )
    return result


def append_line(points, end, spacing):
    if not points:
        points.append(end)
        return
    start = points[-1]
    length = distance(start, end)
    if length <= 1e-6:
        return
    steps = max(1, int(math.ceil(length / spacing)))
    for index in range(1, steps + 1):
        ratio = index / steps
        points.append(
            (
                start[0] + (end[0] - start[0]) * ratio,
                start[1] + (end[1] - start[1]) * ratio,
                start[2] + (end[2] - start[2]) * ratio,
            )
        )


def append_arc(points, center, radius, start_angle, end_angle, z, spacing):
    arc_length = abs(end_angle - start_angle) * radius
    steps = max(2, int(math.ceil(arc_length / spacing)))
    for index in range(1, steps + 1):
        ratio = index / steps
        angle = start_angle + (end_angle - start_angle) * ratio
        points.append(
            (
                center[0] + radius * math.cos(angle),
                center[1] + radius * math.sin(angle),
                z,
            )
        )


class CenterlineTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("centerline_trajectory_publisher")
        self.declare_parameter("map_path", "")
        self.declare_parameter("output_topic", "/planning/scenario_planning/trajectory")
        self.declare_parameter("marker_topic", "/visualization/centerline_trajectory_marker")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("target_speed", 4.0)
        self.declare_parameter("min_speed", 0.5)
        self.declare_parameter("max_lateral_accel", 0.35)
        self.declare_parameter("curvature_window", 6)
        self.declare_parameter("speed_smoothing_passes", 3)
        self.declare_parameter("short_straight_speed_limit", 1.0)
        self.declare_parameter("turn_speed_limit", 1.0)
        self.declare_parameter("turn_curvature_threshold", 0.015)
        self.declare_parameter("turn_entry_hold_distance", 8.0)
        self.declare_parameter("turn_exit_hold_distance", 12.0)
        self.declare_parameter("max_profile_accel", 0.35)
        self.declare_parameter("max_profile_decel", 0.35)
        self.declare_parameter("path_smoothing_passes", 3)
        self.declare_parameter("point_spacing", 0.5)
        self.declare_parameter("route_mode", "dual_lane_change")
        self.declare_parameter("route_lane_index", 1)
        self.declare_parameter("route_turn_radius", 10.0)
        self.declare_parameter("lane_change_length", 16.0)
        self.declare_parameter("inner_to_outer_inner_s", 84.0)
        self.declare_parameter("inner_to_outer_outer_s", 84.0)
        self.declare_parameter("outer_to_inner_outer_s", 218.0)
        self.declare_parameter("outer_to_inner_inner_s", 212.0)
        self.declare_parameter("initial_x", 9.9991)
        self.declare_parameter("initial_y", 28.0013)
        self.declare_parameter("initial_yaw", -1.5708)

        map_path = self.get_parameter("map_path").get_parameter_value().string_value
        if not map_path:
            map_path = os.path.join(
                get_package_share_directory("agv_maps"), "map", "xiamen-prod.osm"
            )
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.target_speed = self.get_parameter("target_speed").get_parameter_value().double_value
        self.min_speed = self.get_parameter("min_speed").get_parameter_value().double_value
        self.max_lateral_accel = (
            self.get_parameter("max_lateral_accel").get_parameter_value().double_value
        )
        self.curvature_window = (
            self.get_parameter("curvature_window").get_parameter_value().integer_value
        )
        self.speed_smoothing_passes = (
            self.get_parameter("speed_smoothing_passes").get_parameter_value().integer_value
        )
        self.short_straight_speed_limit = (
            self.get_parameter("short_straight_speed_limit").get_parameter_value().double_value
        )
        self.turn_speed_limit = (
            self.get_parameter("turn_speed_limit").get_parameter_value().double_value
        )
        self.turn_curvature_threshold = (
            self.get_parameter("turn_curvature_threshold").get_parameter_value().double_value
        )
        self.turn_entry_hold_distance = (
            self.get_parameter("turn_entry_hold_distance").get_parameter_value().double_value
        )
        self.turn_exit_hold_distance = (
            self.get_parameter("turn_exit_hold_distance").get_parameter_value().double_value
        )
        self.max_profile_accel = (
            self.get_parameter("max_profile_accel").get_parameter_value().double_value
        )
        self.max_profile_decel = (
            self.get_parameter("max_profile_decel").get_parameter_value().double_value
        )
        self.path_smoothing_passes = (
            self.get_parameter("path_smoothing_passes").get_parameter_value().integer_value
        )
        self.point_spacing = self.get_parameter("point_spacing").get_parameter_value().double_value
        self.route_mode = self.get_parameter("route_mode").get_parameter_value().string_value
        self.route_lane_index = max(
            1, self.get_parameter("route_lane_index").get_parameter_value().integer_value
        )
        self.route_turn_radius = (
            self.get_parameter("route_turn_radius").get_parameter_value().double_value
        )
        self.lane_change_length = (
            self.get_parameter("lane_change_length").get_parameter_value().double_value
        )
        self.inner_to_outer_inner_s = (
            self.get_parameter("inner_to_outer_inner_s").get_parameter_value().double_value
        )
        self.inner_to_outer_outer_s = (
            self.get_parameter("inner_to_outer_outer_s").get_parameter_value().double_value
        )
        self.outer_to_inner_outer_s = (
            self.get_parameter("outer_to_inner_outer_s").get_parameter_value().double_value
        )
        self.outer_to_inner_inner_s = (
            self.get_parameter("outer_to_inner_inner_s").get_parameter_value().double_value
        )
        self.initial_xy = (
            self.get_parameter("initial_x").get_parameter_value().double_value,
            self.get_parameter("initial_y").get_parameter_value().double_value,
        )
        self.initial_yaw = self.get_parameter("initial_yaw").get_parameter_value().double_value

        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value
        self.publisher = self.create_publisher(Trajectory, output_topic, 1)
        self.marker_publisher = self.create_publisher(MarkerArray, marker_topic, 1)
        self.all_lanelet_centerlines = []
        self.trajectory_points = self.build_centerline(map_path)
        self.trajectory_speeds = self.build_speed_profile(self.trajectory_points)

        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / max(0.1, publish_rate), self.on_timer)
        self.get_logger().info(
            f"Publishing {len(self.trajectory_points)} centerline trajectory points from {map_path}; "
            f"speed range={min(self.trajectory_speeds):.2f}-{max(self.trajectory_speeds):.2f} m/s"
        )

    def build_centerline(self, map_path):
        root = ET.parse(map_path).getroot()
        nodes = {}
        ways = {}
        for node in root.findall("node"):
            tags = {tag.attrib["k"]: tag.attrib["v"] for tag in node.findall("tag")}
            if "local_x" in tags and "local_y" in tags:
                nodes[node.attrib["id"]] = (
                    float(tags["local_x"]),
                    float(tags["local_y"]),
                    float(tags.get("ele", "0.0")),
                )
        for way in root.findall("way"):
            ways[way.attrib["id"]] = [nd.attrib["ref"] for nd in way.findall("nd")]

        lanelets = []
        for relation in root.findall("relation"):
            tags = {tag.attrib["k"]: tag.attrib["v"] for tag in relation.findall("tag")}
            if tags.get("type") != "lanelet" or tags.get("subtype") != "road":
                continue
            members = {
                member.attrib.get("role"): member.attrib.get("ref")
                for member in relation.findall("member")
                if member.attrib.get("type") == "way"
            }
            left_id = members.get("left")
            right_id = members.get("right")
            if left_id not in ways or right_id not in ways:
                continue
            left = [nodes[node_id] for node_id in ways[left_id] if node_id in nodes]
            right = [nodes[node_id] for node_id in ways[right_id] if node_id in nodes]
            count = max(len(left), len(right), 2)
            left = resample_polyline(left, count)
            right = resample_polyline(right, count)
            center = [
                (
                    (left[i][0] + right[i][0]) * 0.5,
                    (left[i][1] + right[i][1]) * 0.5,
                    (left[i][2] + right[i][2]) * 0.5,
                )
                for i in range(count)
            ]
            lanelets.append({"id": relation.attrib["id"], "points": center})

        if not lanelets:
            raise RuntimeError(f"No road lanelets found in {map_path}")

        self.all_lanelet_centerlines = [lanelet["points"] for lanelet in lanelets]

        if self.route_mode == "initial_straight":
            return self.build_initial_straight_route(lanelets)
        if self.route_mode == "port_loop":
            return self.build_port_loop_route(lanelets)

        inner_ordered = self.build_lanelet_loop(lanelets)
        inner_ids = {lanelet["id"] for lanelet in inner_ordered}
        if self.route_mode == "dual_lane_change":
            outer_lanelets = [lanelet for lanelet in lanelets if lanelet["id"] not in inner_ids]
            if outer_lanelets:
                outer_start_index = min(
                    range(len(outer_lanelets)),
                    key=lambda i: distance(inner_ordered[0]["points"][0], outer_lanelets[i]["points"][0]),
                )
                outer_ordered = self.build_lanelet_loop(outer_lanelets, outer_start_index)
                return self.build_dual_lane_change_route(inner_ordered, outer_ordered)

        return self.build_single_loop_route(inner_ordered)

    def build_lanelet_loop(self, lanelets, start_index=None):
        remaining = list(lanelets)
        if start_index is None:
            start_index = min(
                range(len(remaining)), key=lambda i: distance(remaining[i]["points"][0], self.initial_xy)
            )
        ordered = [remaining.pop(start_index)]
        route_start = ordered[0]["points"][0]
        while remaining:
            tail = ordered[-1]["points"][-1]
            if distance(tail, route_start) < 1.0:
                break
            next_index = min(
                range(len(remaining)), key=lambda i: distance(tail, remaining[i]["points"][0])
            )
            if distance(tail, remaining[next_index]["points"][0]) > 5.0:
                break
            ordered.append(remaining.pop(next_index))
        return ordered

    def lanelets_to_points(self, ordered):
        raw_points = []
        for lanelet in ordered:
            if raw_points and distance(raw_points[-1], lanelet["points"][0]) < 0.1:
                raw_points.extend(lanelet["points"][1:])
            else:
                raw_points.extend(lanelet["points"])
        return raw_points

    def rotate_to_initial(self, points, spacing):
        if not points:
            return points
        nearest = min(range(len(points)), key=lambda i: distance(points[i], self.initial_xy))
        rotated = list(points[nearest:])
        if nearest > 0:
            rotated.extend(interpolate_gap(points[-1], points[0], spacing))
            rotated.extend(points[:nearest])
        return rotated

    def align_route_direction(self, points, spacing):
        if len(points) < 2:
            return points
        yaw = math.atan2(points[1][1] - points[0][1], points[1][0] - points[0][0])
        if abs(normalize_angle(yaw - self.initial_yaw)) <= math.pi * 0.5:
            return points

        reversed_points = list(reversed(points))
        aligned = self.rotate_to_initial(reversed_points, spacing)
        aligned_yaw = math.atan2(aligned[1][1] - aligned[0][1], aligned[1][0] - aligned[0][0])
        self.get_logger().info(
            "Reversed centerline route to match initial_yaw: route_yaw=%.1f deg, aligned_yaw=%.1f deg"
            % (math.degrees(yaw), math.degrees(aligned_yaw))
        )
        return aligned

    def build_initial_straight_route(self, lanelets):
        spacing = max(0.1, self.point_spacing)
        heading = (math.cos(self.initial_yaw), math.sin(self.initial_yaw))
        lateral = (-heading[1], heading[0])
        candidates = []
        for lanelet in lanelets:
            for point in lanelet["points"]:
                dx = point[0] - self.initial_xy[0]
                dy = point[1] - self.initial_xy[1]
                lateral_offset = abs(dx * lateral[0] + dy * lateral[1])
                longitudinal = dx * heading[0] + dy * heading[1]
                if lateral_offset <= 2.0 and longitudinal >= -0.5:
                    candidates.append((longitudinal, point))

        if len(candidates) < 2:
            self.get_logger().warn(
                "Failed to build initial_straight route; falling back to single loop route."
            )
            return self.build_single_loop_route(self.build_lanelet_loop(lanelets))

        candidates.sort(key=lambda item: item[0])
        max_longitudinal = max(item[0] for item in candidates)
        z = min(candidates, key=lambda item: abs(item[0]))[1][2]
        points = []
        sample_count = max(2, int(max_longitudinal / spacing) + 1)
        for index in range(sample_count):
            s = min(max_longitudinal, index * spacing)
            points.append(
                (
                    self.initial_xy[0] + heading[0] * s,
                    self.initial_xy[1] + heading[1] * s,
                    z,
                )
            )
        end_point = (
            self.initial_xy[0] + heading[0] * max_longitudinal,
            self.initial_xy[1] + heading[1] * max_longitudinal,
            z,
        )
        if distance(points[-1], end_point) > spacing * 0.25:
            points.append(end_point)
        self.get_logger().info(
            "Built initial_straight route with %d points, length %.1f m"
            % (len(points), cumulative_lengths(points)[-1])
        )
        return points

    def build_port_loop_route(self, lanelets):
        verticals = []
        horizontals = []
        for lanelet in lanelets:
            start = lanelet["points"][0]
            end = lanelet["points"][-1]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            if abs(dx) <= 1.0 and abs(dy) >= 15.0:
                verticals.append(
                    {
                        "x": 0.5 * (start[0] + end[0]),
                        "min_y": min(start[1], end[1]),
                        "max_y": max(start[1], end[1]),
                        "direction": math.copysign(1.0, dy),
                        "z": 0.5 * (start[2] + end[2]),
                    }
                )
            elif abs(dy) <= 1.0 and abs(dx) >= 50.0:
                horizontals.append(
                    {
                        "y": 0.5 * (start[1] + end[1]),
                        "min_x": min(start[0], end[0]),
                        "max_x": max(start[0], end[0]),
                        "z": 0.5 * (start[2] + end[2]),
                    }
                )

        if not verticals or len(horizontals) < 2:
            self.get_logger().warn(
                "Failed to infer the port loop lanes; falling back to initial_straight."
            )
            return self.build_initial_straight_route(lanelets)

        initial_direction = math.copysign(1.0, math.sin(self.initial_yaw))
        west_candidates = self.unique_lanes_by_coordinate(
            [
                lane
                for lane in verticals
                if lane["min_y"] <= self.initial_xy[1] <= lane["max_y"]
                and lane["direction"] == initial_direction
            ],
            "x",
        )
        if not west_candidates:
            west_candidates = self.unique_lanes_by_coordinate(
                [
                    lane
                    for lane in verticals
                    if lane["min_y"] <= self.initial_xy[1] <= lane["max_y"]
                ],
                "x",
            )
        west_candidates.sort(key=lambda lane: lane["x"])
        lane_offset = min(self.route_lane_index - 1, len(west_candidates) - 1)
        west = west_candidates[lane_offset]

        east_candidates = self.unique_lanes_by_coordinate(
            [
                lane
                for lane in verticals
                if lane["direction"] == -west["direction"]
                and abs(lane["min_y"] - west["min_y"]) <= 2.0
                and abs(lane["max_y"] - west["max_y"]) <= 2.0
                and lane["x"] > west["x"] + 20.0
            ],
            "x",
        )
        if not east_candidates:
            self.get_logger().warn(
                "Failed to find the return lane for port_loop; falling back to initial_straight."
            )
            return self.build_initial_straight_route(lanelets)
        east_candidates.sort(key=lambda lane: lane["x"], reverse=True)
        east = east_candidates[min(lane_offset, len(east_candidates) - 1)]

        spanning_horizontals = self.unique_lanes_by_coordinate(
            [
                lane
                for lane in horizontals
                if lane["min_x"] <= west["x"] + 1.0
                and lane["max_x"] >= east["x"] - 1.0
            ],
            "y",
        )
        lower_candidates = sorted(
            [
            lane for lane in spanning_horizontals if lane["y"] <= west["min_y"] + 0.5
            ],
            key=lambda lane: lane["y"],
            reverse=True,
        )
        upper_candidates = sorted(
            [
                lane
                for lane in spanning_horizontals
                if lane["y"] >= west["max_y"] - 0.5
            ],
            key=lambda lane: lane["y"],
        )
        if not lower_candidates or not upper_candidates:
            self.get_logger().warn(
                "Failed to find horizontal lanes for port_loop; falling back to initial_straight."
            )
            return self.build_initial_straight_route(lanelets)

        lower = lower_candidates[min(lane_offset, len(lower_candidates) - 1)]
        upper = upper_candidates[min(lane_offset, len(upper_candidates) - 1)]
        west_x = west["x"]
        east_x = east["x"]
        lower_y = lower["y"]
        upper_y = upper["y"]
        available_radius = min(
            0.45 * (east_x - west_x),
            0.45 * (upper_y - lower_y),
        )
        radius = max(2.0, min(self.route_turn_radius, available_radius))
        start_y = max(lower_y + radius, min(upper_y - radius, self.initial_xy[1]))
        z = 0.25 * (west["z"] + east["z"] + lower["z"] + upper["z"])
        spacing = max(0.1, self.point_spacing)

        points = [(west_x, start_y, z)]
        append_line(points, (west_x, lower_y + radius, z), spacing)
        append_arc(
            points,
            (west_x + radius, lower_y + radius),
            radius,
            math.pi,
            1.5 * math.pi,
            z,
            spacing,
        )
        append_line(points, (east_x - radius, lower_y, z), spacing)
        append_arc(
            points,
            (east_x - radius, lower_y + radius),
            radius,
            -0.5 * math.pi,
            0.0,
            z,
            spacing,
        )
        append_line(points, (east_x, upper_y - radius, z), spacing)
        append_arc(
            points,
            (east_x - radius, upper_y - radius),
            radius,
            0.0,
            0.5 * math.pi,
            z,
            spacing,
        )
        append_line(points, (west_x + radius, upper_y, z), spacing)
        append_arc(
            points,
            (west_x + radius, upper_y - radius),
            radius,
            0.5 * math.pi,
            math.pi,
            z,
            spacing,
        )
        append_line(points, (west_x, start_y, z), spacing)

        self.get_logger().info(
            "Built port_loop lane %d: west=%.1f, east=%.1f, lower=%.1f, upper=%.1f, "
            "radius=%.1f, length=%.1f m, points=%d"
            % (
                self.route_lane_index,
                west_x,
                east_x,
                lower_y,
                upper_y,
                radius,
                cumulative_lengths(points)[-1],
                len(points),
            )
        )
        return points

    @staticmethod
    def unique_lanes_by_coordinate(lanes, coordinate):
        unique = {}
        for lane in lanes:
            key = round(lane[coordinate], 2)
            if key not in unique:
                unique[key] = lane
        return list(unique.values())

    def build_single_loop_route(self, ordered):
        spacing = max(0.1, self.point_spacing)
        raw_points = self.lanelets_to_points(ordered)
        points = resample_by_spacing(raw_points, spacing)
        rotated = self.rotate_to_initial(points, spacing)
        aligned = self.align_route_direction(rotated, spacing)
        return smooth_path_points(aligned, self.path_smoothing_passes)

    def build_dual_lane_change_route(self, inner_ordered, outer_ordered):
        spacing = max(0.1, self.point_spacing)
        inner = resample_by_spacing(self.lanelets_to_points(inner_ordered), spacing)
        outer = resample_by_spacing(self.lanelets_to_points(outer_ordered), spacing)
        inner_lengths = cumulative_lengths(inner)
        outer_lengths = cumulative_lengths(outer)

        start_s = min(
            inner_lengths,
            key=lambda s: distance(sample_by_s(inner, inner_lengths, s), self.initial_xy),
        )
        change_length = max(4.0, self.lane_change_length)

        route = []
        route.extend(slice_ring(inner, inner_lengths, start_s, self.inner_to_outer_inner_s, spacing))
        route.extend(
            lane_change_curve(
                sample_by_s(inner, inner_lengths, self.inner_to_outer_inner_s),
                sample_by_s(
                    outer, outer_lengths, self.inner_to_outer_outer_s + change_length
                ),
                spacing,
            )[1:]
        )
        route.extend(
            slice_ring(
                outer,
                outer_lengths,
                self.inner_to_outer_outer_s + change_length,
                self.outer_to_inner_outer_s,
                spacing,
            )[1:]
        )
        route.extend(
            lane_change_curve(
                sample_by_s(outer, outer_lengths, self.outer_to_inner_outer_s),
                sample_by_s(
                    inner, inner_lengths, self.outer_to_inner_inner_s + change_length
                ),
                spacing,
            )[1:]
        )
        route.extend(
            slice_ring(
                inner,
                inner_lengths,
                self.outer_to_inner_inner_s + change_length,
                start_s + inner_lengths[-1],
                spacing,
            )[1:]
        )
        aligned = self.align_route_direction(route, spacing)
        return smooth_path_points(aligned, self.path_smoothing_passes)

    def build_speed_profile(self, points):
        if len(points) < 3:
            return [self.target_speed for _ in points]

        min_speed = max(0.1, min(self.min_speed, self.target_speed))
        max_speed = max(min_speed, self.target_speed)
        window = max(2, int(self.curvature_window))
        speeds = []
        turn_flags = []

        is_closed = len(points) > 3 and distance(points[0], points[-1]) < self.point_spacing
        unique_count = len(points) - 1 if is_closed else len(points)
        for i, point in enumerate(points):
            point_index = i % unique_count
            if is_closed:
                prev_point = points[(point_index - window) % unique_count]
                next_point = points[(point_index + window) % unique_count]
            else:
                prev_point = points[max(0, i - window)]
                next_point = points[min(len(points) - 1, i + window)]
            if prev_point == point or next_point == point:
                speeds.append(max_speed)
                continue

            yaw_prev = math.atan2(point[1] - prev_point[1], point[0] - prev_point[0])
            yaw_next = math.atan2(next_point[1] - point[1], next_point[0] - point[0])
            heading_change = abs(math.atan2(math.sin(yaw_next - yaw_prev), math.cos(yaw_next - yaw_prev)))
            arc_length = max(1e-3, distance(prev_point, point) + distance(point, next_point))
            curvature = 2.0 * heading_change / arc_length

            if curvature < 1e-4:
                speed = max_speed
            else:
                speed = math.sqrt(max(0.1, self.max_lateral_accel) / curvature)
            is_turn = curvature >= max(0.0, self.turn_curvature_threshold)
            turn_flags.append(is_turn)
            if is_turn:
                speed = min(speed, max(min_speed, self.turn_speed_limit))
            tangent_prev = points[(point_index - 1) % unique_count] if is_closed else points[max(0, i - 1)]
            tangent_next = points[(point_index + 1) % unique_count] if is_closed else points[min(len(points) - 1, i + 1)]
            tangent_dx = tangent_next[0] - tangent_prev[0]
            tangent_dy = tangent_next[1] - tangent_prev[1]
            if abs(tangent_dy) > abs(tangent_dx):
                speed = min(speed, max(min_speed, self.short_straight_speed_limit))
            speeds.append(max(min_speed, min(max_speed, speed)))

        if is_closed:
            turn_flags.append(turn_flags[0])

        hold_distance = max(0.0, self.turn_exit_hold_distance)
        if hold_distance > 0.0 and unique_count > 1:
            for i in range(unique_count):
                distance_from_turn = 0.0
                cursor = i
                while distance_from_turn <= hold_distance:
                    if turn_flags[cursor]:
                        speeds[i] = min(
                            speeds[i], max(min_speed, self.turn_speed_limit)
                        )
                        break
                    previous = (cursor - 1) % unique_count if is_closed else cursor - 1
                    if previous < 0:
                        break
                    distance_from_turn += distance(points[previous], points[cursor])
                    cursor = previous
            if is_closed:
                speeds[-1] = speeds[0]

        entry_hold_distance = max(0.0, self.turn_entry_hold_distance)
        if entry_hold_distance > 0.0 and unique_count > 1:
            for i in range(unique_count):
                distance_to_turn = 0.0
                cursor = i
                while distance_to_turn <= entry_hold_distance:
                    if turn_flags[cursor]:
                        speeds[i] = min(
                            speeds[i], max(min_speed, self.turn_speed_limit)
                        )
                        break
                    next_cursor = (cursor + 1) % unique_count if is_closed else cursor + 1
                    if next_cursor >= unique_count:
                        break
                    distance_to_turn += distance(points[cursor], points[next_cursor])
                    cursor = next_cursor
            if is_closed:
                speeds[-1] = speeds[0]

        for _ in range(max(0, int(self.speed_smoothing_passes))):
            smoothed = speeds[:]
            if is_closed:
                for i in range(unique_count):
                    smoothed[i] = (
                        0.25 * speeds[(i - 1) % unique_count]
                        + 0.5 * speeds[i]
                        + 0.25 * speeds[(i + 1) % unique_count]
                    )
                smoothed[-1] = smoothed[0]
            else:
                for i in range(1, len(speeds) - 1):
                    smoothed[i] = (
                        0.25 * speeds[i - 1] + 0.5 * speeds[i] + 0.25 * speeds[i + 1]
                    )
            speeds = [max(min_speed, min(max_speed, speed)) for speed in smoothed]

        segment_count = unique_count if is_closed else unique_count - 1
        if segment_count > 0:
            accel_limit = max(0.01, self.max_profile_accel)
            decel_limit = max(0.01, self.max_profile_decel)
            # Repeat around a closed loop so the acceleration envelope also crosses the seam.
            passes = 2 if is_closed else 1
            for _ in range(passes):
                for i in range(segment_count - 1, -1, -1):
                    next_i = (i + 1) % unique_count
                    ds = max(1e-3, distance(points[i], points[next_i]))
                    speeds[i] = min(
                        speeds[i],
                        math.sqrt(max(0.0, speeds[next_i] ** 2 + 2.0 * decel_limit * ds)),
                    )
                for i in range(segment_count):
                    next_i = (i + 1) % unique_count
                    ds = max(1e-3, distance(points[i], points[next_i]))
                    speeds[next_i] = min(
                        speeds[next_i],
                        math.sqrt(max(0.0, speeds[i] ** 2 + 2.0 * accel_limit * ds)),
                    )
            if is_closed:
                speeds[-1] = speeds[0]

        return speeds

    def on_timer(self):
        msg = Trajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        elapsed = 0.0
        for i, point in enumerate(self.trajectory_points):
            target_speed = self.trajectory_speeds[i]
            next_point = self.trajectory_points[min(i + 1, len(self.trajectory_points) - 1)]
            prev_point = self.trajectory_points[max(i - 1, 0)]
            yaw = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
            if i > 0:
                elapsed += distance(self.trajectory_points[i - 1], point) / max(
                    0.1, self.trajectory_speeds[max(0, i - 1)]
                )

            traj_point = TrajectoryPoint()
            traj_point.time_from_start = Duration(
                sec=int(elapsed), nanosec=int((elapsed - int(elapsed)) * 1e9)
            )
            traj_point.pose.position.x = point[0]
            traj_point.pose.position.y = point[1]
            traj_point.pose.position.z = point[2]
            traj_point.pose.orientation = yaw_to_quaternion(yaw)
            traj_point.longitudinal_velocity_mps = float(target_speed)
            traj_point.acceleration_mps2 = 0.0
            traj_point.heading_rate_rps = 0.0
            msg.points.append(traj_point)

        self.publisher.publish(msg)
        self.publish_marker(msg.header.stamp)

    def publish_marker(self, stamp):
        full_centerlines = Marker()
        full_centerlines.header.stamp = stamp
        full_centerlines.header.frame_id = self.frame_id
        full_centerlines.ns = "lanelet_trajectory_centerlines"
        full_centerlines.id = 0
        full_centerlines.type = Marker.LINE_LIST
        full_centerlines.action = Marker.ADD
        full_centerlines.pose.orientation.w = 1.0
        full_centerlines.scale.x = 0.12
        full_centerlines.color.r = 0.15
        full_centerlines.color.g = 1.0
        full_centerlines.color.b = 0.25
        full_centerlines.color.a = 0.95
        full_centerlines.lifetime.sec = 0

        for centerline in self.all_lanelet_centerlines:
            for first, second in zip(centerline, centerline[1:]):
                p0 = Point()
                p0.x = first[0]
                p0.y = first[1]
                p0.z = first[2] + 0.25
                p1 = Point()
                p1.x = second[0]
                p1.y = second[1]
                p1.z = second[2] + 0.25
                full_centerlines.points.append(p0)
                full_centerlines.points.append(p1)

        route = Marker()
        route.header.stamp = stamp
        route.header.frame_id = self.frame_id
        route.ns = "mpc_route"
        route.id = 1
        route.type = Marker.LINE_STRIP
        route.action = Marker.ADD
        route.pose.orientation.w = 1.0
        route.scale.x = 0.2
        route.color.r = 1.0
        route.color.g = 1.0
        route.color.b = 1.0
        route.color.a = 1.0
        route.lifetime.sec = 0

        for point in self.trajectory_points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2] + 0.35
            route.points.append(p)

        markers = MarkerArray()
        markers.markers.append(full_centerlines)
        markers.markers.append(route)
        self.marker_publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = CenterlineTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
