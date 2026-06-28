#!/usr/bin/env python3
import argparse
import math
import os
import xml.etree.ElementTree as ET


ROAD_Z = 0.005
LINE_Z = 0.035
ROAD_THICKNESS = 0.02
LINE_THICKNESS = 0.025
ROAD_MARGIN = 0.35
LINE_WIDTH = 0.08


def point_at_distance(points, target):
    if not points:
        return (0.0, 0.0)
    if target <= 0.0:
        return points[0]

    remaining = target
    for start, end in zip(points, points[1:]):
        seg_len = distance(start, end)
        if seg_len <= 1e-9:
            continue
        if remaining <= seg_len:
            t = remaining / seg_len
            return (
                start[0] + (end[0] - start[0]) * t,
                start[1] + (end[1] - start[1]) * t,
            )
        remaining -= seg_len
    return points[-1]


def polyline_length(points):
    return sum(distance(start, end) for start, end in zip(points, points[1:]))


def resample(points, count):
    if count <= 1:
        return points[:1]
    length = polyline_length(points)
    if length <= 1e-9:
        return [points[0]] * count
    return [point_at_distance(points, length * i / (count - 1)) for i in range(count)]


def align_boundaries(left, right):
    same = distance(left[0], right[0]) + distance(left[-1], right[-1])
    reversed_right = distance(left[0], right[-1]) + distance(left[-1], right[0])
    if reversed_right < same:
        return left, list(reversed(right))
    return left, right


def distance(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def segment_pose(start, end, z):
    mid_x = (start[0] + end[0]) * 0.5
    mid_y = (start[1] + end[1]) * 0.5
    yaw = math.atan2(end[1] - start[1], end[0] - start[0])
    length = max(distance(start, end), 0.01)
    return mid_x, mid_y, z, yaw, length


def fmt(value):
    return f"{value:.4f}".rstrip("0").rstrip(".")


def parse_osm(path):
    root = ET.parse(path).getroot()

    nodes = {}
    for node in root.findall("node"):
        tags = {tag.get("k"): tag.get("v") for tag in node.findall("tag")}
        if "local_x" not in tags or "local_y" not in tags:
            continue
        nodes[node.get("id")] = (float(tags["local_x"]), float(tags["local_y"]))

    ways = {}
    way_tags = {}
    for way in root.findall("way"):
        refs = [nd.get("ref") for nd in way.findall("nd")]
        points = [nodes[ref] for ref in refs if ref in nodes]
        if len(points) >= 2:
            ways[way.get("id")] = points
            way_tags[way.get("id")] = {
                tag.get("k"): tag.get("v") for tag in way.findall("tag")
            }

    lanelets = []
    for relation in root.findall("relation"):
        tags = {tag.get("k"): tag.get("v") for tag in relation.findall("tag")}
        if tags.get("type") != "lanelet":
            continue
        members = {member.get("role"): member.get("ref") for member in relation.findall("member")}
        left = ways.get(members.get("left"))
        right = ways.get(members.get("right"))
        if left and right:
            lanelets.append((relation.get("id"), left, right))

    return nodes, ways, way_tags, lanelets


def material(name, ambient, diffuse):
    return f"""      <material>
        <ambient>{ambient}</ambient>
        <diffuse>{diffuse}</diffuse>
      </material>
"""


def box_visual(name, pose, size, mat):
    x, y, z, yaw = pose
    sx, sy, sz = size
    return f"""    <visual name="{name}">
      <pose>{fmt(x)} {fmt(y)} {fmt(z)} 0 0 {fmt(yaw)}</pose>
      <geometry>
        <box>
          <size>{fmt(sx)} {fmt(sy)} {fmt(sz)}</size>
        </box>
      </geometry>
{mat}    </visual>
"""


def box_collision(name, pose, size):
    x, y, z, yaw = pose
    sx, sy, sz = size
    return f"""    <collision name="{name}">
      <pose>{fmt(x)} {fmt(y)} {fmt(z)} 0 0 {fmt(yaw)}</pose>
      <geometry>
        <box>
          <size>{fmt(sx)} {fmt(sy)} {fmt(sz)}</size>
        </box>
      </geometry>
    </collision>
"""


def road_segments(lanelets):
    for lanelet_id, left, right in lanelets:
        left, right = align_boundaries(left, right)
        sample_count = max(len(left), len(right), 2)
        left_samples = resample(left, sample_count)
        right_samples = resample(right, sample_count)
        centers = [
            ((left_point[0] + right_point[0]) * 0.5, (left_point[1] + right_point[1]) * 0.5)
            for left_point, right_point in zip(left_samples, right_samples)
        ]
        widths = [
            distance(left_point, right_point) + ROAD_MARGIN
            for left_point, right_point in zip(left_samples, right_samples)
        ]

        for index, (start, end) in enumerate(zip(centers, centers[1:])):
            if distance(start, end) < 0.05:
                continue
            x, y, z, yaw, length = segment_pose(start, end, ROAD_Z)
            width = max((widths[index] + widths[index + 1]) * 0.5, 0.5)
            yield lanelet_id, index, (x, y, z, yaw), (length + 0.18, width, ROAD_THICKNESS)


def line_segments(ways, way_tags):
    for way_id, points in ways.items():
        tags = way_tags.get(way_id, {})
        subtype = tags.get("subtype", "solid")
        for index, (start, end) in enumerate(zip(points, points[1:])):
            if distance(start, end) < 0.05:
                continue
            x, y, z, yaw, length = segment_pose(start, end, LINE_Z)
            yield way_id, index, subtype, (x, y, z, yaw), (length, LINE_WIDTH, LINE_THICKNESS)


def generate_sdf(osm_path):
    nodes, ways, way_tags, lanelets = parse_osm(osm_path)
    if not nodes:
        raise RuntimeError(f"No local_x/local_y nodes found in {osm_path}")

    xs = [point[0] for point in nodes.values()]
    ys = [point[1] for point in nodes.values()]
    road_mat = material("road", "0.16 0.17 0.17 1", "0.18 0.19 0.19 1")
    solid_mat = material("solid_line", "0.85 0.85 0.78 1", "0.95 0.95 0.86 1")
    dashed_mat = material("dashed_line", "0.85 0.74 0.18 1", "0.95 0.84 0.22 1")

    parts = [
        "<?xml version=\"1.0\"?>\n",
        "<sdf version=\"1.9\">\n",
        "  <model name=\"xiamen_site\">\n",
        "    <static>true</static>\n",
        "    <link name=\"map_link\">\n",
    ]

    for lanelet_id, index, pose, size in road_segments(lanelets):
        name = f"lanelet_{lanelet_id}_{index}"
        parts.append(box_visual(f"{name}_visual", pose, size, road_mat))

    for way_id, index, subtype, pose, size in line_segments(ways, way_tags):
        mat = dashed_mat if subtype == "dashed" else solid_mat
        parts.append(box_visual(f"line_{way_id}_{index}_{subtype}", pose, size, mat))

    parts.extend(
        [
            "    </link>\n",
            "  </model>\n",
            "</sdf>\n",
            f"<!-- bounds: x {fmt(min(xs))}..{fmt(max(xs))}, y {fmt(min(ys))}..{fmt(max(ys))} -->\n",
        ]
    )
    return "".join(parts), len(lanelets), len(ways)


def main():
    parser = argparse.ArgumentParser(
        description="Generate a simplified Gazebo SDF model from the Xiamen Lanelet2 OSM map."
    )
    parser.add_argument("--osm", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    sdf, lanelet_count, way_count = generate_sdf(args.osm)
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as out:
        out.write(sdf)
    print(f"Generated {args.output} from {lanelet_count} lanelets and {way_count} ways")


if __name__ == "__main__":
    main()
