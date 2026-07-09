#!/usr/bin/env python3

import math
import pathlib
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(__file__).resolve().parents[1]
SOURCE = ROOT / "models" / "xiamen_site" / "model.sdf"
OUTPUT_DIR = ROOT / "models" / "xiamen_site_optimized" / "meshes"
OBJ_PATH = OUTPUT_DIR / "xiamen_site_merged.obj"
MTL_PATH = OUTPUT_DIR / "xiamen_site_merged.mtl"
LANE_OUTPUT_DIR = ROOT / "models" / "xiamen_lane_overlay" / "meshes"


def rotate(point, roll, pitch, yaw):
    x, y, z = point
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return (
        (cy * cp) * x + (cy * sp * sr - sy * cr) * y + (cy * sp * cr + sy * sr) * z,
        (sy * cp) * x + (sy * sp * sr + cy * cr) * y + (sy * sp * cr - cy * sr) * z,
        (-sp) * x + (cp * sr) * y + (cp * cr) * z,
    )


def main():
    root = ET.parse(SOURCE).getroot()
    vertices = []
    faces_by_material = {}
    colors = {}
    lane_meshes = {
        "white": {"vertices": [], "faces": []},
        "yellow": {"vertices": [], "faces": []},
    }

    cube_faces = (
        (0, 2, 3), (0, 3, 1),
        (4, 5, 7), (4, 7, 6),
        (0, 1, 5), (0, 5, 4),
        (2, 6, 7), (2, 7, 3),
        (0, 4, 6), (0, 6, 2),
        (1, 3, 7), (1, 7, 5),
    )

    for visual in root.findall(".//visual"):
        size_element = visual.find("./geometry/box/size")
        if size_element is None:
            continue

        size = [float(value) for value in size_element.text.split()]
        pose_element = visual.find("./pose")
        pose = [0.0] * 6 if pose_element is None else [
            float(value) for value in pose_element.text.split()
        ]
        while len(pose) < 6:
            pose.append(0.0)

        diffuse_element = visual.find("./material/diffuse")
        diffuse = (0.5, 0.5, 0.5)
        if diffuse_element is not None:
            values = [float(value) for value in diffuse_element.text.split()]
            diffuse = tuple(values[:3])
        color_key = tuple(round(value, 3) for value in diffuse)
        material_name = "material_" + "_".join(f"{value:.3f}" for value in color_key)
        colors[material_name] = color_key

        hx, hy, hz = (value * 0.5 for value in size)
        corners = (
            (-hx, -hy, -hz), (-hx, hy, -hz),
            (hx, -hy, -hz), (hx, hy, -hz),
            (-hx, -hy, hz), (-hx, hy, hz),
            (hx, -hy, hz), (hx, hy, hz),
        )
        start = len(vertices) + 1
        for corner in corners:
            x, y, z = rotate(corner, pose[3], pose[4], pose[5])
            vertices.append((x + pose[0], y + pose[1], z + pose[2]))

        material_faces = faces_by_material.setdefault(material_name, [])
        material_faces.extend(
            tuple(start + index for index in face) for face in cube_faces
        )

        if visual.get("name", "").startswith("line_"):
            lane_key = "yellow" if diffuse[0] > 0.9 and diffuse[1] < 0.9 else "white"
            lane_mesh = lane_meshes[lane_key]
            lane_start = len(lane_mesh["vertices"]) + 1
            for corner in corners:
                x, y, z = rotate(corner, pose[3], pose[4], pose[5])
                lane_mesh["vertices"].append((x + pose[0], y + pose[1], z + pose[2]))
            lane_mesh["faces"].extend(
                tuple(lane_start + index for index in face) for face in cube_faces
            )

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    with OBJ_PATH.open("w", encoding="ascii") as obj:
        obj.write("mtllib xiamen_site_merged.mtl\n")
        obj.write("o xiamen_site_merged\n")
        for x, y, z in vertices:
            obj.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        for material_name, faces in faces_by_material.items():
            obj.write(f"usemtl {material_name}\n")
            for a, b, c in faces:
                obj.write(f"f {a} {b} {c}\n")

    with MTL_PATH.open("w", encoding="ascii") as mtl:
        for material_name, color in colors.items():
            mtl.write(f"newmtl {material_name}\n")
            mtl.write(f"Ka {color[0]:.3f} {color[1]:.3f} {color[2]:.3f}\n")
            mtl.write(f"Kd {color[0]:.3f} {color[1]:.3f} {color[2]:.3f}\n")
            mtl.write("Ks 0.05 0.05 0.05\n")
            mtl.write("d 1.0\n\n")

    LANE_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for lane_key, lane_mesh in lane_meshes.items():
        lane_path = LANE_OUTPUT_DIR / f"xiamen_lanes_{lane_key}.obj"
        with lane_path.open("w", encoding="ascii") as obj:
            obj.write(f"o xiamen_lanes_{lane_key}\n")
            for x, y, z in lane_mesh["vertices"]:
                obj.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
            for a, b, c in lane_mesh["faces"]:
                obj.write(f"f {a} {b} {c}\n")

    print(
        f"Merged {len(vertices) // 8} visuals into {OBJ_PATH} "
        f"({len(vertices)} vertices)."
    )
    print(
        "Merged lane visuals into "
        + ", ".join(
            f"{key}={len(mesh['vertices']) // 8}" for key, mesh in lane_meshes.items()
        )
        + "."
    )


if __name__ == "__main__":
    main()
