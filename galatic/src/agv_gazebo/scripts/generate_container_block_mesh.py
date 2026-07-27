#!/usr/bin/env python3
"""Generate the batched 3 x 6 x 2 container-yard mesh."""

from pathlib import Path


BOX_SIZE = (12.23, 2.45, 2.59)
X_CENTERS = (6.115, 18.345, 30.575)
Y_CENTERS = (-6.25, -3.75, -1.25, 1.25, 3.75, 6.25)
Z_CENTERS = (1.295, 3.885)


def box_vertices(cx: float, cy: float, cz: float):
    hx, hy, hz = (value / 2.0 for value in BOX_SIZE)
    return (
        (cx - hx, cy - hy, cz - hz),
        (cx + hx, cy - hy, cz - hz),
        (cx + hx, cy + hy, cz - hz),
        (cx - hx, cy + hy, cz - hz),
        (cx - hx, cy - hy, cz + hz),
        (cx + hx, cy - hy, cz + hz),
        (cx + hx, cy + hy, cz + hz),
        (cx - hx, cy + hy, cz + hz),
    )


def write_dae(path: Path) -> None:
    face_specs = (
        ((0, 3, 2, 1), (0.0, 0.0, -1.0)),
        ((4, 5, 6, 7), (0.0, 0.0, 1.0)),
        ((0, 1, 5, 4), (0.0, -1.0, 0.0)),
        ((1, 2, 6, 5), (1.0, 0.0, 0.0)),
        ((2, 3, 7, 6), (0.0, 1.0, 0.0)),
        ((3, 0, 4, 7), (-1.0, 0.0, 0.0)),
    )
    positions = []
    normals = []
    triangles = []

    for y in Y_CENTERS:
        for x in X_CENTERS:
            for z in Z_CENTERS:
                corners = box_vertices(x, y, z)
                for corner_indices, normal in face_specs:
                    face_start = len(positions)
                    positions.extend(corners[index] for index in corner_indices)
                    normals.extend((normal,) * 4)
                    triangles.extend(
                        (
                            face_start,
                            face_start,
                            face_start + 1,
                            face_start + 1,
                            face_start + 2,
                            face_start + 2,
                            face_start,
                            face_start,
                            face_start + 2,
                            face_start + 2,
                            face_start + 3,
                            face_start + 3,
                        )
                    )

    position_values = " ".join(f"{value:.6f}" for point in positions for value in point)
    normal_values = " ".join(f"{value:.1f}" for point in normals for value in point)
    triangle_values = " ".join(str(index) for index in triangles)
    vertex_count = len(positions)
    triangle_count = len(triangles) // 6

    dae = f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor><authoring_tool>generate_container_block_mesh.py</authoring_tool></contributor>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_effects>
    <effect id="container-red-effect">
      <profile_COMMON>
        <technique sid="common">
          <lambert>
            <emission><color>0 0 0 1</color></emission>
            <ambient><color>0.10 0.02 0.015 1</color></ambient>
            <diffuse><color>0.32 0.055 0.045 1</color></diffuse>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>
  <library_materials>
    <material id="container-red-material" name="container_red">
      <instance_effect url="#container-red-effect"/>
    </material>
  </library_materials>
  <library_geometries>
    <geometry id="container-block-geometry" name="container_block_3x6x2">
      <mesh>
        <source id="container-block-positions">
          <float_array id="container-block-positions-array" count="{vertex_count * 3}">{position_values}</float_array>
          <technique_common>
            <accessor source="#container-block-positions-array" count="{vertex_count}" stride="3">
              <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="container-block-normals">
          <float_array id="container-block-normals-array" count="{vertex_count * 3}">{normal_values}</float_array>
          <technique_common>
            <accessor source="#container-block-normals-array" count="{vertex_count}" stride="3">
              <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="container-block-vertices">
          <input semantic="POSITION" source="#container-block-positions"/>
        </vertices>
        <triangles material="container-red-symbol" count="{triangle_count}">
          <input semantic="VERTEX" source="#container-block-vertices" offset="0"/>
          <input semantic="NORMAL" source="#container-block-normals" offset="1"/>
          <p>{triangle_values}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="container-block-node" name="container_block_3x6x2" type="NODE">
        <instance_geometry url="#container-block-geometry">
          <bind_material>
            <technique_common>
              <instance_material symbol="container-red-symbol" target="#container-red-material"/>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
'''
    path.write_text(dae, encoding="utf-8")


def main() -> None:
    output = (
        Path(__file__).resolve().parents[1]
        / "models"
        / "container_block_3x6x2"
        / "meshes"
        / "container_block.dae"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    write_dae(output)


if __name__ == "__main__":
    main()
