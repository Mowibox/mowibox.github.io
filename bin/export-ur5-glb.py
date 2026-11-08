#!/usr/bin/env python3
# The UR5 of example-robot-data baked into assets/models/ur5.glb for assets/js/ur5.js.
#
#   python3 bin/export-ur5-glb.py ../example-robot-data/robots/ur_description
#
# Nodes as in chroma's chromapi.glb: `<joint>_origin` (fixed) > `<joint>` (turned by
# the site) > the part's meshes, merged by colour. No normals: the site creases them.
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import trimesh
from trimesh.transformations import euler_matrix, rotation_matrix, translation_matrix
from trimesh.visual.material import PBRMaterial
from trimesh.visual.texture import TextureVisuals

DESC = Path(sys.argv[1] if len(sys.argv) > 1 else "../example-robot-data/robots/ur_description")
OUT = Path(__file__).resolve().parent.parent / "assets/models/ur5.glb"
HALF_PI = np.pi / 2

# ur5_robot.urdf: joint, parent's frame -> joint (xyz, pitch), child's mesh.
CHAIN = [
    ("shoulder_pan_joint", (0, 0, 0.089159), 0, "shoulder"),
    ("shoulder_lift_joint", (0, 0.13585, 0), HALF_PI, "upperarm"),
    ("elbow_joint", (0, -0.1197, 0.425), 0, "forearm"),
    ("wrist_1_joint", (0, 0, 0.39225), HALF_PI, "wrist1"),
    ("wrist_2_joint", (0, 0.093, 0), 0, "wrist2"),
    ("wrist_3_joint", (0, 0, 0.09465), 0, "wrist3"),
]


def add_link(scene, parent, part):
    """The part's DAE under `parent`, one mesh per colour (welded, positions only)."""
    by_color = {}
    for mesh in trimesh.load(DESC / f"meshes/ur5/visual/{part}.dae", force="scene").dump():
        by_color.setdefault(tuple(mesh.visual.material.main_color[:3]), []).append(trimesh.Trimesh(mesh.vertices, mesh.faces))
    for i, (color, meshes) in enumerate(by_color.items()):
        material = PBRMaterial(baseColorFactor=(*(c / 255 for c in color), 1), roughnessFactor=0.55, metallicFactor=0.05)
        mesh = trimesh.util.concatenate(meshes)
        mesh.visual = TextureVisuals(material=material)
        scene.add_geometry(mesh, node_name=f"{part}_{i}", geom_name=f"{part}_{i}", parent_node_name=parent)


scene = trimesh.Scene()
# URDF is Z-up, glTF Y-up.
scene.graph.update(frame_from="world", frame_to="ur5", matrix=rotation_matrix(-HALF_PI, (1, 0, 0)))
add_link(scene, "ur5", "base")
parent = "ur5"
for joint, xyz, pitch, part in CHAIN:
    origin = translation_matrix(xyz) @ euler_matrix(0, pitch, 0)
    scene.graph.update(frame_from=parent, frame_to=f"{joint}_origin", matrix=origin)
    scene.graph.update(frame_from=f"{joint}_origin", frame_to=joint, matrix=np.eye(4))
    add_link(scene, joint, part)
    parent = joint

OUT.parent.mkdir(exist_ok=True)
with tempfile.TemporaryDirectory() as tmp:
    raw, simp = f"{tmp}/raw.glb", f"{tmp}/simp.glb"
    Path(raw).write_bytes(scene.export(file_type="glb", include_normals=False))
    gltf = ["npx", "-y", "@gltf-transform/cli@4.2.1"]
    subprocess.run([*gltf, "simplify", raw, simp, "--ratio", "0.3", "--error", "0.0005"], check=True)
    subprocess.run([*gltf, "meshopt", simp, str(OUT)], check=True)
print(OUT, OUT.stat().st_size)
