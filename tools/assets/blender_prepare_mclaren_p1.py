"""Blender background script used by process_mclaren_p1.py.

Axes in the manifest describe the model as it appears after Blender's glTF importer.
The script maps those axes to Blender +X forward/+Z up. Blender's glTF exporter then
produces ApexLab's +X forward/+Y up/+Z right convention.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Euler, Matrix, Vector


AXES = {
    "+X": Vector((1.0, 0.0, 0.0)),
    "-X": Vector((-1.0, 0.0, 0.0)),
    "+Y": Vector((0.0, 1.0, 0.0)),
    "-Y": Vector((0.0, -1.0, 0.0)),
    "+Z": Vector((0.0, 0.0, 1.0)),
    "-Z": Vector((0.0, 0.0, -1.0)),
}


def basis(forward: Vector, up: Vector) -> Matrix:
    if abs(forward.dot(up)) > 1.0e-6:
        raise ValueError("forward and up axes must be perpendicular")
    side = forward.cross(up)
    return Matrix((forward, up, side)).transposed()


def main() -> None:
    arguments = sys.argv[sys.argv.index("--") + 1 :]
    if len(arguments) != 3:
        raise SystemExit("expected: INPUT OUTPUT MANIFEST")
    source, output, manifest_path = map(Path, arguments)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    settings = manifest["import"]

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=str(source))
    imported = list(bpy.context.scene.objects)
    if not imported:
        raise RuntimeError("glTF import produced no objects")

    source_basis = basis(AXES[settings["source_forward_axis"]], AXES[settings["source_up_axis"]])
    target_basis = basis(AXES["+X"], AXES["+Z"])
    axis_correction = (target_basis @ source_basis.inverted()).to_4x4()
    extra = settings["transform_corrections"]["additional_rotation_deg_xyz"]
    extra_rotation = Euler(tuple(math.radians(float(value)) for value in extra), "XYZ").to_matrix().to_4x4()
    scale = Matrix.Scale(float(settings["import_scale"]), 4)
    translation = Matrix.Translation(Vector(settings["transform_corrections"]["translation_m"]))
    correction = translation @ extra_rotation @ axis_correction @ scale

    # Apply only to hierarchy roots, preserving named children and separable wheel nodes.
    for obj in imported:
        if obj.parent is None:
            obj.matrix_world = correction @ obj.matrix_world

    for material in bpy.data.materials:
        material.use_nodes = True

    output.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.gltf(
        filepath=str(output),
        export_format="GLB",
        export_yup=True,
        export_apply=False,
        export_materials="EXPORT",
        export_cameras=False,
        export_lights=False,
    )


if __name__ == "__main__":
    main()

