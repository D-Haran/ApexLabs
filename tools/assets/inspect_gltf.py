#!/usr/bin/env python3
"""Dependency-free structural/statistical inspection of glTF, GLB, or a Sketchfab ZIP."""

from __future__ import annotations

import argparse
import json
import re
import struct
import tempfile
import zipfile
from pathlib import Path
from typing import Any


def _read_document(path: Path) -> dict[str, Any]:
    if path.suffix.lower() == ".gltf":
        return json.loads(path.read_text(encoding="utf-8"))
    data = path.read_bytes()
    if len(data) < 20 or data[:4] != b"glTF":
        raise ValueError(f"not a glTF 2.0 GLB: {path}")
    version, total_length = struct.unpack_from("<II", data, 4)
    if version != 2 or total_length != len(data):
        raise ValueError(f"unsupported or truncated GLB: {path}")
    offset = 12
    while offset + 8 <= len(data):
        length, chunk_type = struct.unpack_from("<II", data, offset)
        offset += 8
        chunk = data[offset : offset + length]
        offset += length
        if chunk_type == 0x4E4F534A:
            return json.loads(chunk.rstrip(b" \t\r\n\0"))
    raise ValueError(f"GLB has no JSON chunk: {path}")


def _find_model(root: Path) -> Path:
    models = sorted(
        [*root.rglob("*.glb"), *root.rglob("*.gltf")],
        key=lambda path: (path.name.lower() not in ("scene.gltf", "scene.glb"), len(path.parts), str(path)),
    )
    if not models:
        raise ValueError("archive/directory contains no .gltf or .glb")
    return models[0]


def inspect_model(path: Path) -> dict[str, Any]:
    document = _read_document(path)
    accessors = document.get("accessors", [])
    triangle_count = 0
    primitive_count = 0
    unsupported_primitives = 0
    for mesh in document.get("meshes", []):
        for primitive in mesh.get("primitives", []):
            primitive_count += 1
            mode = primitive.get("mode", 4)
            if mode != 4:
                unsupported_primitives += 1
                continue
            accessor_index = primitive.get("indices")
            if accessor_index is None:
                accessor_index = primitive.get("attributes", {}).get("POSITION")
            if accessor_index is not None:
                triangle_count += int(accessors[accessor_index].get("count", 0)) // 3
    node_names = [node.get("name", "") for node in document.get("nodes", []) if node.get("name")]
    wheel_pattern = re.compile(r"(?:wheel|rim|tire|tyre|front|rear|left|right|fl|fr|rl|rr)", re.I)
    body_pattern = re.compile(r"(?:body|shell|chassis|paint|exterior)", re.I)
    texture_bytes = 0
    external_files: list[str] = []
    for item in [*document.get("buffers", []), *document.get("images", [])]:
        uri = item.get("uri", "")
        if not uri or uri.startswith("data:"):
            continue
        candidate = path.parent / uri
        external_files.append(uri)
        if candidate.is_file():
            texture_bytes += candidate.stat().st_size
    return {
        "selected_model": str(path),
        "container_file_size_bytes": path.stat().st_size,
        "external_resource_size_bytes": texture_bytes,
        "measured_triangle_count": triangle_count,
        "primitive_draw_call_estimate": primitive_count,
        "non_triangle_primitive_count": unsupported_primitives,
        "mesh_count": len(document.get("meshes", [])),
        "node_count": len(document.get("nodes", [])),
        "material_count": len(document.get("materials", [])),
        "texture_count": len(document.get("textures", [])),
        "image_count": len(document.get("images", [])),
        "external_resources": external_files,
        "wheel_node_candidates": [name for name in node_names if wheel_pattern.search(name)],
        "body_node_candidates": [name for name in node_names if body_pattern.search(name)],
        "all_named_nodes": node_names,
    }


def inspect_input(path: Path) -> dict[str, Any]:
    if path.suffix.lower() == ".zip":
        with tempfile.TemporaryDirectory(prefix="apexlab-gltf-inspect-") as directory:
            with zipfile.ZipFile(path) as archive:
                archive.extractall(directory)
            report = inspect_model(_find_model(Path(directory)))
            report["selected_model"] = str(Path(report["selected_model"]).relative_to(directory))
            report["archive_file_size_bytes"] = path.stat().st_size
            return report
    return inspect_model(_find_model(path) if path.is_dir() else path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = inspect_input(args.input)
    text = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
