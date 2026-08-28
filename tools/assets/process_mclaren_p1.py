#!/usr/bin/env python3
"""Inspect, normalize, optimize, and measure the licensed McLaren P1 asset."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import zipfile
from datetime import datetime, timezone
from pathlib import Path

from inspect_gltf import inspect_input, inspect_model


REPOSITORY = Path(__file__).resolve().parents[2]
ASSET_ROOT = REPOSITORY / "assets/external/mclaren-p1"
DEFAULT_ARCHIVE = ASSET_ROOT / "raw/mclaren-p1-sketchfab.zip"
MANIFEST = ASSET_ROOT / "asset-manifest.json"


def _safe_extract(archive_path: Path, destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    root = destination.resolve()
    with zipfile.ZipFile(archive_path) as archive:
        for member in archive.infolist():
            target = (destination / member.filename).resolve()
            if root not in target.parents and target != root:
                raise RuntimeError(f"archive contains an unsafe path: {member.filename}")
        archive.extractall(destination)


def _find_model(root: Path) -> Path:
    models = sorted(
        [*root.rglob("*.glb"), *root.rglob("*.gltf")],
        key=lambda path: (path.name.lower() not in ("scene.gltf", "scene.glb"), len(path.parts), str(path)),
    )
    if not models:
        raise RuntimeError("download archive contains no glTF or GLB")
    return models[0]


def _required_import_fields(manifest: dict) -> None:
    settings = manifest["import"]
    missing = [
        key
        for key in ("original_units", "source_forward_axis", "source_up_axis", "import_scale", "body_node_mapping")
        if settings.get(key) is None
    ]
    if missing:
        raise RuntimeError(
            "inspection is complete, but the manifest still needs verified values for: "
            + ", ".join(missing)
        )
    valid_axes = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"}
    if settings["source_forward_axis"] not in valid_axes or settings["source_up_axis"] not in valid_axes:
        raise RuntimeError("source axes must use one of +X, -X, +Y, -Y, +Z, -Z")
    if settings["source_forward_axis"][-1] == settings["source_up_axis"][-1]:
        raise RuntimeError("source forward and up axes cannot be parallel")


def _command(name: str) -> str:
    executable = shutil.which(name)
    if not executable:
        raise RuntimeError(f"required tool is not installed: {name}")
    return executable


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--inspect-only", action="store_true")
    parser.add_argument("--texture-size", type=int)
    parser.add_argument("--simplify-ratio", type=float)
    parser.add_argument("--simplify-error", type=float)
    args = parser.parse_args()
    if not args.archive.is_file():
        raise RuntimeError(
            f"source archive not found: {args.archive}\n"
            "Complete the single authenticated download step in assets/external/mclaren-p1/README.md."
        )

    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    reports = ASSET_ROOT / "reports"
    reports.mkdir(parents=True, exist_ok=True)
    source_report = inspect_input(args.archive)
    (reports / "source-inspection.json").write_text(
        json.dumps(source_report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(source_report, indent=2))
    if args.inspect_only:
        return 0

    _required_import_fields(manifest)
    blender = _command("blender")
    transform = _command("gltf-transform")
    work = ASSET_ROOT / "work"
    source_root = work / "source"
    if source_root.exists():
        shutil.rmtree(source_root)
    _safe_extract(args.archive, source_root)
    source_model = _find_model(source_root)
    normalized = work / "mclaren-p1-normalized.glb"
    production = ASSET_ROOT / "processed/mclaren-p1-production.glb"
    production.parent.mkdir(parents=True, exist_ok=True)

    subprocess.run(
        [
            blender,
            "--background",
            "--python",
            str(Path(__file__).with_name("blender_prepare_mclaren_p1.py")),
            "--",
            str(source_model),
            str(normalized),
            str(MANIFEST),
        ],
        check=True,
    )
    processing = manifest["processing"]
    texture_size = args.texture_size or int(processing["maximum_texture_dimension"])
    simplify_ratio = args.simplify_ratio or float(processing["simplification_ratio"])
    simplify_error = args.simplify_error or float(processing["simplification_error"])
    subprocess.run(
        [
            transform,
            "optimize",
            str(normalized),
            str(production),
            "--compress", "meshopt",
            "--texture-compress", "ktx2",
            "--texture-size", str(texture_size),
            "--simplify", "true",
            "--simplify-ratio", str(simplify_ratio),
            "--simplify-error", str(simplify_error),
            "--flatten", "false",
            "--join", "false",
        ],
        check=True,
    )

    processed_report = inspect_model(production)
    (reports / "processed-inspection.json").write_text(
        json.dumps(processed_report, indent=2) + "\n", encoding="utf-8"
    )
    measured = manifest["measured_statistics"]
    measured.update(
        {
            "source_triangle_count": source_report["measured_triangle_count"],
            "processed_triangle_count": processed_report["measured_triangle_count"],
            "source_file_size_bytes": args.archive.stat().st_size,
            "processed_file_size_bytes": production.stat().st_size,
            "source_draw_call_estimate": source_report["primitive_draw_call_estimate"],
            "processed_draw_call_estimate": processed_report["primitive_draw_call_estimate"],
            "gpu_render_impact": "not measured by preprocessing; measure in the Phase B runtime",
        }
    )
    manifest["source_geometry_metadata"]["source_file_size_bytes"] = args.archive.stat().st_size
    manifest["processing"]["processed_at"] = datetime.now(timezone.utc).isoformat()
    manifest["status"] = "processed_pending_visual_validation"
    runtime = REPOSITORY / processing["runtime_glb"]
    runtime.parent.mkdir(parents=True, exist_ok=True)
    runtime.unlink(missing_ok=True)
    try:
        runtime.hardlink_to(production)
    except OSError:
        shutil.copy2(production, runtime)
    MANIFEST.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(processed_report, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, subprocess.CalledProcessError, zipfile.BadZipFile) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
