#!/usr/bin/env python3
"""Deterministic static glTF -> semantic vehicle GLB. No node-number semantics.
Requires numpy/scipy/Pillow; see requirements.txt. Scene-space axis overrides are explicit.
"""
from __future__ import annotations
import argparse, copy, hashlib, io, json, struct
from pathlib import Path
from urllib.parse import unquote
import numpy as np
from PIL import Image
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial.transform import Rotation

DTYPES = {5120: "i1", 5121: "u1", 5122: "<i2", 5123: "<u2", 5125: "<u4", 5126: "<f4"}
WIDTHS = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}
WHEELS = ("FL", "FR", "RL", "RR")


def dependency(root, uri):
    p = (root / unquote(uri)).resolve()
    if root.resolve() not in p.parents or not p.is_file():
        raise ValueError(f"Missing or unsafe GLTF dependency: {uri}")
    return p


class Source:
    def __init__(self, path):
        self.path = Path(path)
        self.d = json.loads(self.path.read_text())
        if self.d.get("skins") or self.d.get("animations"):
            raise ValueError("Only static source scenes are supported")
        self.buffers = [
            dependency(self.path.parent, b["uri"]).read_bytes()
            for b in self.d["buffers"]
        ]
        for i in self.d.get("images", []):
            dependency(self.path.parent, i["uri"])

    def accessor(self, index):
        a = self.d["accessors"][index]
        if "sparse" in a:
            raise ValueError("Sparse accessor unsupported; expand before processing")
        b = self.d["bufferViews"][a["bufferView"]]
        dt = np.dtype(DTYPES[a["componentType"]])
        n = WIDTHS[a["type"]]
        return np.ndarray(
            (a["count"], n),
            dtype=dt,
            buffer=self.buffers[b["buffer"]],
            offset=b.get("byteOffset", 0) + a.get("byteOffset", 0),
            strides=(b.get("byteStride", dt.itemsize * n), dt.itemsize),
        ).copy()

    def primitives(self):
        result = []

        def visit(i, parent):
            node = self.d["nodes"][i]
            m = (
                np.array(node.get("matrix", np.eye(4).T.flatten()), dtype=float)
                .reshape(4, 4)
                .T
            )
            if "rotation" in node:
                m[:3, :3] = Rotation.from_quat(node["rotation"]).as_matrix()
            if "scale" in node:
                m[:3, :3] = m[:3, :3] @ np.diag(node["scale"])
            if "translation" in node:
                m[:3, 3] = node["translation"]
            m = parent @ m
            if "mesh" in node:
                for p in self.d["meshes"][node["mesh"]]["primitives"]:
                    if p.get("mode", 4) != 4 or p.get("targets"):
                        raise ValueError("Only static triangle primitives supported")
                    attrs = {k: self.accessor(v) for k, v in p["attributes"].items()}
                    attrs["POSITION"] = attrs["POSITION"] @ m[:3, :3].T + m[:3, 3]
                    for k in ("NORMAL", "TANGENT"):
                        if k in attrs:
                            v = attrs[k][:, :3] @ np.linalg.inv(m[:3, :3])
                            v /= np.maximum(
                                np.linalg.norm(v, axis=1, keepdims=True), 1e-12
                            )
                            attrs[k][:, :3] = v
                    indices = (
                        self.accessor(p["indices"]).reshape(-1, 3)
                        if "indices" in p
                        else np.arange(len(attrs["POSITION"])).reshape(-1, 3)
                    )
                    if np.linalg.det(m[:3, :3]) < 0:
                        indices = indices[:, [0, 2, 1]]
                    result.append(
                        {
                            "attrs": attrs,
                            "indices": indices,
                            "material": p["material"],
                            "sourceNode": node.get("name", str(i)),
                        }
                    )
            for child in node.get("children", []):
                visit(child, m)

        for i in self.d["scenes"][self.d.get("scene", 0)]["nodes"]:
            visit(i, np.eye(4))
        return result


def components(positions, triangles, tolerance=1e-5):
    """Weld UV/material seams, then compute true triangle/vertex connectivity."""
    _, inverse = np.unique(
        np.round(positions / tolerance).astype(np.int64), axis=0, return_inverse=True
    )
    t = inverse[triangles]
    a = t[:, [0, 1, 2]].ravel()
    b = t[:, [1, 2, 0]].ravel()
    graph = coo_matrix(
        (np.ones(len(a)), (a, b)), shape=(int(inverse.max()) + 1,) * 2
    ).tocsr()
    _, labels = connected_components(graph, directed=False)
    vertex_labels = labels[inverse]
    return [
        np.flatnonzero(vertex_labels[triangles[:, 0]] == label)
        for label in np.unique(vertex_labels[triangles])
    ]


def identify_wheels(primitives, materials):
    fragments = []
    for p in primitives:
        if (
            "tyre" not in materials[p["material"]]["name"].lower()
            and "tire" not in materials[p["material"]]["name"].lower()
        ):
            continue
        for tri in components(p["attrs"]["POSITION"], p["indices"]):
            points = p["attrs"]["POSITION"][np.unique(p["indices"][tri])]
            if len(tri) > 20:
                fragments.append(points)
    if not fragments:
        raise ValueError("No connected tire geometry found")
    # Separate sidewalls/tread components can share a wheel. Cluster their geometric bbox centers,
    # never vertex index order; radius-scaled merge threshold cannot merge neighboring wheels.
    groups = []
    for points in sorted(fragments, key=lambda p: tuple((p.min(0) + p.max(0)) / 2)):
        center = (points.min(0) + points.max(0)) / 2
        match = next(
            (
                g
                for g in groups
                if np.linalg.norm(center - g["center"])
                < 0.30 * max(np.ptp(points, axis=0))
            ),
            None,
        )
        if match is None:
            groups.append({"center": center, "points": points})
        else:
            match["points"] = np.concatenate((match["points"], points))
            v = match["points"]
            match["center"] = (v.min(0) + v.max(0)) / 2
    if len(groups) != 4:
        raise ValueError(
            f"Ambiguous wheel classification: found {len(groups)} tire groups, expected 4. Explicit geometry override required."
        )
    return groups


class Writer:
    def __init__(self, source):
        self.d = {
            "asset": {
                "version": "2.0",
                "generator": "ApexLab semantic car processor v1",
                "extras": source.d["asset"].get("extras", {}),
            },
            "scene": 0,
            "scenes": [{"nodes": [0]}],
            "nodes": [{"name": "VEHICLE", "children": []}],
            "meshes": [],
            "materials": copy.deepcopy(source.d["materials"]),
            "textures": copy.deepcopy(source.d.get("textures", [])),
            "samplers": copy.deepcopy(source.d.get("samplers", [])),
            "images": [],
            "buffers": [{}],
            "bufferViews": [],
            "accessors": [],
        }
        if source.d.get("extensionsUsed"):
            self.d["extensionsUsed"] = source.d["extensionsUsed"]
        self.data = bytearray()

    def blob(self, data):
        self.data.extend(b"\0" * (-len(self.data) % 4))
        i = len(self.d["bufferViews"])
        self.d["bufferViews"].append(
            {"buffer": 0, "byteOffset": len(self.data), "byteLength": len(data)}
        )
        self.data.extend(data)
        return i

    def accessor(self, a, indices=False):
        a = np.ascontiguousarray(a, dtype="<u4" if indices else "<f4")
        n = 1 if a.ndim == 1 else a.shape[1]
        i = len(self.d["accessors"])
        v = {
            "bufferView": self.blob(a.tobytes()),
            "componentType": 5125 if indices else 5126,
            "count": len(a),
            "type": {1: "SCALAR", 2: "VEC2", 3: "VEC3", 4: "VEC4"}[n],
        }
        if n == 3:
            v.update(min=a.min(0).tolist(), max=a.max(0).tolist())
        self.d["accessors"].append(v)
        return i

    def node(self, name, parent, translation=None):
        i = len(self.d["nodes"])
        n = {"name": name, "children": []}
        if translation is not None:
            n["translation"] = np.asarray(translation).tolist()
        self.d["nodes"].append(n)
        self.d["nodes"][parent]["children"].append(i)
        return i

    def mesh(self, p, tri, parent, name, offset):
        ids, inv = np.unique(p["indices"][tri], return_inverse=True)
        attrs = {k: v[ids].copy() for k, v in p["attrs"].items()}
        attrs["POSITION"] -= offset
        primitive = {
            "attributes": {k: self.accessor(v) for k, v in attrs.items()},
            "indices": self.accessor(inv.astype(np.uint32).ravel(), True),
            "material": p["material"],
        }
        mi = len(self.d["meshes"])
        self.d["meshes"].append({"name": name, "primitives": [primitive]})
        ni = self.node(name, parent)
        self.d["nodes"][ni]["mesh"] = mi

    def save(self, path):
        self.d["buffers"][0]["byteLength"] = len(self.data)
        j = json.dumps(self.d, separators=(",", ":")).encode()
        j += b" " * (-len(j) % 4)
        b = bytes(self.data)
        b += b"\0" * (-len(b) % 4)
        Path(path).write_bytes(
            struct.pack("<III", 0x46546C67, 2, 28 + len(j) + len(b))
            + struct.pack("<II", len(j), 0x4E4F534A)
            + j
            + struct.pack("<II", len(b), 0x004E4942)
            + b
        )


def process(path, output, forward="+Z", wheelbase=None, texture_size=1024):
    source = Source(path)
    prims = source.primitives()
    materials = source.d["materials"]
    groups = identify_wheels(prims, materials)
    axes = {"+X": (1, 0, 0), "-X": (-1, 0, 0), "+Z": (0, 0, 1), "-Z": (0, 0, -1)}
    f = np.array(axes[forward])
    up = np.array([0, 1, 0])
    right = np.cross(f, up)
    rotation = np.array([f, up, right])
    centers = np.array([g["center"] for g in groups]) @ rotation.T
    order = sorted(range(4), key=lambda i: (-centers[i, 0], centers[i, 2]))
    front = sorted(order[:2], key=lambda i: centers[i, 2])
    rear = sorted(order[2:], key=lambda i: centers[i, 2])
    order = front + rear
    raw_base = centers[front, 0].mean() - centers[rear, 0].mean()
    scale = wheelbase / raw_base if wheelbase else 1.0
    centers *= scale
    for pair in (front, rear):
        if abs(centers[pair[0], 0] - centers[pair[1], 0]) > 0.1 * raw_base * scale:
            raise ValueError("Wheel axle pair is ambiguous")
    if (
        raw_base <= 0
        or min(
            abs(centers[front[0], 2] - centers[front[1], 2]),
            abs(centers[rear[0], 2] - centers[rear[1], 2]),
        )
        <= 0.3
    ):
        raise ValueError("Invalid wheelbase/track")
    radial = []
    ground = []
    for i in order:
        v = groups[i]["points"] @ rotation.T * scale
        radial.append(float(np.ptp(v, axis=0)[1] / 2))
        ground.append(float(v[:, 1].min()))
    origin = np.array([centers[:, 0].mean(), np.mean(ground), centers[:, 2].mean()])
    centers -= origin
    for p in prims:
        p["attrs"]["POSITION"] = p["attrs"]["POSITION"] @ rotation.T * scale - origin
        for k in ("NORMAL", "TANGENT"):
            if k in p["attrs"]:
                p["attrs"][k][:, :3] = p["attrs"][k][:, :3] @ rotation.T
    writer = Writer(source)
    body = writer.node("BODY", 0)
    wheel_nodes = {}
    manifest_wheels = {}
    for n, i, r in zip(WHEELS, order, radial):
        c = centers[i].copy()
        correction = r - c[1]
        c[1] = r
        steer = writer.node(n + "_STEER", 0, c)
        spin = writer.node(n + "_SPIN", steer)
        wheel_nodes[n] = (steer, spin, centers[i])
        manifest_wheels[n] = {
            "steerPivot": n + "_STEER",
            "spinPivot": n + "_SPIN",
            "center": c.tolist(),
            "radius": r,
            "tireNodes": [],
            "rimNodes": [],
            "discNodes": [],
            "caliperNodes": [],
            "contactPlaneCorrectionM": float(correction),
        }
    removed = []
    source_tris = sum(len(p["indices"]) for p in prims)
    out_tris = 0
    for pi, p in enumerate(prims):
        material = materials[p["material"]]["name"]
        lower = material.lower()
        if "blurred" in lower or "damage_glass" in lower:
            removed.append(material)
            continue  # Alternative blur/damage geometry, not simultaneous visible layers.
        kind = (
            "tire"
            if ("tyre" in lower or "tire" in lower)
            else (
                "rim"
                if "rim" in lower
                else (
                    "caliper"
                    if ("caliper" in lower or lower == "brake")
                    else "disc" if "brake_disk" in lower else None
                )
            )
        )
        if kind:
            for ci, tri in enumerate(components(p["attrs"]["POSITION"], p["indices"])):
                points = p["attrs"]["POSITION"][np.unique(p["indices"][tri])]
                center = (points.min(0) + points.max(0)) / 2
                distances = [np.linalg.norm(center - centers[i]) for i in order]
                k = int(np.argmin(distances))
                n = WHEELS[k]
                if distances[k] > radial[k] * 1.3:
                    raise ValueError(
                        f"Ambiguous {material} component at {center}: distances={distances}"
                    )
                steer, spin, offset = wheel_nodes[n]
                name = f"{n}_{kind}_{pi}_{ci}"
                writer.mesh(p, tri, steer if kind == "caliper" else spin, name, offset)
                manifest_wheels[n][kind + "Nodes"].append(name)
                out_tris += len(tri)
        else:
            writer.mesh(
                p,
                np.arange(len(p["indices"])),
                body,
                f"body_{pi}_{material}",
                np.zeros(3),
            )
            out_tris += len(p["indices"])
    texture_bytes = 0
    for image in source.d.get("images", []):
        file = dependency(source.path.parent, image["uri"])
        with Image.open(file) as im:
            im.thumbnail((texture_size, texture_size), Image.Resampling.LANCZOS)
            im = im.convert("RGBA" if "A" in im.getbands() else "RGB")
            texture_bytes += im.width * im.height * 4 * 4 // 3
            buffer = io.BytesIO()
            im.save(buffer, format="PNG", optimize=True)
        writer.d["images"].append(
            {
                "name": file.stem,
                "bufferView": writer.blob(buffer.getvalue()),
                "mimeType": "image/png",
            }
        )
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    writer.save(output)
    manifest = {
        "schemaVersion": 1,
        "assetName": source.d["asset"]
        .get("extras", {})
        .get("title", source.path.parent.name),
        "sourceFile": str(source.path),
        "source": source.d["asset"].get("extras", {}),
        "outputGLB": output.name,
        "localForwardAxis": "+X",
        "localLeftAxis": "-Z",
        "localUpAxis": "+Y",
        "sourceSceneForwardAxis": forward,
        "sourceSceneUpAxis": "+Y",
        "scaleMetersPerAssetUnit": float(scale),
        "scaleBasis": (
            "explicit wheelbase normalization; dimension anchor is documented by caller"
            if wheelbase
            else "source scene units assumed metres"
        ),
        "bodyNode": "BODY",
        "wheels": manifest_wheels,
        "wheelbase": float(raw_base * scale),
        "frontTrack": float(abs(centers[front[0], 2] - centers[front[1], 2])),
        "rearTrack": float(abs(centers[rear[0], 2] - centers[rear[1], 2])),
        "groundOffset": float(-origin[1]),
        "sourceOriginCorrection": (-origin).tolist(),
        "cameraTarget": [0, 0.65, 0],
        "visualCG": [0, 0.45, 0],
        "originConvention": "axle midpoint on tire-derived contact plane; runtime aligns midpoint with physical axle midpoint",
        "sourceTriangleCount": source_tris,
        "processedTriangleCount": out_tris,
        "sourcePackageBytes": sum(
            p.stat().st_size for p in source.path.parent.rglob("*") if p.is_file()
        ),
        "processedBytes": output.stat().st_size,
        "textureMemoryEstimateBytes": texture_bytes,
        "primitiveCount": len(writer.d["meshes"]),
        "removedAlternativeMaterials": removed,
        "sourceSHA256": hashlib.sha256(
            source.path.read_bytes() + b"".join(source.buffers)
        ).hexdigest(),
        "classification": "welded triangle connectivity, spatial merging of concentric tire shells, bbox centers and explicit source forward sign",
        "limitations": [
            "Kinematic wheel spin only; no physical angular momentum or tire wheelspin.",
            "F1 brake material assigned stationary: supplied rear-only low-detail blocks; no fabricated front brake components.",
        ],
    }
    output.with_suffix(".json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--forward", choices=["+X", "-X", "+Z", "-Z"], default="+Z")
    parser.add_argument("--wheelbase", type=float)
    parser.add_argument("--texture-size", type=int, default=1024)
    a = parser.parse_args()
    m = process(a.source, a.output, a.forward, a.wheelbase, a.texture_size)
    print(json.dumps({k: v for k, v in m.items() if k not in ["wheels"]}, indent=2))
