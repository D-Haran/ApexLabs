import json
import struct
import tempfile
import unittest
from pathlib import Path

from inspect_gltf import inspect_model


DOCUMENT = {
    "asset": {"version": "2.0"},
    "accessors": [{"count": 6}],
    "meshes": [{"primitives": [{"indices": 0, "mode": 4}]}],
    "nodes": [{"name": "Body_Paint"}, {"name": "Wheel_FL"}],
    "materials": [{"name": "paint"}],
    "textures": [],
    "images": [],
}


class InspectGltfTest(unittest.TestCase):
    def test_gltf_statistics_and_node_candidates(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scene.gltf"
            path.write_text(json.dumps(DOCUMENT), encoding="utf-8")
            report = inspect_model(path)
        self.assertEqual(report["measured_triangle_count"], 2)
        self.assertEqual(report["primitive_draw_call_estimate"], 1)
        self.assertEqual(report["wheel_node_candidates"], ["Wheel_FL"])
        self.assertEqual(report["body_node_candidates"], ["Body_Paint"])

    def test_glb_json_chunk(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scene.glb"
            payload = json.dumps(DOCUMENT, separators=(",", ":")).encode("utf-8")
            payload += b" " * ((4 - len(payload) % 4) % 4)
            total_length = 12 + 8 + len(payload)
            path.write_bytes(
                b"glTF"
                + struct.pack("<II", 2, total_length)
                + struct.pack("<II", len(payload), 0x4E4F534A)
                + payload
            )
            report = inspect_model(path)
        self.assertEqual(report["measured_triangle_count"], 2)


if __name__ == "__main__":
    unittest.main()
