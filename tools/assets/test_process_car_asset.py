import json, struct, tempfile, unittest
from pathlib import Path
import numpy as np
from process_car_asset import Source, components, identify_wheels

ROOT = Path(__file__).resolve().parents[2]


class Assets(unittest.TestCase):
    def test_missing_dependencies(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "scene.gltf"
            path.write_text(json.dumps({"buffers": [{"uri": "scene.bin"}]}))
            with self.assertRaisesRegex(ValueError, "dependency"):
                Source(path)

    def test_connectivity_welds_seams(self):
        points = np.array(
            [
                [0, 0, 0],
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 0],
                [1, 0, 0],
                [0, -1, 0],
                [10, 0, 0],
                [11, 0, 0],
                [10, 1, 0],
            ]
        )
        groups = components(points, np.array([[0, 1, 2], [3, 4, 5], [6, 7, 8]]))
        self.assertEqual(sorted(map(len, groups)), [1, 2])

    def test_ambiguous_wheels_fail(self):
        with self.assertRaisesRegex(ValueError, "No connected tire"):
            identify_wheels([], [])

    def test_processed_assets(self):
        for name in ("mclaren-p1", "mclaren-f1-2022"):
            path = ROOT / "apps/dashboard/public/assets" / f"{name}.glb"
            manifest = json.loads(path.with_suffix(".json").read_text())
            b = path.read_bytes()
            size = struct.unpack_from("<I", b, 12)[0]
            d = json.loads(b[20 : 20 + size])
            nodes = {n["name"]: n for n in d["nodes"]}
            self.assertEqual(struct.unpack_from("<I", b, 8)[0], len(b))
            self.assertGreater(manifest["wheelbase"], 2)
            self.assertGreater(manifest["frontTrack"], 1)
            for key, w in manifest["wheels"].items():
                self.assertAlmostEqual(w["center"][1] - w["radius"], 0, places=8)
                self.assertTrue(w["tireNodes"])
                self.assertTrue(w["rimNodes"])
                self.assertGreater(
                    w["center"][0] if key[0] == "F" else -w["center"][0], 0
                )
                self.assertGreater(
                    -w["center"][2] if key[1] == "L" else w["center"][2], 0
                )
                steer = nodes[w["steerPivot"]]
                spin = nodes[w["spinPivot"]]
                self.assertIn(
                    w["spinPivot"], [d["nodes"][i]["name"] for i in steer["children"]]
                )
                for n in w["caliperNodes"]:
                    self.assertNotIn(
                        n, [d["nodes"][i]["name"] for i in spin["children"]]
                    )
            for mesh in d["meshes"]:
                for primitive in mesh["primitives"]:
                    self.assertEqual(
                        d["accessors"][primitive["indices"]]["type"], "SCALAR"
                    )
            for im in d["images"]:
                self.assertIn("bufferView", im)
                self.assertNotIn("uri", im)


if __name__ == "__main__":
    unittest.main()
