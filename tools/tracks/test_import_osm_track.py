import json
import tempfile
import unittest
from pathlib import Path

import import_osm_track as importer


def way(osm_id, first, second, a, b, **tags):
    return {
        "type": "way",
        "id": osm_id,
        "nodes": [first, second],
        "geometry": [{"lat": a[0], "lon": a[1]}, {"lat": b[0], "lon": b[1]}],
        "tags": tags,
    }


class ImportOsmTrackTest(unittest.TestCase):
    def test_length_selected_course_and_render_context_are_separate(self):
        # Approximately 100 m square and a separate approximately 50 m square.
        latitude = 50.0
        dlat = 100.0 / 111_200.0
        dlon = 100.0 / (111_200.0 * 0.6428)
        large = [(latitude, 5.0), (latitude, 5.0 + dlon),
                 (latitude + dlat, 5.0 + dlon), (latitude + dlat, 5.0)]
        small = [(latitude + 0.01, 5.0), (latitude + 0.01, 5.0 + dlon / 2),
                 (latitude + 0.01 + dlat / 2, 5.0 + dlon / 2),
                 (latitude + 0.01 + dlat / 2, 5.0)]
        elements = []
        for offset, points in ((0, large), (10, small)):
            node_ids = [offset + index + 1 for index in range(4)]
            for index in range(4):
                elements.append(
                    way(
                        100 + offset + index,
                        node_ids[index],
                        node_ids[(index + 1) % 4],
                        points[index],
                        points[(index + 1) % 4],
                        highway="raceway",
                        sport="motor",
                        oneway="yes",
                    )
                )
        elements.append(
            way(999, 30, 31, large[0], large[1], highway="service", service="pit_lane")
        )

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.json"
            output = root / "track.json"
            render = root / "render.json"
            source.write_text(json.dumps({"elements": elements}), encoding="utf-8")
            result = importer.main(
                [
                    "--bbox", "49.9,4.9,50.2,5.2",
                    "--name", "Fixture Circuit",
                    "--reference-length-m", "400",
                    "--reference-url", "https://example.test/reference",
                    "--half-width-m", "6",
                    "--osm-file", str(source),
                    "--output", str(output),
                    "--render-output", str(render),
                    "--elevation", "none",
                    "--spacing-m", "20",
                ]
            )
            self.assertEqual(result, 0)
            track = json.loads(output.read_text(encoding="utf-8"))
            context = json.loads(render.read_text(encoding="utf-8"))
            self.assertEqual(track["schema_version"], 2)
            self.assertEqual(track["kind"], "real_imported")
            self.assertEqual(set(track["course_selection"]["selected_osm_way_ids"]), {100, 101, 102, 103})
            self.assertLess(track["length_validation"]["percentage_difference"], 2.0)
            self.assertTrue(all(point["elevation_m"] == 0.0 for point in track["control_points"]))
            self.assertEqual(context["purpose"], "render_context_only")
            self.assertIn(999, {feature["osm_way_id"] for feature in context["features"]})

    def test_explicit_open_way_selection_is_rejected(self):
        source_way = way(
            1, 1, 2, (50.0, 5.0), (50.001, 5.0), highway="raceway", sport="motor"
        )
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "source.json"
            source.write_text(json.dumps({"elements": [source_way]}), encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "does not form exactly one closed course"):
                importer.main(
                    [
                        "--bbox", "49.9,4.9,50.2,5.2",
                        "--name", "Broken",
                        "--way-id", "1",
                        "--half-width-m", "6",
                        "--osm-file", str(source),
                        "--output", str(Path(directory) / "track.json"),
                    ]
                )


if __name__ == "__main__":
    unittest.main()
