#!/usr/bin/env python3
"""Import an explicitly selected OSM race course into ApexLab track schema v2.

The importer never concatenates every `highway=raceway` feature. It finds closed
topological cycles, compares them with a documented reference length, and records
the exact selected OSM way IDs. An explicit list of `--way-id` values can be used
instead. Render-only facility geometry is emitted to a separate sidecar.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import statistics
import sys
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import date
from pathlib import Path
from typing import Iterable, Sequence


USER_AGENT = "ApexLab-track-import/0.1 (offline engineering asset pipeline)"
OVERPASS_ENDPOINTS = (
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
)
OSM_COPYRIGHT_URL = "https://www.openstreetmap.org/copyright"
OPEN_METEO_ELEVATION_URL = "https://api.open-meteo.com/v1/elevation"


@dataclass(frozen=True)
class CircuitSpec:
    slug: str
    name: str
    bbox: tuple[float, float, float, float]  # south, west, north, east
    reference_length_m: float
    reference_url: str
    estimated_half_width_m: float
    default_elevation: str = "none"


CIRCUITS = {
    "spa-francorchamps": CircuitSpec(
        "spa-francorchamps",
        "Spa-Francorchamps",
        (50.42, 5.94, 50.46, 6.00),
        7004.0,
        "https://www.spa-francorchamps.be/en/the-circuit",
        6.0,
        "open-meteo",
    ),
    "monza": CircuitSpec(
        "monza",
        "Monza",
        (45.61, 9.275, 45.635, 9.30),
        5793.0,
        "https://www.formula1.com/en/information/italy-autodromo-nazionalemonza.FiJN1jnQlRLeHqOxIt13m",
        6.0,
    ),
    "silverstone": CircuitSpec(
        "silverstone",
        "Silverstone",
        (52.05, -1.03, 52.09, -0.99),
        5891.0,
        "https://ticketing.formula1.com/great-britain/",
        7.0,
    ),
    "suzuka": CircuitSpec(
        "suzuka",
        "Suzuka",
        (34.83, 136.52, 34.86, 136.56),
        5807.0,
        "https://www.suzukacircuit.jp/eng/course_s/",
        6.0,
    ),
}


@dataclass(frozen=True)
class GeoPoint:
    lat: float
    lon: float


@dataclass(frozen=True)
class Way:
    osm_id: int
    node_ids: tuple[int, ...]
    points: tuple[GeoPoint, ...]
    tags: dict[str, str]

    @property
    def endpoints(self) -> tuple[int, int]:
        return self.node_ids[0], self.node_ids[-1]


def _parse_overpass_json(path: Path) -> list[Way]:
    document = json.loads(path.read_text(encoding="utf-8"))
    ways: list[Way] = []
    for element in document.get("elements", []):
        if element.get("type") != "way" or not element.get("geometry"):
            continue
        nodes = tuple(int(value) for value in element.get("nodes", []))
        geometry = tuple(GeoPoint(float(p["lat"]), float(p["lon"])) for p in element["geometry"])
        if len(nodes) != len(geometry):
            # `out geom` should supply both. Synthetic stable IDs retain topology if it does not.
            nodes = tuple(-(int(element["id"]) * 1_000_000 + index) for index in range(len(geometry)))
        ways.append(Way(int(element["id"]), nodes, geometry, dict(element.get("tags", {}))))
    return ways


def _parse_osm_xml(path: Path) -> list[Way]:
    root = ET.parse(path).getroot()
    nodes = {
        int(node.attrib["id"]): GeoPoint(float(node.attrib["lat"]), float(node.attrib["lon"]))
        for node in root.findall("node")
    }
    ways: list[Way] = []
    for element in root.findall("way"):
        node_ids = tuple(int(node.attrib["ref"]) for node in element.findall("nd"))
        if len(node_ids) < 2 or any(node_id not in nodes for node_id in node_ids):
            continue
        tags = {tag.attrib["k"]: tag.attrib["v"] for tag in element.findall("tag")}
        ways.append(
            Way(int(element.attrib["id"]), node_ids, tuple(nodes[node_id] for node_id in node_ids), tags)
        )
    return ways


def load_osm(path: Path) -> list[Way]:
    return _parse_overpass_json(path) if path.suffix.lower() == ".json" else _parse_osm_xml(path)


def fetch_overpass(bbox: tuple[float, float, float, float], output: Path | None = None) -> list[Way]:
    south, west, north, east = bbox
    box = f"{south},{west},{north},{east}"
    query = (
        "[out:json][timeout:120];("
        f'way["highway"="raceway"]({box});'
        f'way["service"="pit_lane"]({box});'
        f'way["highway"="service"]({box});'
        ");out body geom;"
    )
    payload = urllib.parse.urlencode({"data": query}).encode("utf-8")
    errors: list[str] = []
    for endpoint in OVERPASS_ENDPOINTS:
        request = urllib.request.Request(
            endpoint,
            data=payload,
            headers={"User-Agent": USER_AGENT, "Content-Type": "application/x-www-form-urlencoded"},
        )
        try:
            with urllib.request.urlopen(request, timeout=150) as response:
                raw = response.read()
            if output:
                output.parent.mkdir(parents=True, exist_ok=True)
                output.write_bytes(raw)
                return _parse_overpass_json(output)
            temporary = Path("/tmp/apexlab-overpass-response.json")
            temporary.write_bytes(raw)
            return _parse_overpass_json(temporary)
        except (OSError, urllib.error.URLError) as error:
            errors.append(f"{endpoint}: {error}")
    raise RuntimeError("all Overpass endpoints failed:\n  " + "\n  ".join(errors))


def _is_course_candidate(way: Way) -> bool:
    if way.tags.get("highway") != "raceway":
        return False
    if way.tags.get("sport") not in (None, "motor"):
        return False
    name = way.tags.get("name", "")
    if way.tags.get("raceway") == "pit_lane" or way.tags.get("service") == "pit_lane":
        return False
    return re.search(r"(?:pit\s*lane|circuit\s*pit|kart|moto|service)", name, flags=re.IGNORECASE) is None


def _find_cycles(ways: Sequence[Way], maximum_cycles: int = 20_000) -> list[list[tuple[int, bool]]]:
    by_id = {way.osm_id: way for way in ways}
    adjacency: dict[int, list[int]] = {}
    for way in ways:
        first, last = way.endpoints
        adjacency.setdefault(first, []).append(way.osm_id)
        adjacency.setdefault(last, []).append(way.osm_id)

    found: dict[frozenset[int], list[tuple[int, bool]]] = {}

    def visit(
        start: int,
        current: int,
        used_edges: set[int],
        visited_nodes: set[int],
        path: list[tuple[int, bool]],
    ) -> None:
        if len(found) >= maximum_cycles:
            raise RuntimeError("course topology produced too many cycles; use explicit --way-id selection")
        for way_id in adjacency.get(current, []):
            if way_id in used_edges:
                continue
            way = by_id[way_id]
            first, last = way.endpoints
            forward = current == first
            following = last if forward else first
            next_path = path + [(way_id, forward)]
            if following == start:
                key = frozenset(item[0] for item in next_path)
                found.setdefault(key, next_path)
            elif following not in visited_nodes:
                visit(
                    start,
                    following,
                    used_edges | {way_id},
                    visited_nodes | {following},
                    next_path,
                )

    for node_id in sorted(adjacency):
        visit(node_id, node_id, set(), {node_id}, [])
    return list(found.values())


def _oneway_violation(way: Way, forward: bool) -> int:
    value = way.tags.get("oneway", "").lower()
    if value in ("yes", "1", "true"):
        return int(not forward)
    if value == "-1":
        return int(forward)
    return 0


def _prefer_osm_direction(
    cycle: list[tuple[int, bool]], by_id: dict[int, Way]
) -> list[tuple[int, bool]]:
    reverse = [(way_id, not forward) for way_id, forward in reversed(cycle)]
    forward_score = sum(_oneway_violation(by_id[way_id], direction) for way_id, direction in cycle)
    reverse_score = sum(_oneway_violation(by_id[way_id], direction) for way_id, direction in reverse)
    return reverse if reverse_score < forward_score else cycle


def _cycle_points(
    cycle: Sequence[tuple[int, bool]], by_id: dict[int, Way]
) -> tuple[list[tuple[int, GeoPoint]], list[int]]:
    result: list[tuple[int, GeoPoint]] = []
    way_ids: list[int] = []
    for way_id, forward in cycle:
        way = by_id[way_id]
        pairs = list(zip(way.node_ids, way.points))
        if not forward:
            pairs.reverse()
        if result and result[-1][0] != pairs[0][0]:
            raise RuntimeError(
                f"topological gap between OSM ways {way_ids[-1]} and {way_id}; "
                f"node {result[-1][0]} != {pairs[0][0]}"
            )
        result.extend(pairs if not result else pairs[1:])
        way_ids.append(way_id)
    if not result or result[0][0] != result[-1][0]:
        raise RuntimeError("selected OSM course is not closed; refusing to bridge the gap")
    return result, way_ids


def _ecef(point: GeoPoint, elevation_m: float = 0.0) -> tuple[float, float, float]:
    # WGS84 geodetic to Earth-centred Earth-fixed.
    a = 6_378_137.0
    eccentricity_sq = 6.69437999014e-3
    latitude = math.radians(point.lat)
    longitude = math.radians(point.lon)
    sin_lat = math.sin(latitude)
    normal = a / math.sqrt(1.0 - eccentricity_sq * sin_lat * sin_lat)
    return (
        (normal + elevation_m) * math.cos(latitude) * math.cos(longitude),
        (normal + elevation_m) * math.cos(latitude) * math.sin(longitude),
        (normal * (1.0 - eccentricity_sq) + elevation_m) * sin_lat,
    )


def project_enu(point: GeoPoint, origin: GeoPoint) -> tuple[float, float]:
    x, y, z = _ecef(point)
    x0, y0, z0 = _ecef(origin)
    dx, dy, dz = x - x0, y - y0, z - z0
    latitude = math.radians(origin.lat)
    longitude = math.radians(origin.lon)
    east = -math.sin(longitude) * dx + math.cos(longitude) * dy
    north = (
        -math.sin(latitude) * math.cos(longitude) * dx
        - math.sin(latitude) * math.sin(longitude) * dy
        + math.cos(latitude) * dz
    )
    return east, north


def _polyline_length(points: Sequence[tuple[float, float]]) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def _select_course(
    ways: Sequence[Way],
    reference_length_m: float | None,
    explicit_way_ids: set[int],
) -> tuple[list[tuple[int, GeoPoint]], list[int], list[dict[str, object]]]:
    by_id = {way.osm_id: way for way in ways}
    candidates = [way for way in ways if _is_course_candidate(way)]
    if explicit_way_ids:
        missing = explicit_way_ids - by_id.keys()
        if missing:
            raise RuntimeError(f"explicit OSM ways are absent from source: {sorted(missing)}")
        cycles = _find_cycles([by_id[way_id] for way_id in explicit_way_ids])
        cycles = [cycle for cycle in cycles if {item[0] for item in cycle} == explicit_way_ids]
        if not cycles:
            raise RuntimeError("explicit OSM way selection does not form exactly one closed course")
    else:
        if reference_length_m is None:
            raise RuntimeError("course selection requires a published reference length or explicit --way-id values")
        cycles = _find_cycles(candidates)
        if not cycles:
            raise RuntimeError("no closed raceway cycle found; inspect OSM topology or use explicit --way-id values")

    diagnostics: list[dict[str, object]] = []
    evaluated: list[tuple[float, list[tuple[int, bool]], list[tuple[int, GeoPoint]], list[int]]] = []
    for cycle in cycles:
        oriented = _prefer_osm_direction(cycle, by_id)
        route, way_ids = _cycle_points(oriented, by_id)
        origin = GeoPoint(
            statistics.fmean(point.lat for _, point in route[:-1]),
            statistics.fmean(point.lon for _, point in route[:-1]),
        )
        planar = [project_enu(point, origin) for _, point in route]
        length = _polyline_length(planar)
        error = abs(length - reference_length_m) if reference_length_m is not None else -length
        diagnostics.append(
            {
                "way_ids": way_ids,
                "polyline_length_m": round(length, 3),
                "reference_error_m": round(error, 3) if reference_length_m is not None else None,
            }
        )
        evaluated.append((error, oriented, route, way_ids))
    evaluated.sort(key=lambda value: (value[0], len(value[3])))
    _, _, route, way_ids = evaluated[0]
    return route, way_ids, diagnostics


def _deduplicate(
    points: Sequence[tuple[float, float, float, float]], tolerance_m: float = 0.02
) -> list[tuple[float, float, float, float]]:
    result = [points[0]]
    for point in points[1:]:
        if math.dist(point[:2], result[-1][:2]) > tolerance_m:
            result.append(point)
    if math.dist(result[0][:2], result[-1][:2]) > tolerance_m:
        raise RuntimeError(
            f"course closure gap is {math.dist(result[0][:2], result[-1][:2]):.3f} m; refusing to bridge it"
        )
    result[-1] = result[0]
    return result


def _resample_closed(
    points: Sequence[tuple[float, float, float, float]], spacing_m: float
) -> list[tuple[float, float, float, float]]:
    distances = [0.0]
    for first, second in zip(points, points[1:]):
        distances.append(distances[-1] + math.dist(first[:2], second[:2]))
    length = distances[-1]
    count = max(4, round(length / spacing_m))
    result: list[tuple[float, float, float, float]] = []
    segment = 0
    for index in range(count):
        target = length * index / count
        while segment + 1 < len(distances) and distances[segment + 1] < target:
            segment += 1
        span = distances[segment + 1] - distances[segment]
        fraction = 0.0 if span == 0.0 else (target - distances[segment]) / span
        a, b = points[segment], points[segment + 1]
        result.append(tuple(a[i] + fraction * (b[i] - a[i]) for i in range(4)))
    return result


def _sample_open_meteo(
    points: Sequence[tuple[float, float, float, float]]
) -> tuple[list[float], int]:
    # The source raster is 90 m. Query at most 100 nearly uniform points, then
    # interpolate to the denser geometry grid. This avoids pretending that API
    # oversampling adds terrain detail and stays within one documented request.
    source_count = min(100, len(points))
    indices = [math.floor(index * len(points) / source_count) for index in range(source_count)]
    batch = [points[index] for index in indices]
    query = urllib.parse.urlencode(
        {
            "latitude": ",".join(f"{point[2]:.7f}" for point in batch),
            "longitude": ",".join(f"{point[3]:.7f}" for point in batch),
        },
        safe=",",
    )
    request = urllib.request.Request(
        f"{OPEN_METEO_ELEVATION_URL}?{query}", headers={"User-Agent": USER_AGENT}
    )
    with urllib.request.urlopen(request, timeout=60) as response:
        values = [float(value) for value in json.loads(response.read())["elevation"]]
    if len(values) != len(batch):
        raise RuntimeError("elevation service returned incomplete data")

    result: list[float] = []
    for point_index in range(len(points)):
        scaled = point_index * source_count / len(points)
        lower = math.floor(scaled)
        fraction = scaled - lower
        first = values[lower % source_count]
        second = values[(lower + 1) % source_count]
        result.append(first + fraction * (second - first))
    return result, source_count


def _smooth_elevation(values: Sequence[float], window: int = 7) -> tuple[list[float], int]:
    radius = max(1, window // 2)
    cleaned: list[float] = []
    outliers = 0
    for index, value in enumerate(values):
        neighborhood = [values[(index + offset) % len(values)] for offset in range(-radius, radius + 1)]
        median = statistics.median(neighborhood)
        if abs(value - median) > 20.0:
            value = median
            outliers += 1
        cleaned.append(value)
    smoothed = [
        statistics.fmean(cleaned[(index + offset) % len(cleaned)] for offset in range(-radius, radius + 1))
        for index in range(len(cleaned))
    ]
    return smoothed, outliers


def _sample_copernicus_geotiff(
    path: Path, points: Sequence[tuple[float, float, float, float]]
) -> list[float]:
    try:
        import rasterio  # type: ignore[import-not-found]
    except ImportError as error:
        raise RuntimeError(
            "Copernicus GeoTIFF sampling requires rasterio (`python3 -m pip install rasterio`)"
        ) from error
    with rasterio.open(path) as dataset:
        if dataset.crs is None or dataset.crs.to_epsg() != 4326:
            raise RuntimeError(f"expected an EPSG:4326 Copernicus tile, got {dataset.crs}")
        values = [float(sample[0]) for sample in dataset.sample((point[3], point[2]) for point in points)]
        nodata = dataset.nodata
    if any(not math.isfinite(value) or (nodata is not None and value == nodata) for value in values):
        raise RuntimeError("Copernicus GeoTIFF returned missing/non-finite elevation samples")
    return values


def _catmull(values: Sequence[float], index: int, t: float) -> float:
    count = len(values)
    p0, p1, p2, p3 = (
        values[(index - 1) % count],
        values[index % count],
        values[(index + 1) % count],
        values[(index + 2) % count],
    )
    return 0.5 * (
        2.0 * p1
        + (p2 - p0) * t
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t * t
        + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t * t * t
    )


def _periodic_spline_length(control_points: Sequence[dict[str, float]], samples_per_segment: int = 64) -> float:
    xs = [point["x_m"] for point in control_points]
    ys = [point["y_m"] for point in control_points]
    zs = [point["elevation_m"] for point in control_points]
    previous = (_catmull(xs, 0, 0.0), _catmull(ys, 0, 0.0), _catmull(zs, 0, 0.0))
    length = 0.0
    for sample in range(1, len(control_points) * samples_per_segment + 1):
        parameter = sample / samples_per_segment
        segment = math.floor(parameter) % len(control_points)
        t = parameter - math.floor(parameter)
        current = (_catmull(xs, segment, t), _catmull(ys, segment, t), _catmull(zs, segment, t))
        length += math.dist(previous, current)
        previous = current
    return length


def _render_context(ways: Sequence[Way], origin: GeoPoint) -> dict[str, object]:
    features: list[dict[str, object]] = []
    for way in ways:
        if way.tags.get("highway") not in ("raceway", "service") and way.tags.get("service") != "pit_lane":
            continue
        geometry = [project_enu(point, origin) for point in way.points]
        features.append(
            {
                "osm_way_id": way.osm_id,
                "classification": "raceway" if way.tags.get("highway") == "raceway" else "service_road",
                "tags": {
                    key: way.tags[key]
                    for key in ("name", "highway", "service", "surface", "sport", "operator")
                    if key in way.tags
                },
                "points_m": [[round(x, 3), round(y, 3)] for x, y in geometry],
            }
        )
    return {
        "schema_version": 1,
        "purpose": "render_context_only",
        "warning": "This file is not mathematical race-course geometry.",
        "source": "OpenStreetMap contributors, ODbL 1.0",
        "features": features,
    }


def _parse_bbox(value: str) -> tuple[float, float, float, float]:
    values = tuple(float(part.strip()) for part in value.split(","))
    if len(values) != 4:
        raise argparse.ArgumentTypeError("bbox must be SOUTH,WEST,NORTH,EAST")
    south, west, north, east = values
    if south >= north or west >= east:
        raise argparse.ArgumentTypeError("bbox south/west must be below north/east")
    return values


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--circuit", choices=sorted(CIRCUITS))
    parser.add_argument("--bbox", type=_parse_bbox, help="SOUTH,WEST,NORTH,EAST for an explicit area")
    parser.add_argument("--name", help="required with --bbox")
    parser.add_argument("--reference-length-m", type=float)
    parser.add_argument("--reference-url")
    parser.add_argument("--way-id", action="append", type=int, default=[])
    parser.add_argument("--osm-file", type=Path, help="cached Overpass JSON or OSM XML")
    parser.add_argument("--raw-output", type=Path, help="cache the Overpass JSON response")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--render-output", type=Path)
    parser.add_argument("--diagnostics-output", type=Path)
    parser.add_argument("--spacing-m", type=float, default=10.0)
    parser.add_argument("--half-width-m", type=float)
    parser.add_argument("--elevation", choices=("none", "open-meteo", "copernicus-geotiff"))
    parser.add_argument("--elevation-geotiff", type=Path, help="local Copernicus GLO-30 EPSG:4326 tile")
    parser.add_argument("--maximum-length-error-percent", type=float, default=10.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    spec = CIRCUITS.get(args.circuit) if args.circuit else None
    if not spec and (not args.bbox or not args.name):
        raise SystemExit("select a named --circuit or provide both --bbox and --name")
    bbox = spec.bbox if spec else args.bbox
    name = spec.name if spec else args.name
    reference_length = args.reference_length_m or (spec.reference_length_m if spec else None)
    reference_url = args.reference_url or (spec.reference_url if spec else None)
    if not reference_url and not args.way_id:
        raise SystemExit("a documented --reference-url is required for length-based course selection")
    half_width = args.half_width_m or (spec.estimated_half_width_m if spec else None)
    if half_width is None or half_width <= 0.0:
        raise SystemExit("provide a positive --half-width-m; OSM centerlines do not define reliable widths")
    if args.spacing_m <= 0.0:
        raise SystemExit("--spacing-m must be positive")
    elevation_mode = args.elevation or (spec.default_elevation if spec else "none")
    if args.elevation_geotiff:
        elevation_mode = "copernicus-geotiff"

    ways = load_osm(args.osm_file) if args.osm_file else fetch_overpass(bbox, args.raw_output)
    route, selected_way_ids, cycle_diagnostics = _select_course(
        ways, reference_length, set(args.way_id)
    )
    raw_origin = GeoPoint(
        statistics.fmean(point.lat for _, point in route[:-1]),
        statistics.fmean(point.lon for _, point in route[:-1]),
    )
    projected = [
        (*project_enu(point, raw_origin), point.lat, point.lon) for _, point in route
    ]
    cleaned = _deduplicate(projected)
    resampled = _resample_closed(cleaned, args.spacing_m)

    if elevation_mode == "open-meteo":
        absolute_elevation, source_sample_count = _sample_open_meteo(resampled)
        absolute_elevation, outlier_count = _smooth_elevation(absolute_elevation)
        origin_elevation = statistics.fmean(absolute_elevation)
        local_elevation = [value - origin_elevation for value in absolute_elevation]
        elevation_metadata = {
            "source": "Copernicus DEM 2021 GLO-90 via Open-Meteo Elevation API",
            "source_url": "https://open-meteo.com/en/docs/elevation-api",
            "source_resolution_m": 90,
            "source_sample_count": source_sample_count,
            "interpolated_control_point_count": len(absolute_elevation),
            "smoothing": "circular seven-sample moving mean after 20 m local-median outlier rejection",
            "outliers_replaced": outlier_count,
            "absolute_min_m": round(min(absolute_elevation), 3),
            "absolute_max_m": round(max(absolute_elevation), 3),
            "limitation": "Terrain-derived DSM elevation; not survey-grade racetrack paving geometry.",
        }
    elif elevation_mode == "copernicus-geotiff":
        if not args.elevation_geotiff or not args.elevation_geotiff.is_file():
            raise RuntimeError("--elevation copernicus-geotiff requires --elevation-geotiff FILE")
        absolute_elevation = _sample_copernicus_geotiff(args.elevation_geotiff, resampled)
        absolute_elevation, outlier_count = _smooth_elevation(absolute_elevation)
        origin_elevation = statistics.fmean(absolute_elevation)
        local_elevation = [value - origin_elevation for value in absolute_elevation]
        elevation_metadata = {
            "source": "Copernicus DEM 2021 GLO-30 Public GeoTIFF",
            "source_url": "https://registry.opendata.aws/copernicus-dem/",
            "source_tile": args.elevation_geotiff.name,
            "source_resolution_m": 30,
            "source_sample_count": len(absolute_elevation),
            "smoothing": "circular seven-sample moving mean after 20 m local-median outlier rejection",
            "outliers_replaced": outlier_count,
            "absolute_min_m": round(min(absolute_elevation), 3),
            "absolute_max_m": round(max(absolute_elevation), 3),
            "limitation": "Terrain-derived DSM elevation; not survey-grade racetrack paving geometry.",
        }
    else:
        origin_elevation = 0.0
        local_elevation = [0.0] * len(resampled)
        elevation_metadata = {
            "source": None,
            "limitation": "No elevation source requested; this import is flat.",
        }

    control_points = [
        {
            "x_m": round(point[0], 6),
            "y_m": round(point[1], 6),
            "elevation_m": round(elevation, 6),
            "left_width_m": half_width,
            "right_width_m": half_width,
        }
        for point, elevation in zip(resampled, local_elevation)
    ]
    imported_length = _periodic_spline_length(control_points)
    absolute_difference = (
        abs(imported_length - reference_length) if reference_length is not None else None
    )
    percentage_difference = (
        100.0 * absolute_difference / reference_length
        if absolute_difference is not None and reference_length
        else None
    )
    if percentage_difference is not None and percentage_difference > args.maximum_length_error_percent:
        raise RuntimeError(
            f"imported length differs from reference by {percentage_difference:.3f}% "
            f"(limit {args.maximum_length_error_percent:.3f}%); no output written"
        )

    track = {
        "schema_version": 2,
        "name": name,
        "kind": "real_imported",
        "provenance": "OpenStreetMap contributors (ODbL 1.0); imported without forced length scaling",
        "closed": True,
        "interpolation": "periodic_uniform_catmull_rom",
        "coordinate_system": {
            "type": "local_enu_wgs84",
            "origin_lat_deg": round(raw_origin.lat, 9),
            "origin_lon_deg": round(raw_origin.lon, 9),
            "origin_elevation_m": round(origin_elevation, 3),
            "x_axis": "east",
            "y_axis": "north",
            "z_axis": "up",
            "units": "metres",
        },
        "control_points": control_points,
        "sector_boundaries_fraction": [1.0 / 3.0, 2.0 / 3.0, 1.0],
        "source": {
            "geometry": "OpenStreetMap / Overpass API",
            "license": "ODbL 1.0",
            "attribution_url": OSM_COPYRIGHT_URL,
            "accessed_date": date.today().isoformat(),
            "bbox_south_west_north_east": list(bbox),
        },
        "course_selection": {
            "method": "explicit_way_ids" if args.way_id else "closed_cycle_closest_to_published_length",
            "selected_osm_way_ids": selected_way_ids,
            "candidate_cycle_count": len(cycle_diagnostics),
            "excluded_name_pattern": "pit lane|circuit pit|kart|moto|service",
            "width_m_each_side": half_width,
            "width_classification": "estimated generic half-width; not surveyed track width",
            "resample_spacing_m": args.spacing_m,
            "forced_scale": False,
        },
        "length_validation": {
            "imported_length_m": round(imported_length, 3),
            "reference_length_m": reference_length,
            "reference_url": reference_url,
            "absolute_difference_m": round(absolute_difference, 3) if absolute_difference is not None else None,
            "percentage_difference": round(percentage_difference, 5) if percentage_difference is not None else None,
        },
        "elevation": elevation_metadata,
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(track, indent=2) + "\n", encoding="utf-8")
    render_output = args.render_output or args.output.with_suffix(".render.json")
    render_output.write_text(json.dumps(_render_context(ways, raw_origin), indent=2) + "\n", encoding="utf-8")
    diagnostics = {
        "selected_way_ids": selected_way_ids,
        "cycle_candidates": cycle_diagnostics,
        "output": str(args.output),
        "render_output": str(render_output),
        "length_validation": track["length_validation"],
        "elevation": elevation_metadata,
    }
    if args.diagnostics_output:
        args.diagnostics_output.parent.mkdir(parents=True, exist_ok=True)
        args.diagnostics_output.write_text(json.dumps(diagnostics, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(diagnostics, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
