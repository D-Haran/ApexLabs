# OSM real-circuit import pipeline

`tools/tracks/import_osm_track.py` turns OpenStreetMap raceway ways into mathematical ApexLab track
schema v2. It supports a named circuit or an explicit bounding box plus either published length or
explicit `--way-id` selection.

## Geometry policy

The importer queries raceways, pit lanes, and nearby service roads, but does not concatenate them.
For the physics course it filters obvious pit/kart/motorcycle/service features, builds the OSM-way
endpoint graph, enumerates closed cycles, respects OSM one-way direction where possible, and chooses
the cycle closest to the documented published layout length. The output records every selected OSM
way ID and every competing cycle considered. An explicit way list bypasses length selection.

Ordered nodes pass through duplicate cleanup, closure/gap validation, 10 m resampling, and periodic
Catmull–Rom reconstruction. A non-closing course is an error; the tool never silently bridges a
large gap. The imported curve is never scaled to force agreement with a reference length.

Latitude/longitude is converted to metres with WGS84 geodetic → Earth-centred Earth-fixed → local
east/north/up. Each track stores the exact tangent-plane origin. `x` is east, `y` north, and `z` up.
Arc length is accumulated in 3D by C++; curvature remains horizontal and grade is `dz` divided by
horizontal distance.

Render context is written separately as `*.render.json`. It contains pit/service/raceway polylines
for later environment work and is never read as physics geometry.

## Reproduction

Ordinary imports query Overpass directly:

```sh
python3 tools/tracks/import_osm_track.py \
  --circuit monza \
  --elevation none \
  --output configs/tracks/monza.json
```

Use `--raw-output data/raw/osm/monza.json` to cache an Overpass response, then repeat with
`--osm-file`. The generated mathematical and render files contain OSM attribution and the ODbL
license URL. Raw responses remain untracked.

Spa was generated from OSM geometry and the public Copernicus GLO-30 tile:

```sh
curl -L \
  'https://copernicus-dem-30m.s3.amazonaws.com/Copernicus_DSM_COG_10_N50_00_E005_00_DEM/Copernicus_DSM_COG_10_N50_00_E005_00_DEM.tif' \
  -o /tmp/Copernicus_DSM_COG_10_N50_00_E005_00_DEM.tif
python3 -m pip install rasterio
python3 tools/tracks/import_osm_track.py \
  --circuit spa-francorchamps \
  --elevation-geotiff /tmp/Copernicus_DSM_COG_10_N50_00_E005_00_DEM.tif \
  --output configs/tracks/spa-francorchamps.json \
  --diagnostics-output docs/tracks/spa-francorchamps-import.json
```

Open-Meteo's Copernicus GLO-90 elevation API is also supported with `--elevation open-meteo` and
does not require Rasterio. It intentionally queries at most 100 points because oversampling a 90 m
raster would add no information.

## 2026-09-19 import results

| Circuit | Selected ways | Mathematical length | Published length | Difference | Elevation |
|---|---:|---:|---:|---:|---|
| Spa-Francorchamps | 30 | 7,014.887 m | 7,004 m | 10.887 m / 0.15544% | Copernicus GLO-30 |
| Monza | 20 | 5,796.971 m | 5,793 m | 3.971 m / 0.06854% | flat for Phase A |
| Silverstone | 27 | 5,890.022 m | 5,891 m | 0.978 m / 0.01661% | flat for Phase A |
| Suzuka | 40 | 5,805.847 m | 5,807 m | 1.153 m / 0.01986% | flat for Phase A |

Spa's raw horizontal OSM polyline is 7,003.968 m. The 3D periodic curve is longer because it includes
elevation. Its ENU origin is 50.434583407° N, 5.969146848° E at 408.062 m; smoothed Copernicus
samples range from 364.106 to 466.899 m absolute elevation. No outliers were replaced.

The published lengths are documented in each output/diagnostic file. Spa uses the circuit operator,
Monza and Silverstone use Formula 1 circuit information, and Suzuka uses the circuit operator.

## Limitations

- OSM is community geometry, not a circuit survey.
- GLO-30 is a digital surface model. Its 30 m terrain/building/vegetation samples do not reproduce
  paving detail, kerbs, or banking. Banking remains zero rather than being invented.
- Half-widths are explicitly labeled generic engineering estimates because the selected OSM
  centerlines do not provide consistent surveyed left/right boundaries.
- Monza, Silverstone, and Suzuka elevation is intentionally flat in this phase; only Spa is the
  elevation showcase.
- Start position and thirds-based sectors are mathematical defaults, not official timing lines.

Data © OpenStreetMap contributors, ODbL 1.0. Copernicus DEM 2021 was accessed from the AWS Open Data
registry; cite `https://doi.org/10.5270/ESA-c5d3d65` in published research.
