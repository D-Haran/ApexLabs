# Milestone 6 — Phase A results

Date: 2026-09-19

Status: **complete with one documented external-asset handoff**. The repository is runnable and all
Phase A tests pass. Phase B has not started.

## McLaren P1 asset

The requested Sketchfab model metadata is recorded in
`assets/external/mclaren-p1/asset-manifest.json`: creator Kevin Love SketchFab / `Tyler_Kevin`, CC
BY 4.0, published 2026-08-07. The public API reports 241,320 faces, 140,459 vertices, 47 materials,
and 36 textures. “Faces” is retained as the API's term and is not mislabeled as a measured triangle
count.

Sketchfab's download endpoint returned HTTP 401 without user authentication. No mesh was fabricated
or replaced with a different P1. The only manual action is to download glTF while signed in and put
the untouched archive at `assets/external/mclaren-p1/raw/mclaren-p1-sketchfab.zip`.

The completed local pipeline provides:

- dependency-free GLTF/GLB structure, triangle, draw-call, material, texture, and node inspection;
- safe ZIP extraction;
- explicit axis, scale, pivot, body, and wheel mapping gates that refuse unknown values;
- Blender normalization without merging named wheel/body nodes;
- Meshopt geometry and KTX2/Basis texture compression through glTF-Transform;
- conservative 85% mesh simplification with a small error tolerance;
- before/after manifest statistics once a real source file exists;
- ignored raw/work/processed outputs so a large asset cannot enter Git accidentally.

Processed triangle count, processed file size, wheel separability, and GPU impact remain unknown
because the source asset is not present. These values are intentionally `null`, not estimates.

## P1 model configuration

`configs/vehicles/mclaren-p1-approx.json` loads successfully in the production simulator. Its UI
name is **McLaren P1 — approximate model**. `docs/vehicles/mclaren-p1-approx.md` classifies every
parameter as published, derived, estimated, or fitted and separately lists unknown dynamics data.
The visual configuration is independent at `configs/visuals/mclaren-p1.json`, with orange-metallic
paint, wheel material, ride offset, scale, camera target, and procedural fallback.

## Real-track pipeline

`tools/tracks/import_osm_track.py` supports Spa-Francorchamps, Monza, Silverstone, Suzuka, or an
explicit bounding box. It performs closed-cycle topology selection, direction correction,
duplicate cleanup, gap rejection, metric ENU projection, 10 m resampling, periodic reconstruction,
arc-length validation, and source diagnostics. The selected physics course and render-only facility
context are separate files.

Track schema v2 adds elevation and coordinate provenance while schema v1 synthetic fixtures remain
loadable. The C++ representation now provides local `x/y/z`, grade, and 3D arc length. The planar
vehicle equations do not yet apply grade forces; that limitation is explicit.

| Circuit | Imported / reference | Difference | Status |
|---|---:|---:|---|
| Spa-Francorchamps | 7,014.887 / 7,004 m | +10.887 m (0.15544%) | elevated showcase geometry |
| Monza | 5,796.971 / 5,793 m | +3.971 m (0.06854%) | functional, flat |
| Silverstone | 5,890.022 / 5,891 m | −0.978 m (0.01661%) | functional, flat |
| Suzuka | 5,805.847 / 5,807 m | −1.153 m (0.01986%) | functional, flat |

No circuit was length-scaled. Spa's raw horizontal OSM centerline is 7,003.968 m; the mathematical
length increases in 3D. Copernicus GLO-30 terrain samples span 364.106–466.899 m absolute elevation
and are smoothed longitudinally. They are DSM-derived terrain, not laser-scanned paving or banking.
Full origins, selected way IDs, competing cycles, sources, and commands are in
`docs/tracks/osm-import.md` and the per-track diagnostic JSON files.

## Validation

Commands:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
cd apps/dashboard
npm test
npm run build
```

Results:

- CTest: **8/8 passed**, including legacy/elevated track geometry, four imported track lengths,
  Spa elevation/grade bounds, importer topology behavior, asset inspection, and P1 configuration loading.
- Dashboard unit tests: **30/30 passed**.
- Dashboard Playwright regression: **8/8 passed** on the Apple M2 Pro development machine.
- TypeScript + Vite production build: **passed**.
- The existing large-bundle warning remains (1,331.76 kB minified / 385.19 kB gzip); Phase A did not
  alter the dashboard bundle.
- Python pipeline sources compile successfully.
- Running the asset processor without the manual archive exits with a precise actionable error,
  as intended.

## Phase A gate

The gate is green. Phase B can begin after—or independently of—the one authenticated Sketchfab
download; the procedural vehicle remains a supported fallback, so the missing external binary does
not block circuit/environment work.
