# Milestone 5 — recorded engineering application

Preserve Milestones 1–4 and their physics. Baseline verification: Release CTest and all four
Python experiment suites run before implementing the client. This is a local React/TypeScript,
Vite, React Three Fiber / Three.js application; no hosting, live transport or new physics.

## Data boundary

Keep schema-v4 CSV intact. Export a version-1 session manifest, exact C++ sampled track geometry,
a timestamp/step-aligned wheel-state companion (existing `LapTelemetrySample::forces`, no force
recomputation), simulator lap/sector results and metadata. Export distance samples with the
existing C++ spatial-resampling utility and monotonic unwrapped-progress time interpolation.
A packaging command creates self-contained demo directories. One centrally maintained typed
contract validates all files, versions, finite required quantities and alignment. Missing data
is an error, never a plausible numeric default. Decode sessions in a Web Worker.

## Client boundaries

- `telemetry`: contract, strict loaders, binary-search interpolation and spatial seeks.
- `state`: one simulation-time cursor; play/pause/seek/restart/rate/frame-step.
- `analysis`: deterministic extrema/saturation events, spatial deltas, regional summaries.
- `three`: exported road ribbon, asset adapter, cameras and physical-state overlays.
- `components`: map, tire/friction widgets, canvas telemetry plots (uPlot), analysis controls.
- `workers`: parsing and analysis preparation away from the render thread.

All displayed state shares one interpolated snapshot. Only cameras are smoothed. Pose is mapped
from simulator `(X,Y,yaw)` to Three `(X,up,-Y)`. Continuous telemetry is linearly interpolated,
yaw/heading use shortest angular distance, flags/categories are left-held. Lap seams are explicit.

Comparison uses exported spatial grids and elapsed timestamps: delta = B − A; negative means B
is ahead. Require identical track geometry/length and reference offset. Restrict regional
statistics and comparison to shared coverage; no seam extrapolation. Lap times remain simulator
results, distinguishable from interpolated spatial interval times.

## Delivery and evidence

Build a restrained charcoal desktop workspace with replay, four cameras, force/load overlays,
clickable map/events/charts, all required telemetry channels, tire inspection, region analysis
and two-lap comparison. Include a permitted procedural placeholder and GLB/GLTF adapter.
Tests cover synchronization, wrap interpolation, discrete fields, seeks, comparison, events,
invalid schema/missing values, seams and exporter integrity. Verify the application in-browser,
capture major states, measure actual frame intervals, seek and load latency, and report the
hardware/browser and measurement limitations. Update README and experiment documentation.

## Completion

Implemented and verified: [architecture and data policy](milestone-5-replay.md),
[results, measurements and screenshots](../experiments/milestone-5-results.md).
The implementation required only an additive CLI presentation exporter; the core models and
schema-v4 CSV writer remained unchanged. Release and UBSan CTest, all M1–4 Python suites,
30 frontend logic tests, 8 production browser tests and multi-lap export integrity checks pass.
