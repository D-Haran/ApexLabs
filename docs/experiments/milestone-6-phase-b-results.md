# Milestone 6 Phase B — real-circuit rendering

Status: **implemented and gated**. The runtime can load the exported Spa session and renders the
same elevated metric centerline used by the simulator. The preferred Sketchfab P1 source remains
unmodified and unavailable until the documented authenticated download is completed; ApexLab
therefore displays an explicitly labelled orange procedural P1 fallback.

## Runtime result

- Circuit: Spa-Francorchamps, OpenStreetMap geometry, 7,014.888 m mathematical 3D length.
- Elevation: Copernicus DEM 2021 GLO-30, 364.106–466.899 m absolute elevation. This is
  terrain-derived DSM data, not survey-grade paving geometry.
- Vehicle physics: `McLaren P1 — approximate model`; public headline specifications plus labelled
  engineering estimates. It is not McLaren proprietary dynamics data.
- Vehicle visual: procedural orange fallback. The fallback supports front-wheel steering and
  kinematic wheel spin derived from speed and a 0.33 m visual radius. Spin is not a physical wheel
  state.
- Environment: elevated asphalt, painted boundaries, illustrative red/yellow kerbs, grass/verges,
  runoff, guardrails, basic fencing, start/finish marking, gap-checked OSM pit-lane context, bounded
  terrain and 668 procedurally varied GPU-instanced trees.

Render details do not feed back into track mathematics. In particular, kerb placement, barrier
offsets, vegetation and pit-road width are visualization layers and are not collision or grip
inputs. Kerb placement is presently illustrative rather than a surveyed corner-by-corner Spa
inventory.

## Alignment

`geometry.csv` is emitted from `Track::position_3d_at_s()` and is the only source used to construct
the render ribbon. An automated dashboard test checks four known Spa rows at the start, interior
locations and periodic endpoint. It verifies:

1. physics `(x, y, elevation)` maps to render `(x, elevation, -y)`;
2. the width-weighted center reconstructed from left/right render boundaries returns the physics
   center.

The assertions use 9–10 decimal-place tolerances. OSM render-context ways are split at planar gaps
above 35 m or sampled elevation jumps above 3 m so ambiguous source fragments are never spanned by
a rendered polygon.

## Measured performance

The deterministic capture is `docs/screenshots/milestone-6-spa-p1.png`; raw measurements are in
`docs/experiments/milestone-6-phase-b-performance.json`.

| Measurement | Result |
|---|---:|
| Platform | Apple M2 Pro, Chromium/ANGLE Metal |
| Viewport | 1512 × 1100 @ DPR 1 |
| Session load | 558.4 ms |
| Average frame interval, 300 frames | 16.667 ms |
| p95 frame interval | 17.300 ms |
| Selector seek average / p95 | 0.099 / 0.200 ms |
| Seek-to-paint | 361.2 ms |
| Exported Spa session size | 26 MiB |

The seek-to-paint result is substantially higher than the synthetic M5 fixture and remains a Phase
G optimization target. No production P1 load time, processed triangle count, file size, texture
memory, or GPU impact is reported because the authenticated source file is not present. Reporting
zeros would be misleading.

## Gate

- C++/Python CTest suite: 8/8 passing.
- Dashboard unit tests: 31/31 passing, including real-track render alignment.
- TypeScript production build: passing; the existing 1,339.69 kB large-bundle warning remains.
- Browser suite: 9/9 passing, including the Spa/P1 smoke, visual capture and M2 Pro performance run,
  with no page or scene errors.

Phase B is green and Phase C may begin.
