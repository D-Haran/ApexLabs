# Milestone 6 Phases E–F — Racing line and product integration

## Status

The technical validation circuit has a validated **locally optimized racing-line solution** and a
self-contained product session. No global-optimality claim is made. The result is produced by the
same CasADi 3.8.1/IPOPT stack as Phase D and is replayed through the independent C++ production
dynamics before acceptance.

## Free-line formulation

The Phase D states and controls are augmented with lateral offset `e_y(s)`. The displaced path is
represented in the centerline Frenet frame. Its metric, heading, and curvature are derived from
`e_y`, `de_y/ds`, `d2e_y/ds2`, centerline curvature, and centerline-curvature derivative. This
formulation exactly reduces to the imported centerline at zero offset and avoids inaccurate sparse
chord curvature.

The CG constraint is:

```text
-right_width(s) + 2.5 m <= e_y(s) <= left_width(s) - 2.5 m
```

The 2.5 m value is a replay safety margin, not a body-envelope collision model. Cyclic indices
enforce a closed periodic line. Slope and second-derivative bounds suppress mesh-scale zig-zag.
No apex, entry, track-out, or other handcrafted racing-line rules are present.

## Accepted result

| Quantity | Value |
|---|---:|
| Nodes | 100 |
| Decision variables / constraints | 701 / 2,202 |
| IPOPT iterations / wall time | 1,238 / 7.267 s |
| Fixed-line reduced-model time | 42.027 s |
| Free-line predicted time | 35.022 s |
| Production replay time | 35.635 s |
| Replay minus prediction | +0.613 s (+1.750%) |
| Reference controller lap | 50.710 s |
| Production gain vs reference | 15.075 s |
| Maximum / RMS path tracking error | 1.893 / 0.721 m |
| Maximum / RMS speed error | 1.873 / 0.681 m/s |
| Maximum / RMS yaw error | 0.442 / 0.132 rad |
| Maximum track violation | 0 m |
| Maximum production tire utilization | 1.0000000000000004 |

The optimizer's maximum speed defect is `4.12e-13 m²/s²`, time defect `5.11e-15 s`, lateral
equilibrium residual `8.88e-12 m/s²`, normalized yaw residual `8.96e-14`, and reported tire-force
violation `1.00e-8` at the numerical tolerance. The machine-readable acceptance artifact is
`data/generated/optimization/technical-racing/validation-center-n100.json`.

## Local-solution sensitivity

Center and left-biased initializations converged to 35.022 s and 35.092 s respectively at the
100-node transcription. Right-biased and heuristic starts reached the iteration limit and retained
residuals, so they were rejected. The experiment demonstrates local sensitivity; it does not
support a global optimum claim. Refinement to 200 nodes also failed the residual/iteration gate and
is not reported as a solution.

## Where the lap changes

The production replays show the largest 25 m local time gain around `s = 592 m`, worth about
`0.936 s` over that interval. Maximum recorded trajectory separation is `7.479 m` near `s = 644 m`.
The reference replay averages 69.84% maximum-per-axle tire utilization across the spatial grid and
spends 20.96% of samples above 95%; the optimized replay averages 98.78% and spends 92.95% above
95%. Both peak at the production force cap. These values explain the large gain but also show why
the independent replay and explicit margin are mandatory.

## Product integration

`apps/dashboard/public/demo/technical-optimized` is a complete schema-v4 session generated from
the production replay. The dashboard supports Reference, Optimized, and Compare modes. Comparison
adds:

- reference and optimized lines on the track map and 3D scene;
- synchronized overlaid telemetry and `delta_t(s)` / speed delta;
- reference/optimized lap and sector results;
- selectable region metrics;
- largest-gain, braking-shift, tire-use, and trajectory-deviation events;
- explicit local-optimization and production-replay provenance.

The normal landing state remains the Spa/P1 showcase. Selecting the technical circuit loads the
reference and optimized sessions together.

## Reproduce

```sh
.cache/optimizer-venv/bin/python tools/optimization/racing_line_optimizer.py \
  --vehicle configs/vehicles/generic_nonlinear_performance_car.json \
  --session apps/dashboard/public/demo/baseline \
  --warm-start-trajectory data/generated/optimization/technical-fixed/fixed-line-n100.csv \
  --fixed-line-time 42.0270 \
  --output data/generated/optimization/technical-racing \
  --nodes 100 --initialization center --margin 2.5 --warm-speed-scale 1.0

python3 tools/python/replay/export_session.py \
  --vehicle configs/vehicles/generic_nonlinear_performance_car.json \
  --track configs/tracks/technical_test_circuit.json \
  --output apps/dashboard/public/demo/technical-optimized \
  --name 'Technical circuit · locally optimized racing line' \
  --warmup-laps 0 --laps 1 --maximum-duration 120 \
  --optimization-profile data/generated/optimization/technical-racing/racing-line-center-n100.csv \
  --replay-diagnostics data/generated/optimization/technical-racing/replay-center-n100.json

.cache/optimizer-venv/bin/python tools/optimization/validate_racing_line.py \
  --optimizer data/generated/optimization/technical-racing/racing-line-center-n100.json \
  --replay data/generated/optimization/technical-racing/replay-center-n100.json \
  --output data/generated/optimization/technical-racing/validation-center-n100.json
```

