# ApexLab Milestone 6 — implementation report

## Completion status

The real-track pipeline, four real circuits, elevated Spa showcase, dense UI redesign, fixed-line
optimizer, validated free-line optimizer on the synthetic validation circuit, comparison product
workflow, sensitivity experiments, and validation tooling are implemented. **Formal Milestone 6
completion is not claimed** because two externally constrained acceptance items remain:

1. the requested Sketchfab P1 archive needs one authenticated manual download;
2. Spa fixed-line solves converged through 400 nodes in the reduced model, but all three attempted
   profiles diverged in production replay. They are retained as rejected experiments and are not
   exposed as optimized Spa laps.

This status avoids fabricating an asset or presenting an unvalidated Spa solution as optimal.

## 1–5. Assets and real tracks

1. **P1 integration:** the importer/processor, visual configuration, runtime auto-detection, node
   inspection, wheel mapping, kinematic steering/spin, provenance, and repository hygiene are
   implemented. Runtime currently uses the orange procedural fallback.
2. **Asset statistics:** Sketchfab reports 241,320 faces, 140,459 vertices, 47 materials, and 36
   textures. Processed triangle count, file size, draw calls, and GPU impact remain null because the
   authenticated source was not downloaded. They are intentionally not invented.
3. **Track pipeline:** `tools/tracks/import_osm_track.py` performs explicit race-course selection,
   ECEF-to-local-ENU conversion, topology ordering, direction correction, duplicate/gap checks,
   resampling, smoothing, arc-length construction, diagnostics, and optional Copernicus sampling.
4. **Spa accuracy:** 7,014.887 m imported versus 7,004 m reference: +10.887 m (+0.155%). Copernicus
   GLO-30 elevation spans 364.106–466.899 m. Elevation is terrain-derived DSM data, not survey data.
5. **Other circuits:** Monza 5,796.971 m (+0.068%), Silverstone 5,890.022 m (−0.017%), and Suzuka
   5,805.847 m (−0.020%). All four real tracks have selectable functional P1 reference sessions.

## 6. UI redesign

The interface now follows the supplied motorsport target: restrained two-level navigation, dominant
3D viewport, session rail, circuit map, physical tire schematic, aero/load and G panels, stacked
telemetry, setup/optimization panel, dense navy engineering styling, and subtle provenance view.
Only modeled/recorded quantities are shown; no fictional gear, RPM, temperature, or pressure appears.

## 7–9. Optimization architecture

7. **Fixed line:** spatial trapezoidal direct collocation minimizes `integral(ds/v)` with periodic
   speed, signed drive/brake command, steering, sideslip, acceleration, nonlinear tire/contact
   constraints, and quasi-steady lateral/yaw equilibrium. The controller lap is the warm start.
8. **Free line:** adds periodic `e_y(s)` and analytic Frenet path metric/heading/curvature, explicit
   CG bounds, a 2.5 m safety margin, and line smoothness constraints. No racing-line rules are used.
9. **Solver:** CasADi 3.8.1 with bundled IPOPT was chosen for sparse automatic differentiation,
   mature constrained NLP support, and reproducible Python tooling isolated from C++ production
   simulation. No generic NLP solver was written in-repository.

## 10–18. Numerical results

10. **Accepted free-line residuals:** speed defect `4.12e-13 m²/s²`, time defect `5.11e-15 s`,
    lateral equilibrium `8.88e-12 m/s²`, normalized yaw `8.96e-14`, track/control/periodic residual
    zero, tire-force excess `1.00e-8` at tolerance.
11. **Fixed-line refinement:** N=40/100/200/400/800 predicted 40.170/42.027/42.500/42.826/42.949 s;
    production replay 42.215/42.210/42.725/43.050/43.175 s. N=800 required N=400 continuation.
12. **Time replay:** fixed N=800 differs by +0.226 s (+0.526%), max/RMS position 0.272/0.048 m.
    Free N=100 differs by +0.613 s (+1.750%), max/RMS position 1.893/0.721 m, with no boundary exit.
13. **Reference time:** 50.710 s controller-driven technical-circuit lap.
14. **Optimized time:** 35.635 s validated production replay of the local free-line result; 15.075 s
    faster than the reference. Fixed-line production baseline is 42.210 s at N=100.
15. **Largest gain:** about 0.936 s over a 25 m region centered near `s = 592 m`.
16. **Line changes:** up to ±5.5 m optimizer offset with 2.5 m declared CG clearance; maximum
    recorded reference/optimized trajectory separation 7.479 m near `s = 644 m`.
17. **Tire use:** spatial mean maximum-wheel utilization rises from 69.84% to 98.78%; samples above
    95% rise from 20.96% to 92.95%; both production laps reach the force cap.
18. **Runtime:** free N=100 uses 701 variables, 2,202 constraints, 1,238 iterations, and 7.267 s.
    Fixed N=800 uses 4,801 variables, 15,202 constraints, 790 iterations, and 20.570 s.

## 19–23. Product, tests, limitations, next milestone

19. **Rendering:** Apple M2 Pro headless measurement at 1512×1100: 16.667 ms average frame
    interval, 16.9 ms p95, 26.4 ms seek-to-paint, 515.7 ms Spa session load. The showcase uses one
    668-instance forest draw architecture plus bounded terrain and track environment meshes.
20. **Validation:** C++ unit/integration suite, Python importer/asset tests, 31 dashboard tests,
    production build, and 10 browser E2E cases are the required gate. See the exact final run in the
    handoff; rejected solver/replay experiments remain explicitly rejected.
21. **Screenshots:** `milestone-6-spa-p1.png` and `milestone-6-reference-vs-optimized.png` plus the
    retained M5 regression states under `docs/screenshots/`.
22. **Known limitations:** authenticated P1 source absent; procedural fallback active; no validated
    optimized Spa replay; free-line refinement beyond N=100 needs improved numerical continuation;
    grade is rendered/exported but not a production force; CG boundary only; no body envelope or
    banking; planar/quasi-static tires; local rather than global optimization.
23. **Recommended Milestone 7:** complete the licensed P1 download/processing, add robust
    continuation/scaling and smooth out-of-domain load handling, validate 200–800 node free-line
    refinement, incorporate grade force in both production and optimizer models, and only then
    produce and expose a validated Spa racing-line replay. Do not add ABS, TC, thermal tires,
    detailed differential behavior, or multiplayer before those integrity gaps close.

## Sensitivity experiments

At N=100 the grip cases μ=0.90/1.20/1.35 produce 47.339/42.027/40.709 s. Mass cases
1,200/1,450/1,600 kg produce 42.740/42.027/42.211 s. These are local reduced-model solutions; the
non-monotonic low-mass result is retained rather than “corrected,” and no OEM interpretation is made.
Machine-readable inputs/results are under `data/generated/optimization/sensitivity/`.

## Spa rejected optimization experiment

Spa fixed-line NLPs converged at N=100/200/400 with predicted times 174.021/182.630/188.077 s and
spacing 70.149/35.074/17.537 m. A guard for fractional load powers at infeasible IPOPT trial loads
enabled refinement while leaving the accepted `load >= 50 N` formula unchanged. Independent replay
failed to complete any case in 300 s; N=400 still reached 232.673 m maximum path error and
226.673 m track violation. Therefore the Spa UI remains a feasible heuristic reference lap and
never labels a rejected solve as optimized. Large rejected replay CSVs were removed after their
small diagnostics JSON files were retained; they are reproducible from the documented commands.
