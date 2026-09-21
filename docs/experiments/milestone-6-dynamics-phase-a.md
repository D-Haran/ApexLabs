# Milestone 6 dynamics — Phase A

Validated 2026-09-20 on Apple M2 Pro. **Phase A is implemented; the expanded Milestone 6 is not
complete.** This report follows the newer attached dynamics brief, not the older M6 asset/optimizer
brief shown in the IDE selection. [Remaining phase gates](../architecture/milestone-6-dynamics-plan.md).

## Implemented

- Independent `ContactVehicleModel`: world XYZ, quaternion attitude, linear/angular velocity,
  four unsprung extension/rate states, coupled mass matrix and unilateral normal tire contact.
- Existing nonlinear combined-force tire law, evaluated using actual contact loads and individual
  surface friction. Airborne tire Fx/Fy/Fz are exactly zero, including with both pedals commanded.
- Spring/damper suspension, compliant tire landings, bump/droop stops and explicit contact telemetry.
- Per-wheel metric road queries with normal/material/height, physical synthetic curbs, asphalt
  runoff and rough grass. Leaving asphalt does not reset the state.
- Native scene exporter and a separate contact validation viewer using the existing licensed P1
  GLB. Body pose and wheel centers come from recorded simulation, without road-height grounding.
- Exported road mesh checked against its native surface query; source physics geometry is not
  reconstructed or modified by the viewer.

The longitudinal, linear bicycle, nonlinear planar four-tire, reference-lap, passive sprung-body
and existing optimization systems remain independently runnable. The main Spa replay still uses
its preserved M5.5 passive model. No old solver result has been promoted to a new 3D optimum.

## Reproduce

From the repository root:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
python3 tools/python/validation/run_contact_validation.py

cd apps/dashboard
npm test
npm run build
npm run test:e2e -- contact.spec.ts
npm run dev
```

Open **http://127.0.0.1:5173/?contact**. Select crest 12/40 m/s, the 0.5 m drop, left curb or grass
coast; play, pause, seek and reset the recorded interval. `/` still opens the existing Spa session.
These intervals are never represented as completed laps. The native executable also runs directly:

```sh
./build/apps/sim-cli/apexlab-contact-scenes \
  configs/vehicles/mclaren-p1-sprung-approx.json \
  data/generated/contact-3d/dt-002 .002
```

`data/generated/contact-3d/validation.json` contains machine-readable metrics for all 30 runs.
Dense CSVs and per-run meshes under `dt-*` are ignored and regenerated. The small summary and five
browser-ready interval files are retained. The source asset/GLB ignore policy is unchanged.

## Crest transition

The synthetic crest is `z(x)=2 cos^4(pi x/70)` inside ±35 m, joining a flat approach with continuous
height, slope and curvature. Apex radius is approximately 62.059 m. The no-aero mechanical fixture
coasts onto it from x=−45 m. These are **not** Spa or published McLaren test results.

| Initial speed (m/s) | Speed near apex (m/s) | Required constrained vertical acceleration near apex (m/s²) | FL/FR/RL/RR normal force near apex (kN) | Total full-airborne time (s) | Max body CG height above its local static height (m) | Peak total normal force (kN) |
|---:|---:|---:|---|---:|---:|---:|
| 12 | 10.249 | −1.693 | 2.604 / 2.604 / 3.461 / 3.461 | 0 | 0.00461 | 16.454 |
| 20 | 18.996 | −5.814 | 1.269 / 1.269 / 1.643 / 1.643 | 0 | 0.02406 | 20.141 |
| 28 | 27.267 | −11.977 | 0 / 0 / 0 / 0 | 0.640 | 0.26622 | 46.411 |
| 36 | 35.373 | −20.097 | 0 / 0 / 0 / 0 | 1.922 | 1.56259 | 97.041 |
| 40 | 39.411 | −24.896 | 0 / 0 / 0 / 0 | 2.018 | 2.23184 | 105.820 |
| 44 | 43.443 | −30.198 | 0 / 0 / 0 / 0 | 2.060 | 2.82211 | 117.522 |

Airborne duration sums all full-airborne intervals, including rebound. At 40 m/s the first interval
starts at 0.950 s and first landing begins at 2.114 s; it is not a single 2.018 s flight. The short
40 m/s recording ends with two contacts during rebound. The reported body-height metric subtracts
the 0.45 m static body height; it is not a full vehicle underside clearance calculation.

The extreme cases exceed the small-deflection interpretation of the linear tire and suspension
surrogates. In the 40 m/s stress case, peak tire compression is 214 mm and peak spring compression
236 mm. Those are explicit model limitations, **not credible calibrated P1 impact predictions**.
The sweep establishes unilateral contact and numerical convergence, not structural survivability
or realistic high-impact tire bottoming. A nonlinear tire bottoming/damage model and body collision
would be necessary before interpreting such severe impacts as real-car results.

## Timestep refinement and landing

| Physics step | 40 m/s total airborne time | Max excess body height | Peak total normal force |
|---:|---:|---:|---:|
| 2 ms | 2.0180 s | 2.231841 m | 105,819.60 N |
| 1 ms | 2.0180 s | 2.231861 m | 105,978.70 N |
| 0.5 ms | 2.0175 s | 2.231855 m | 105,999.05 N |

The peak force differs by about 0.17% between 2 and 0.5 ms. Across all ten scenarios, the validator
requires airborne duration differences ≤10 ms, excess-height differences ≤5 mm and peak total
normal-force differences ≤3%. It also rejects nonfinite data, negative Fz, nonzero airborne tire
force, invalid contact flags, utilization above one and quaternion drift.

The 0.5 m drop first contacts at 0.300 s, has 0.428 s total airborne time including bounce, and
peaks at 66.721 kN total normal force. Its largest individual wheel load is 19.235 kN; additional
spring compression reaches 101.9 mm beyond static. The test requires the body to settle within
2 mm of static height after five seconds. Per-step body movement remains below 20 mm at 2 ms;
no landing reset or imposed road-normal constraint is used.

## Analytical and surface validation

- Static body/wheel force and moment balance, and no equilibrium drift over one second.
- Rotating airborne vehicle: total COM follows the analytical ballistic trajectory within the
  2 μm test tolerance over 0.5 s; angular momentum about COM is conserved within the test tolerance
  of 1e−4 N·m·s; passive suspension does not create mechanical energy.
- High upward separation velocity produces zero normal force rather than tensile road support.
- Contact tire output matches the existing nonlinear tire law for the same load/slip/demand.
- Mirrored rear sideslip produces mirrored road-on-vehicle lateral forces with zero rear steer.
  This is not a substitute for Phase F's full steady-turn force audit.
- An asymmetric left bump changes the left wheel loads without moving the right wheel road plane.
- Curbs, runoff, grass friction and roughness, periodic seams and world/track projection are checked.
- Sixteen distributed Spa locations exercise all four independent elevated road queries. This is
  a geometry-query check, not a completed 3D Spa lap or a Spa airborne-event claim.
- Maximum render-mesh triangle-centroid height error is **0.4473 mm on the crest** and **0.3822 mm
  on the curb/grass surface**, against the 1 mm gate. Flat/drop errors are zero.

The recorded mixed force/moment mass-matrix equation residual is below 4.5e−11 for the 40 m/s case.
That is linear-solve roundoff, **not** an optimizer dynamic-defect or real-world model error metric.

## Verification and performance

| Check | Result |
|---|---|
| Release CTest, including preserved systems and native contact CLI | 12 / 12 passed |
| Debug + UndefinedBehaviorSanitizer CTest | 12 / 12 passed |
| Dashboard unit tests | 50 / 50 passed |
| TypeScript + production Vite build | Passed; existing large-bundle advisory remains |
| Focused Playwright contact replay and preserved Spa checks | 2 / 2 passed |
| Ten native scenarios at three timesteps | 30 / 30 passed |

In the recorded Release run, the 2 ms / 40 m/s experiment advances 3.5 simulated seconds in
0.079 s wall time, including CSV writing. The curb case advances 6 s in 0.244 s. Surface export,
mesh validation and Python packaging are excluded from those per-run times. These offline bench
measurements do not establish live-driver transport latency, full Spa throughput or rendering FPS.
Those remain later gates.

The Browser connector had no available browser. The repository's Playwright runner exercised the
UI and produced screenshots that were visually inspected:

- [Airborne P1 with four zero-force tires](../screenshots/m6-contact-airborne.png)
- [Physical left-curb replay](../screenshots/m6-contact-curb.png)

These illustrate the new contact behavior; they are not the final target-matched Spa screenshots.

## Remaining milestone work

Phase B is next: separately parameterized and calibrated P1 / MCL36-inspired vehicles, distinct aero
and drivetrains, and sourced parameter classes. Fuel/energy/thermal/wear states, real-time human
driving, optimized ghost and live delta, new-model multi-objective optimization, full force audit,
the final target UI/environment pass and vehicle-specific Spa validation remain unimplemented.
No claims are made for new optimized lap times, P1/F1 racing-line differences or final M6 completion.
See [model assumptions](../physics/contact-3d-model.md) for the precise current physical scope.
