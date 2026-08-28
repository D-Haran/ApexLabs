# Milestone 5.5 — physical vehicle integration and environment

Validated 2026-09-20 on Apple M2 Pro. The attached **Milestone 5.5** specification takes precedence
over the IDE-selected M6 brief. No new minimum-time optimization was undertaken; existing M6
artifacts were preserved. The application opens the new **Spa / passive sprung-body P1 reference**
session. Visual selection can switch independently between the supplied P1 and F1 meshes.

## Assets and provenance

| Supplied file | Metadata identification | Author | License |
|---|---|---|---|
| `assets/cars/mclaren_p1/scene.gltf` | Mclaren P1 | Kevin Love SketchFab / Tyler_Kevin | CC BY 4.0 |
| `assets/cars/gulf_mclaren_f1_2022_car/scene.gltf` | Gulf Mclaren F1 2022 Car | chwashere123 | CC BY 4.0 |

Both packages have their required binary and texture dependencies. Identification reads embedded
metadata and license files, not generic `Object_*` names. Original source units are unspecified.
See [asset commands and source links](../../assets/cars/README.md). No manual download is required
in this workspace. Raw packages and processed GLBs are ignored to prevent accidental Git bloat;
a fresh checkout needs the source packages and documented processing commands. This workspace
itself has no `.git` directory, so there is no commit/diff history to report.

`process_car_asset.py` flattens source scene transforms, identifies material roles, welds coincident
positions, computes true triangle connected components, and merges concentric tire shells by
spatial proximity. Four wheel clusters must form two valid axle pairs. Invalid dependencies,
unsupported source features and ambiguous classification fail clearly. Explicit forward direction
and wheelbase normalization are command inputs. Classification never uses Object node numbers.

Textures remain embedded, capped at 1024 pixels. Lossless Meshopt compression verifies every
decoded buffer byte-for-byte. No body simplification was needed. P1's alternate blurred rims and
damage-glass layers are excluded; the F1 triangle count is unchanged. Draco/KTX2/LOD were considered
but not introduced because current frame measurements did not justify additional runtime tooling.

| Measured asset statistic | P1 | F1 2022 |
|---|---:|---:|
| Source triangles | 241,320 | 1,050,630 |
| Processed triangles | 228,936 | 1,050,630 |
| Source package bytes | 16,772,489 | 67,774,007 |
| Uncompressed processed bytes | 16,493,672 | 66,127,396 |
| Compressed GLB bytes | 11,751,168 | 29,951,804 |
| Mesh primitives | 88 | 59 |
| Estimated texture bytes, including mipmaps | 69,847,873 | 39,146,834 |

## Axes, wheel mappings and grounding

Source scene convention is +Y up / +Z forward, confirmed using P1 lighting geometry and the F1
nose/wing silhouette. Runtime is **+X forward, +Y up, -Z left**; physical world XYZ maps to render
(X,Z,-Y). The source axle midpoint becomes the visual ground origin. Runtime places that midpoint
relative to the physical CG, without scaling the artwork to another physical model.

P1 wheelbase normalization uses the existing published **2.67 m** anchor. F1 **3.6 m** is an explicit
engineering visual assumption, not an official measured McLaren F1 chassis specification. Therefore
the table contains mesh dimensions *after* this normalization, not independent vehicle surveys.

| Quantity | P1 | F1 2022 |
|---|---:|---:|
| Meters/source scene unit | 1.000108164 | 0.776324657 |
| Wheelbase m | 2.670000000 | 3.600000000 |
| Front track m | 1.644017462 | 1.649972438 |
| Rear track m | 1.586405697 | 1.550293972 |
| Global vertical grounding correction m | 0.000107717 | -0.049205591 |

| Asset / corner | Steer → spin | Radius m | Tire / rim / disc / stationary brake components |
|---|---|---:|---|
| P1 FL | `FL_STEER` → `FL_SPIN` | 0.324966335 | 1 / 8 / 1 / 2 |
| P1 FR | `FR_STEER` → `FR_SPIN` | 0.324966335 | 1 / 8 / 1 / 2 |
| P1 RL | `RL_STEER` → `RL_SPIN` | 0.347415932 | 1 / 8 / 1 / 2 |
| P1 RR | `RR_STEER` → `RR_SPIN` | 0.347415932 | 1 / 8 / 1 / 2 |
| F1 FL | `FL_STEER` → `FL_SPIN` | 0.352561247 | 1 / 3 / 0 / 0 |
| F1 FR | `FR_STEER` → `FR_SPIN` | 0.353487393 | 1 / 3 / 0 / 0 |
| F1 RL | `RL_STEER` → `RL_SPIN` | 0.368441743 | 1 / 3 / 0 / 1 |
| F1 RR | `RR_STEER` → `RR_SPIN` | 0.368441720 | 1 / 4 / 0 / 1 |

`BODY` contains non-wheel geometry. Tire/rim/disc nodes are children of the corresponding spin
pivot; calipers are siblings of spin under steer. Rear steer is zero. Exact node arrays, centers,
source corrections and per-wheel contact corrections are in the [P1 manifest](../../apps/dashboard/public/assets/mclaren-p1.json)
and [F1 manifest](../../apps/dashboard/public/assets/mclaren-f1-2022.json). F1 source brake geometry
contains rear low-detail blocks only, conservatively kept stationary; absent front brakes/discs
are not fabricated.

Grounding derives from tire bounds and measured radii, including recorded per-wheel corrections.
Each visual contact follows the same piecewise elevated road mesh. There is no hand-tuned lift
offset. Chassis motion rotates about the configured physical CG; wheels remain road constrained.
Debug shows road and body axes, visual wheel centers/contacts, CG, body origin and recorded
compression/heave/roll/pitch, alongside existing physical tire force vectors. A missing physical
CG value is not replaced with a displayed invented CG.

Front steering comes directly from recorded equivalent steering: FL=FR=delta (no Ackermann model).
Visual spin uses the rigid-body contact velocity
`(vx-r*y)*cos(delta)+(vy+r*x)*sin(delta)`, integrated over recorded timestamps and divided by each
mesh radius. It is seek/rate independent and gives distinct inside/outside rotation. This is a
kinematic visual approximation, not wheel rotational dynamics, slip ratio, wheelspin, ABS or TC.

## Physical model

The original quasi-static branch remains the default. New `normal_load_model: sprung_body` selects
`RoadVehicleModel`, with the same production nonlinear tires and suspension-supplied normal loads.
No extra game physics engine or duplicate quasi-static transfer is involved.

Road frame uses normalized `T=(tx,ty,grade)`, horizontal left `L`, and `N=T×L`, with bank exactly
zero. World gravity is `(0,0,-9.80665)` and is projected into the road/body frame. Existing track s
remains horizontal arc length; road velocity maps consistently to horizontal world kinematics.
The full derivation, signs, yaw mapping and limitations are in
[road-sprung-body-model.md](../physics/road-sprung-body-model.md).

For heave h (up), roll phi (left side up), pitch theta (nose up), corner position `(x_i,y_i)`:

```
c_i = -h - y_i*phi - x_i*theta
F_i = F_i0 + k_i*c_i + damping_i*c_dot_i
m*hdd = sum(F_i) - m*g_normal - downforce
Ix*phidd = sum(y_i*F_i) + h_CG*sum(Fy_tire_i)
Iy*thetadd = sum(x_i*F_i) + h_CG*sum(Fx_tire_i)
```

Static preloads F_i0 remain fixed at level-road equilibrium, so changing grade changes spring
settling rather than silently resetting preload. Damping selects compression/rebound coefficients.
Negative support or excessive travel fails explicitly. Coupled vehicle/body states use RK4;
normal loads feed the unchanged nonlinear tire law at every substage. Aero acts at CG.

| New physical parameter | Value | Provenance |
|---|---|---|
| Front corner springs | 65,000 N/m each | estimated generic passive model |
| Rear corner springs | 75,000 N/m each | estimated generic passive model |
| Compression damping | 3,500 Ns/m each | estimated |
| Rebound damping | 4,500 Ns/m each | estimated |
| Nominal ride-height/travel envelope | 0.12 m each | estimated |
| Roll / pitch inertia | 650 / 2,200 kg m² | estimated |

Mass 1,490 kg and previous P1 inputs retain their [existing provenance](../vehicles/mclaren-p1-approx.md).
These are not McLaren proprietary suspension specifications. All mass is lumped as sprung; this
does not reproduce P1 interconnected active/hydraulic suspension. The F1 mesh does not select an
unvalidated F1 physics model.

## Independent numerical results

- **Static:** front corners 3,068.500785 N each; rear 4,237.453465 N each; sum=mg and body at rest.
- **Grade:** -10°, -5°, 0°, +5°, +10° force-free acceleration agrees with `-g*sin(alpha)` within
  1e-10 m/s². Six-second ±10° coast runs end at 9.782558591 / 30.217441409 m/s from 20 m/s;
  speed error is 5.58e-13 m/s.
- **Pitch:** ±6,000 N longitudinal force gives ±0.005438319 rad (0.312°), with braking nose-down,
  front compression and rear extension. Axle load transfer 1,011.235955 N matches `h_CG*Fx/L`.
- **Roll:** ±6,000 N lateral force gives ±0.014530040 rad (0.833°); outside suspension compresses.
  Independent stiffness-matrix equilibrium agrees to below 3e-17 in tested body coordinates.
- **Damping:** analytical natural frequency 2.019913 Hz; expected damped frequency 1.974207 Hz,
  measured 1.975309 Hz. Expected damping ratio 0.211525, measured 0.211528. Maximum analytical
  displacement error 9.29e-12 m for the decoupled oscillator fixture.
- **Energy:** free damped oscillator energy falls monotonically from 12 J to 6.44e-23 J.

Maximum transient body-position error against a 1.25 ms reference:

| Case | 10 ms | 5 ms | 2.5 ms |
|---|---:|---:|---:|
| Braking | 2.344e-09 | 9.715e-10 | 2.140e-10 |
| Cornering | 7.998e-09 | 8.135e-10 | 1.811e-10 |
| Free transient | 6.629e-08 | 2.698e-08 | 3.014e-09 |

Errors decrease; no blanket fourth-order convergence claim is made across the piecewise damping
switch. These are maxima over mixed h/phi/theta coordinates in their SI units, not a world-position
tracking error. The validation JSON retains full precision.

Seven C++ six-second visual validation intervals additionally check flat/grade, left/right turns,
braking and acceleration. All remain on road. Left/right body roll reaches ±0.019991136 rad;
braking pitch -0.003074135 rad, accelerating pitch +0.003534659 rad. They are explicitly marked
**validation interval — not a lap** in the application.

The complete new Spa reference lap is **277.010 s**, 55,403 timed samples at 5 ms physics / 20 ms
controller, after one settling lap. It has no recorded CG track-boundary violation and maximum
lateral error **0.575236 m**. Maximum absolute heave is **6.561 mm**, roll **1.912°**, pitch **0.686°**.
This is a controller-driven reference lap, with terrain-derived grade and approximate parameters;
it is not a real P1 performance claim or an optimized result. Legacy technical reference remains
**50.710 s**.

## Rendering and environment

- Real processed P1 with orange PBR clearcoat; preserved F1 livery/carbon/rim/tire distinctions.
  Shared GLB data is cached; owned material clones and debugging geometry are disposed on switching.
- Road-frame quaternion and recorded body pose; per-wheel road-height contacts, steering and
  deterministic integrated spin. Stable engineering/orbit views and an analytic critically damped
  chase-camera spring, with explicit seek reset.
- Locally authored asphalt/grass/gravel color, roughness and normal textures; no runtime hotlinks.
- Bounded terrain height field interpolated from the existing elevated centerline. Track corridor
  cells are removed and verges cover the join. This replaces long folded terrain strips.
- Red/yellow curbs appear only in explicit Spa render metadata. Placement is illustrative, not
  surveyed. Forest uses seeded pine/broadleaf variants in instanced groups with road exclusion.
- Barrier meshes are vertical strips with finite/short segment checks and nearest-road exclusion
  across hairpin branches; invalid segments are omitted. Debug centerlines use the **same accepted
  segments**, so they cannot silently reconnect rejected gaps. Fencing uses validated meshes.
- Sun/environment lighting, soft road shadows, tighter shadow bounds and restrained normal bias.
  Pit/service context and start markings remain separate from mathematical circuit geometry.
- The flat-background occlusion found in downhill screenshot review was corrected by aligning
  analytical test-scene ground with the road grade; the road and car are visible on both slopes.

## Measured rendering performance

Production build, Chromium 153 / ANGLE Metal Apple M2 Pro, 1512×1100 at device pixel ratio 1.
Each frame sample contains 300 rendered intervals during playback. These are browser frame
intervals, not isolated GPU execution times.

| View | Mean / p95 frame ms | Draw calls | Rendered triangles | Seek-to-paint ms |
|---|---:|---:|---:|---:|
| Baseline Spa placeholder | 16.667 / 17.500 | not recorded | not recorded | 27.1 |
| Spa P1 chase | 16.664 / 17.300 | 118 | 635,192 | 23.4 |
| Spa P1 debug after switching | 16.669 / 16.900 | 135 | 636,128 | 22.7 |
| Spa F1 chase | 16.664 / 16.900 | 87 | 1,456,088 | 37.1 |

Normal P1 asset-ready elapsed: **3,547 ms**, with session load **2,456 ms** overlapping that path.
F1 switch-to-ready: **323 ms** after the P1 session is loaded; cached P1 switch-back **7.7 ms**.
These are different cache/setup conditions and should not be interpreted as an asset decoder race.
Initial startup includes terrain/environment preparation; its isolated cost was not measured.
Vehicle decoded texture estimates: P1 **66.61 MiB**, F1 **37.33 MiB**. Full scene texture counts
include retained cache entries. Draw calls/triangles include the environment and active overlays;
they are not just the GLB triangle count. Both real meshes retain approximately 60 Hz rendering
in these measurements without silhouette decimation.

Raw measurements: [baseline](milestone-5_5-baseline-performance.json),
[P1](milestone-5_5-performance.json), [P1 debug](milestone-5_5-p1-debug-performance.json),
[F1](milestone-5_5-f1-performance.json). Re-run the browser suite or the application's performance
panel to measure another camera, hardware or browser; no long-duration thermal stress claim is made.

## Tests and reproduction

| Check | Result |
|---|---|
| Release CTest | 10 / 10 passed |
| UBSan CTest | 10 / 10 passed |
| Existing M1–4 independent Python validations | passed |
| Existing replay export validation | passed; original 50.710 s reference retained |
| New body/static/grade/energy/convergence validation | passed |
| Seven scene analytical/sign checks | passed |
| Asset dependency/connectivity/hierarchy tests | 4 / 4 passed |
| Frontend unit tests | 47 / 47 passed |
| Production TypeScript/Vite build | passed; existing large bundle warning remains |
| Chromium browser suite | 19 / 19 passed |

The baseline was measured before implementation and the plan written first. Current checks cover
manifest/side mapping, missing dependencies, contact plane, slope quaternion, body interpolation,
patch velocity/steering, deterministic spin/seek/pause, spring time subdivision, terrain/barrier/
forest constraints, visual switching, screenshots, and legacy replay/analysis interactions.

See [README reproduction commands](../../README.md#milestone-55-reproduction) and
[asset processing commands](../../assets/cars/README.md). From repository root after building:

```sh
.cache/car-venv/bin/python -m unittest discover -s tools/assets -p test_process_car_asset.py
.cache/car-venv/bin/python tools/python/validation/run_milestone5_5_validation.py
python3 tools/python/replay/export_chassis_scenes.py
ctest --test-dir build --output-on-failure
ctest --test-dir build-ubsan --output-on-failure
npm --prefix apps/dashboard test
npm --prefix apps/dashboard run build
```

`run_milestone5_5_validation.py` writes `data/generated/milestone-5_5/validation.json` and the
compressed body experiment CSV. `export_chassis_scenes.py` regenerates scenes and independently
checks their analytical end states, writing `scene-validation.json`. The README also provides all
legacy Python validation commands and the full Spa generation command.

Start `npm run preview` in `apps/dashboard`, then run
`PLAYWRIGHT_BASE_URL=http://127.0.0.1:4173 npm run test:e2e` from that directory. Browser screenshots
are deterministic paused captures; camera and live performance timing remain hardware dependent.
The in-app Browser service returned no available browsers, so the repository's Playwright Chromium
suite performed the render checks.

## Screenshots

The visual reference was inspected before interface changes. These captures show larger actual
cars, elevated roads, restrained engineering overlays, and richer forest/terrain. This remains a
stylized engineering environment rather than a photorealistic replica of the target.

- [Spa / P1 chase](../screenshots/milestone-5_5-spa-p1.png)
- [Spa / F1](../screenshots/milestone-5_5-spa-f1.png)
- [Spa grounding and axes](../screenshots/milestone-5_5-grounding-debug.png)
- [Flat straight](../screenshots/milestone-5_5-flat.png)
- [10° uphill](../screenshots/milestone-5_5-uphill.png)
- [10° downhill](../screenshots/milestone-5_5-downhill.png)
- [Left turn](../screenshots/milestone-5_5-left.png)
- [Right turn](../screenshots/milestone-5_5-right.png)
- [Braking zone](../screenshots/milestone-5_5-braking.png)
- [Accelerating exit](../screenshots/milestone-5_5-acceleration.png)

## Remaining limitations

The passive small-angle body has no unsprung DOFs, suspension geometry, hydraulics, banking,
curvature-induced normal acceleration, full reference-frame angular transport, jumps or curb
impacts. It is not a complete 3D handling model. Tire angular dynamics remain absent. F1 brake
artwork is incomplete; its visual scale is assumed. Physics and mesh track widths differ slightly,
and the F1 artwork deliberately does not change P1 force parameters.

Spa's 30 m DSM-derived elevation is not survey-grade paving and can contain excessive local grade.
Off-road terrain is interpolated from the profile, not a separately sampled DEM field. Forest,
barriers, runoff and curb sections are illustrative, not a surveyed venue reconstruction. No claim
of photorealism or complete absence of every possible off-track camera artifact is made.

Asset-ready measurements include setup/session overlap and warm-cache effects; they are not an
isolated download benchmark. Texture memory is estimated from decoded image dimensions/mipmaps,
not measured whole-process GPU allocation. Rendering smoothness was measured on this M2 Pro in
headless Chromium, not guaranteed on every display/browser. No new ABS, TC, thermal tires,
differential or multiplayer work was added. Existing optimizer results remain planar-only and
must not be treated as validated sprung-body solutions.

The next milestone should reconcile the existing optimization work with the explicitly chosen
physics model, refine elevation quality, and require independent production replay before making
any new minimum-time claim.
