# Milestone 6.1 — interactive stability and semantics audit

This pass stabilizes the dynamic-contact workspace at `/?drive`. It adds no tire friction, vehicle stabilization, new optimization objective or free-path solver. The preserved planar replay/collocation workspace remains separate. See the [architecture and 6.2 boundary](../architecture/milestone-6_1-stability.md).

## Baseline and replay root cause

Before changing behavior: Release CTest **17/17**, frontend **52/52**, Playwright **23/23**, and the four primary independent feedback replay audits passed. Those browser tests did not cover the reported forward-play/end/orbit cases. Actual DRIVE and policy-play frame measurements were saved in [baseline rendering](../../data/generated/dynamics/m61-before-render.json), rather than inferred from the old static screenshot.

The DRIVE replay animation callback called `setReplayTime(t => t + (now - last)/1000)` and then immediately assigned `last = now`. React can evaluate that updater later. The callback therefore reads a different `last` than the frame that scheduled it: zero elapsed time, or a negative interval if another frame has run. This explains stalled play and interaction-dependent reverse motion without any intentional negative playback rate. Seeking changes React's scheduling/queue circumstances. The loop also lacked a proper end transition and Play-at-end restart.

`DriveClock` now owns the session, authoritative `simulationTime`, paused/playing state, explicit forward direction and positive rate. Time is derived from a wall-time anchor; it is not accumulated by a React updater. Play continues from a pause/seek, or restarts at the first recorded timed-lap sample at the end. Seek pauses. Step selects the next actual recorded timestamp, including from between samples, and stays paused. This is one published sample, not a hard-coded 20 ms increment. The server currently transmits every second 20 ms recording row, so most steps are 40 ms. Floating-point endpoint differences are tolerated when restarting. Timeline dragging is continuous.

The frame callback samples the clock before car, wheel, sun, ghost, force and camera callbacks. UI/map/chart/tire/lap readouts observe its latest snapshot at a lower rate. The time/distance ghost uses the same current pose/time/progress; distance-reference mode intentionally samples the other lap at equal distance. Live input commands and configuration mutations are serialized. Reset/start waits for the command response, and key transitions trigger a fresh input publication immediately rather than waiting behind the next polling interval. Pending input changes are coalesced, not queued as obsolete key states.

Full-suite testing exposed a second host-level problem: all open DRIVE pages could send commands to one native session, including neutral polls from an older tab. This can overwrite current keyboard/gamepad requests. Configuration now assigns input ownership to a browser identifier; stale/missing owners receive HTTP 409 before state mutation and the stale UI stops polling. A host regression test verifies that rejected commands cannot even change the running flag. Existing solver processes were preserved; the optimizer responsiveness test uses an isolated temporary host. A concurrent solve also exposed premature publication of a rejected candidate during replay selection, temporarily removing the available ghost. Candidate output is now separate: only an accepted policy is atomically published, with a content-addressed matching reference. A regression test verifies that rejection leaves the previous accepted pair intact.

## Rendering and cameras

Physics remains fixed 500 Hz in the native host. The host publishes at 100 Hz and the browser polls at 25 Hz, plus immediate key transitions. React is never driven by the 500 Hz integrator. Live rendering uses a bounded timestamp buffer with 60 ms playout allowance; missing future data is held, not extrapolated. Rendering interpolates position, velocity, suspension and continuous telemetry from the bracketing samples. Attitude uses quaternion SLERP. Contact/material/gear flags remain discrete; airborne tire forces remain exactly zero. Pose is a function of time, never a recursive lerp toward the last received pose.

Chase previously consumed a React-delivered 25 Hz pose and then smoothed camera world position toward it. Moving `Canvas.camera.position` props could also interfere with camera control. The new camera initialization is stable. Each frame transports the camera with current vehicle translation and applies an exact critically damped spring to its desired relative position. Tight/Standard/Cinematic frequencies are 18/11/5 s⁻¹. Standard preserves engineering visibility. A unit test verifies the spring's fixed-target response across timestep subdivisions.

Orbit previously moved only `OrbitControls.target`, leaving the camera behind in world space. It now translates camera and pivot equally, preserves user azimuth/elevation/radius, and saves the offset across Chase → Orbit transitions. Controls remain mounted and own orbital input. Automated tests exercise stationary dragging, moving playback, pause, seek and repeated camera switches.

Track geometry, map paths, policy path and chart series are memoized. Force arrows reuse geometry instead of allocating/discarding helpers on every UI update. Rendering consumes stable mutable frame objects; UI refreshes are throttled. Replay-file disk I/O, compression and HTTP transmission no longer hold the simulation lock. No GLB reduction, texture downgrade, tree removal or shadow-quality reduction was used.

| Mode / measurement | Mean frame ms | p95 frame ms | Draw calls | Triangles |
|---|---:|---:|---:|---:|
| Historical M6 comparison report | 20.59 | 35.10 | 296 | 831,530 |
| Baseline DRIVE, R3F callback intervals | 17.74 | 25.50 | 111 | 375,692 |
| Baseline Play-requested policy view, R3F callback intervals | 16.70 | 26.70 | 114 | 376,226 |
| Final DRIVE, callback wall intervals | 16.65 | 24.10 | 108 | 372,472 |
| Final forward policy replay, callback wall intervals | 16.67 | 18.40 | 122 | 375,708 |
| Final moving comparison + tire forces, callback wall intervals | 16.66 | 17.80 | 298 | 829,268 |

The final heavy comparison p95 is 49.3% below the historical 35.1 ms figure with comparable draw/triangle counts and unchanged visual settings. This is not a controlled hardware-wide speedup claim: the historical view was different in time/motion, and concurrent host workloads varied. The baseline R3F intervals include a rolling loading/timing window; the final callback measurements use an eight-second window after two seconds of moving warmup. Baseline compositor-supplied RAF timestamps (also retained in the JSON) understated callback jitter, so they are not used as CPU frame-interval evidence here. The old replay clock could stall; its baseline is labeled Play-requested rather than asserting validated forward motion. Final recorded clocks verify actual advancing live/replay poses.

Final CPU breakdown, **mean / p95 ms per invocation or rendered frame**, in a common trailing eight-second window:

| CPU scope | DRIVE | Policy replay | Comparison + forces |
|---|---:|---:|---:|
| Telemetry selectors | 0.000 / 0.000 | 0.022 / 0.100 | 0.020 / 0.100 |
| Chart geometry rebuild | 0.007 / 0.100 | 0.000 / 0.000 | 0.000 / 0.000 |
| React reconciliation | 0.876 / 1.100 | 1.186 / 1.600 | 1.086 / 1.300 |
| Live/replay interpolation | 0.041 / 0.100 | 0.042 / 0.100 | 0.042 / 0.100 |
| Camera | 0.007 / 0.100 | 0.007 / 0.100 | 0.007 / 0.100 |
| Vehicle pose/wheels | 0.006 / 0.100 | 0.008 / 0.100 | 0.006 / 0.100 |
| Ghost interpolation | 0.035 / 0.100 | 0.000 / 0.000 | 0.031 / 0.100 |
| Ghost pose/wheels | 0.002 / 0.000 | 0.000 / 0.000 | 0.003 / 0.000 |
| Tire-force arrow transforms | 0.000 / 0.000 | 0.000 / 0.000 | 0.007 / 0.100 |
| Other debug-vector transforms | 0.004 / 0.000 | 0.005 / 0.100 | 0.005 / 0.000 |
| Three.js total CPU render | 1.439 / 3.000 | 0.951 / 1.300 | 2.895 / 3.500 |
| Environment draw submission (nested) | 0.120 / 0.300 | 0.176 / 0.300 | 0.137 / 0.300 |
| Ghost draw submission (nested) | 0.575 / 2.000 | 0.000 / 0.000 | 1.955 / 2.400 |
| Tire-force draw submission (nested) | 0.000 / 0.000 | 0.000 / 0.000 | 0.033 / 0.100 |
| Environment React (nested) | 0.102 / 0.300 | 0.105 / 0.300 | 0.097 / 0.300 |

Native simulation: **0.658 ms mean / 0.836 ms p95 per five-step, 10 ms simulation batch**, reported wall lag 0.000 ms. There is no WASM simulation in this workspace. CPU scopes were added during stabilization; a matching per-category pre-change CPU breakdown is unavailable and is not invented. [Final raw frame/CPU profile](../../data/generated/dynamics/m61-after-render.json).

CPU categories are measured separately in the application: native batches; telemetry selectors; React reconciliation; Three.js rendering; environment; ghost; force arrows; charts; interpolation; camera. Per-category draw timings measure **CPU draw submission**, not asynchronous GPU execution. Submission categories are nested inside Three.js rendering; environment React cost is nested inside React cost, so do not add every row together. Browser timer resolution quantizes very small operations. Final CPU reports use a common trailing eight-second window; draw categories include zero-cost culled frames. GPU time was not independently measured. Measurements are local headless Chromium/Apple M2 Pro, 1672×944, development build; they do not establish every hardware/display configuration's smoothness.

## Keyboard mapping and vehicle response

The old default sent a target of ±0.65 rad (±37.24°) for A/D and immediate 0/1 pedals. It did already have a finite 1.4 rad/s rack: the error was an excessive high-speed keyboard target, **not literal instantaneous physical full lock**. A 200 ms press reaches about 0.28 rad (16°) before release. At 30–50 m/s that is a large tire-slip demand.

A separate device adapter (`apps/sim-server/input_adapter.hpp`) now runs before the unchanged vehicle model:

| Parameter | Default | Meaning |
|---|---:|---|
| Steering command rise / return | 1.5 / 2.5 s⁻¹ | normalized command in [-1,1] |
| Throttle rise / release | 1.5 / 3 s⁻¹ | pedal request |
| Brake rise / release | 3 / 5 s⁻¹ | pedal request |
| Low-speed road-wheel limit | 0.65 rad | same configurable physical range |
| High-speed full-key map | min(limit, atan(5 L / max(1,v²))) | wheelbase L; speed v; 5 m/s² nominal kinematic input demand |
| Rack rate | 1.4 rad/s | existing estimate, UI-configurable 0.1–5 rad/s |

The 5 m/s² parameter is an input-precision estimate, not guaranteed lateral acceleration or a friction/stability controller. Settings and the formula are visible. Keyboard Friendly is default; Raw / Research bypasses shaping. Active analog input retains the physical range. There is no yaw correction, ABS, traction control or friction adjustment. The inspector exposes actual pedal requests, normalized steer command, target/actual road-wheel steering, steering rate, yaw, sideslip, four slips/utilizations and front-minus-rear utilization. The latter is explicitly a balance proxy, not a measured understeer gradient. The displayed steering-wheel equivalent uses an explicitly illustrative 15:1 ratio, not an OEM calibration.

The standardized audit uses flat asphalt, frozen resources, native 2 ms physics, identical longitudinal speed feedback, and either a 200 ms full-key pulse at t=1 s or a held 0.02 rad physical step. It runs both cars at 15/30/50 m/s. These are model fixtures, not instrumented real-car measurements. Raw time histories include request, normalized input, delta/delta-dot, steering-wheel equivalent, speed, yaw, beta and every tire's slip/utilization.

At 30 m/s:

| Vehicle / maneuver | Peak yaw rad/s | Peak beta rad | Peak tire slip rad | Peak utilization | Peak lateral g | First saturation s |
|---|---:|---:|---:|---:|---:|---:|
| P1 old keyboard | 0.8422 | 0.1789 | 0.2633 | 1.0000 | 1.2858 | 1.102 |
| P1 Friendly keyboard | 0.0272 | 0.0024 | 0.0036 | 0.0549 | 0.0456 | none |
| MCL36 old keyboard | 1.0964 | 0.0731 | 0.2256 | 1.0000 | 2.2306 | 1.086 |
| MCL36 Friendly keyboard | 0.0416 | 0.0008 | 0.0032 | 0.0727 | 0.1134 | none |
| P1 held physical step | 0.2770 | 0.0504 | 0.0612 | 0.8500 | 0.8335 | none |
| MCL36 held physical step | 0.1700 | 0.0030 | 0.0170 | 0.2647 | 0.5179 | none |

Final yaw-rate/steering gains for the held physical step are P1 **13.722 s⁻¹**, MCL36 **8.502 s⁻¹** at nominal 30 m/s. At 15 m/s they are 5.972/4.182; at 50 m/s, 12.092/14.437. The 50 m/s P1 step saturates at 1.638 s, so that final ratio must not be treated as a linear steady-state gain. The old 50 m/s keyboard pulse reaches peak beta 0.662 rad for P1, versus 0.0019 rad with shaping. Wheelbase, mass, tire/load distribution and aero differences remain; neither car was tuned toward a common subjective feel. This fixture does not demonstrate that the F1 model must be more difficult in every condition.

The measured sliding mechanism is excessive keyboard steering demand → large front slip/saturation → yaw and rear slip buildup, especially for the high-speed P1. Leaving asphalt adds the pre-existing lower-grip grass surface, not an unexplained global loss of friction. See [P1 traces](../../data/generated/dynamics/m61-p1-input-audit.json) and [MCL36 traces](../../data/generated/dynamics/m61-mcl36-input-audit.json).

### Tire relaxation decision

No relaxation-length state was added. It is physically appropriate to study transient tire response, but it requires load/slip-dependent identification and affects transient stability as well as comfort. The [TU Eindhoven transient tire test/model work](https://pure.tue.nl/ws/portalfiles/portal/296854625/1007369_Flat_Plank_Tire_Tester_Enhancements_for_Transient_Measurements_and_Model_Parametrization.pdf) describes measured transient response and first-order contact-patch relaxation. Neither approximate car currently has validated relaxation data. Adding an arbitrary lag to hide the demonstrated input error would change physics and invalidate existing replay calibration. The physical-step traces above establish a reproducible baseline for future identification; no invented before/after relaxation improvement is claimed.

## Tire-force audit

Arrows represent force exerted **by the road on the vehicle**, at each contact patch. Tire-local components transform through the road-tangent forward/left axes and road normal; body components use inverse attitude, and render coordinates map world [x,y,z] to [x,z,-y]. No sign reversal was needed.

Both cars pass constant-radius left/right tests. Reconstruction error is zero; mirror symmetry passes. P1's summed inward force differs from m v |yaw-rate| by **0.0656%**, MCL36 by **0.2075%**, within the 2% finite-steady-turn audit gate. Every wheel records local Fx/Fy/Fz, steering angle, body force and world force. Rear inward Fy at zero rear steering is expected because rear tires have slip angle. Clicking a force arrow/contact marker opens the tire inspector; all three force frames, slip and steering are displayed together. [P1 audit](../../data/generated/dynamics/m61-p1-force-audit.json), [MCL36 audit](../../data/generated/dynamics/m61-mcl36-force-audit.json).

## Optimizer semantics, evaluation cost and cache

The algorithm is **SciPy PRIMA COBYLA**, derivative-free constrained single shooting. It uses local linear objective/constraint approximations in a trust region; no gradients are supplied. Four periodic Catmull–Rom offset nodes and four speed-scale nodes have fixed start nodes, leaving **six decision variables**. Offsets are bounded ±3 m; speed scales 0.7–1.45 of reference. Physical state/control trajectories are generated by a feedback controller, not independently optimized states/controls. The trajectory stays near the reference because of those bounded/coarse nodes and reference-following feedback; it is not cosmetically bent into an apex line.

The objective is the existing normalized weighted sum of time, path length, negative clearance, utilization balance, platform cost, fuel and wear; no new objective was added. Constraints cover completion, approximate body clearance, time ceiling and mechanical closure plus parameter bounds (16 inequalities for six variables). The physical tire/contact/actuator models enforce their own limits. Independent half-step replay remains a separate acceptance gate.

“2 / 8 / 11 / 31 evaluations” counted expensive objective-function calls, not optimizer iterations or simulation steps. An uncached evaluation simulates the entire candidate lap, unless failure terminates it early. SciPy documents COBYLA's `maxiter` as a function-evaluation limit in its [official interface](https://docs.scipy.org/doc/scipy-1.16.0/reference/optimize.minimize-cobyla.html). The UI now exposes algorithm, variables, function evaluations, native evaluations/cache hits, best feasible objective, improvement, constraint violation, wall time and average evaluation time. There is no invented convergence ETA or separate iteration counter.

One accepted P1 Spa candidate, 223.94 simulated seconds, executes **111,970 steps**:

| Category | Measured seconds |
|---|---:|
| Native model stepping, excluding road projection | 2.9863 |
| Track projection and road-surface queries | 15.4302 |
| Objective accumulation plus diagnostics | 0.0637 |
| Other native setup/controller/bookkeeping | 0.0503 |
| Total native rollout | 18.5304 |
| Request JSON encode | 0.000069 |
| Native return serialization/ABI overhead (difference) | approximately 0.000266 |
| Response JSON decode | 0.000192 |
| Total binding call | 18.5309 |
| Per-candidate IPC | 0 (ctypes in the solver process) |
| Per-candidate frontend computation | 0 |

[Raw profile](../../data/generated/dynamics/m61-evaluation-profile.json). Fine-grained timing has instrumentation overhead. Projection is the dominant measured cost (~83%), not browser rendering. The UI polls compact progress at 1.5 s; it does not execute evaluations. Rendering stays independent of the solver subprocess.

The old code already memoized exact x tuples within one solve so objective and constraint requests shared a rollout. That behavior is retained and counted. Persistent exact caching now additionally hashes full initial state/candidate/timestep, objective/normalization/time budget, all configuration files and the native binary. Changes invalidate results; nearly equal floats are not merged. Cache tests verify repeat, process-lifetime reuse and configuration/objective/candidate invalidation.

| Technical-circuit validation solve | Solver calls | New native candidate rollouts | Cache hits | Mean lookup/evaluation | Solver + validation wall | Accepted lap |
|---|---:|---:|---:|---:|---:|---:|
| Cold | 25 | 25 | 26 | 2.5473 s | 73.137 s | 41.600 s |
| Repeated in a new process | 25 | 0 | 51 | 0.000565 s | 11.273 s | 41.600 s |

The remaining wall time includes mandatory recording and independent validation, which are deliberately not replaced with a cached acceptance decision. Initial reference construction precedes the reported solver timer. Objective, selected policy and replay metrics match exactly. [Cache validation summary](../../data/generated/dynamics/m61-cache-summary.json).

**Candidate worker count remains one.** COBYLA's next candidate depends on previous objective/constraint results; the current SciPy interface does not expose an independent candidate batch. Parallelizing those calls would change the algorithm or require speculative duplicates. No misleading parallel-worker claim is made. Future independent multistart studies can use separate native instances/processes, with deterministic result ordering.

The overlay is now **Policy trajectory**; **Optimized path — Not available: free-path optimizer incomplete** is disabled. Replay labels remain Reference/Optimized Policy. The existing feedback policy is not called Perfect Lap, Optimal Racing Line, Minimum-Time Line, Optimized Lap or an unrestricted optimum. Milestone 6.2's state/control, derivative, collocation-residual, warm-start and independent-validation contract is documented separately; no full free-path rewrite was attempted.

## Verification and reproduction

- Release CTest: **19/19** passed, including new input shaping/reset, exact-cache/publication and host-ownership assertions.
- Frontend: **58/58** tests and production build passed. Build retains its pre-existing bundle-size advisory.
- Browser: **28/28 cases passed across the full-suite run and targeted final reruns** (not one uninterrupted all-green run). Regression coverage includes all start/middle/end/seek/play/pause/step semantics; exact pose/time synchronization; moving/paused/seeked orbit and camera switches; raw/friendly keyboard, analog mock, vehicle reset; solver status and a real solver subprocess while replay/camera controls remain responsive.
- Four baseline policy replays pass with **identical numeric metrics before/after**: [baseline](../../data/generated/dynamics/m61-baseline-replay.json), [same-artifact recheck](../../data/generated/dynamics/m61-final-replay.json).
- All **21 historical study replay checks** pass: worst position 0.406225 m, worst yaw 0.0195214 rad; zero track/control violations or invalid airborne tire force. [Study audit](../../data/generated/dynamics/m61-study-replay.json).
- Separate pre-existing optimizer jobs completed during this work and refreshed saved Spa artifacts (P1 now 224.000 s). They were preserved. All four currently published primary artifacts were therefore audited again and pass: [current artifact audit](../../data/generated/dynamics/m61-current-artifacts-replay.json). Do not confuse the changed optimizer selection with a physics regression.
- Both cars' left/right force audits and all 18 standardized input maneuver cases are recorded; the cache repeat yields identical objective/lap/replay metrics.
- Screenshots were visually inspected. Play/frame-step/reset and chase response are visible controls; debug detail remains collapsed by default.

The final focused run passed moving performance, forward replay, orbit preservation and input-mode checks. Its optimizer publication check initially failed with `OSError: [Errno 28] No space left on device` while writing the separate candidate; the previously accepted policy remained available. Redundant trajectory files from the cache experiment were removed, retaining the compact comparison results. The isolated real-solver browser test then passed in 27.4 seconds, including accepted publication and responsive replay/camera interaction. Earlier input failures caused by competing browser clients and a pause-test observation race were fixed and covered by the targeted reruns.

```sh
cmake --build build -j 6
ctest --test-dir build --output-on-failure
npm --prefix apps/dashboard test
npm --prefix apps/dashboard run build
python3 tools/dynamics/server.py
# Separate terminal, with the server running:
cd apps/dashboard
npm run test:e2e
```

Input/force audits use `build/apps/sim-cli/apexlab-input-audit` and `apexlab-force-audit`, each with either `configs/vehicles/mclaren-p1-dynamic.json` or `mcl36-dynamic.json`. Replay reproduction uses `tools/dynamics/revalidate.py`; profile reproduction passes `profileTiming: true` with a saved native rollout profile. `tools/dynamics/m61_summary.py` collects compact audit results. Full trajectory files are generated artifacts.

Browser connection discovery found no connected browser; verification used the repository's installed Chromium/Playwright suite. No hardware steering wheel or physical gamepad was manually tested. Headless camera/pose tests and measured timings support this stabilization result; they are not a claim of OEM handling fidelity or a universal subjective smoothness rating. The frontend build retains the pre-existing large-chunk advisory.
