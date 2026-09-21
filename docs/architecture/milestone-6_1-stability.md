# Milestone 6.1 stability architecture

The dynamic-contact DRIVE workspace uses one `DriveClock`, a forward-only wall-clock-anchored state machine. A session change pauses and resets to its first recorded timed-lap sample. Play at the end restarts there; seek pauses; step selects the next actual timestamp. Positive rate changes re-anchor without discontinuity. There is no React updater integrating elapsed time.

A priority −10 frame callback samples that clock once into stable mutable render objects. Body, wheels, lighting, camera, forces and time/distance ghost consume those objects in the same frame. UI panels read the last published snapshot at 10 Hz. This is a lower-rate observation, not another clock. Physics remains in the 500 Hz native host; HTTP polls are 25 Hz. Live rendering buffers timestamped samples with 60 ms playout delay, interpolates actual brackets and holds the newest sample during starvation. No physics extrapolation and no recursive pose lerp. Continuous telemetry uses linear interpolation; orientation uses quaternion SLERP; discrete contact/gear/material flags hold the left sample. Airborne force stays zero.

The chase camera transports with vehicle translation, then applies an exact critically damped spring to the desired relative position. Tight/Standard/Cinematic frequencies are 18/11/5 s⁻¹. Orientation follows the current interpolated pose. Orbit keeps its offset across mode switches; translation moves camera and target equally. OrbitControls owns user angle/radius. Camera initialization is stable rather than receiving a fresh moving position from React.

`apps/sim-server/input_adapter.hpp` is a device adapter outside all physics models. Friendly keyboard rates: steering rise/return 1.5/2.5 normalized units/s; throttle rise/release 1.5/3; brake rise/release 3/5. Full-key target is min(physical limit, atan(5 L / max(1,v²))). The 5 m/s² nominal kinematic demand is an input-precision estimate, not a friction limit or stability controller. Actual lateral acceleration can differ. Gamepad bypasses shaping and retains the full physical range. Raw keyboard also bypasses shaping. Both use the existing finite 1.4 rad/s rack, configurable 0.1–5 rad/s as an engineering estimate. No yaw feedback, traction control or tire-friction changes.

Policy search remains SciPy PRIMA COBYLA, with periodic lateral-offset and speed-scale nodes, fixed start nodes and native single shooting. The current four-node UI has six decision variables. One objective evaluation integrates a full candidate lap unless failure ends it early. Adaptive trust-region candidates depend on previous results, so they are not safely batched by this SciPy interface: candidate worker count stays one. The solver runs in a separate process.

Exact cache keys hash the full request (including physical initial state and timestep), normalized objective/scales/time budget, every config JSON (including inherited tire/chassis/road files), and native library bytes. In-memory objective/constraint reuse is retained; disk reuse survives process restarts. Hits are reported separately from native evaluations. Failed exceptions are not cached. Accepted output still requires independent replay validation.

## Milestone 6.2 interface boundary

Do not pass a `policy` result into a free-path renderer under an optimized-path label. A future solver result must carry:

- formulation: fixed-path or free-path nonlinear optimal control;
- independent state/control arrays and a time/space grid;
- objective and dimensional normalization;
- bound/path/periodicity residuals and collocation defects;
- derivative method and sparse Jacobian/Hessian metadata;
- solver termination and warm-start identity;
- independent replay acceptance and model/configuration fingerprints.

The present policy result supplies offsets/scales and feedback replay only. It cannot satisfy that interface by renaming its center/reference-following trajectory. The free-path overlay remains explicitly unavailable.

## Timing interpretation

`__apexDriveCPU` exposes CPU samples in a common trailing eight-second window: interpolation, selectors, chart geometry, React reconciliation, camera, force updates, total Three.js rendering, and per-object environment/ghost/force submission. Submission categories are nested inside total rendering; React environment work is nested inside React rendering. Do not sum nested categories. Browser timer quantization can report zero for tiny operations. These are CPU submission measurements, not isolated GPU execution times. Native host statistics measure five-step (10 ms simulated) batches separately. The retained full GLB assets, textures, shadows and scene quality were not reduced.

The native rollout's optional `profileTiming` separates model stepping from road/track projection, objective/diagnostic accumulation and other native work. Python binding timing reports JSON encode/decode and total native-call wall time. The binding is in-process ctypes (zero IPC); the solver process communicates only compact progress/result files with the UI host. One candidate's wall time must not be confused with a frame time.

## Input ownership

The native host owns one driving session. Every browser module has a fresh client identifier; `/configure` explicitly assigns that session to its caller. Commands and optimization starts from replaced clients receive HTTP 409 before mutating any state. The replaced page stops polling controls and displays the ownership message. Read-only telemetry remains accessible. This prevents concurrent tabs' neutral input from fighting the active driver. The solver responsiveness browser test uses its own temporary host to avoid disrupting existing solver jobs. `--resume-job PID --resume-progress PATH` preserves monitoring of an already-running solver during a host-only restart; it verifies the process command before considering it active.

## Accepted-result publication

Optimizer candidates and replay-selection audits are written to a separate candidate artifact. Publication rejects an unaccepted result. The reference recording is content-addressed (excluding wall time), then the accepted policy is atomically replaced with a pointer to that reference artifact. The HTTP reference endpoint resolves that pointer; legacy offline reference filenames are retained. An unsuccessful solve therefore cannot make the last good ghost disappear, and a newly published policy identifies its matching reference.
