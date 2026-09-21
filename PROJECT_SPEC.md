# ApexLab project specification

## Active milestone: 6 — dynamic vehicle physics

The latest attached request is preserved in
[the M6 dynamics brief](docs/architecture/milestone-6-dynamics-request.txt).
Follow its [gated implementation plan](docs/architecture/milestone-6-dynamics-plan.md).
It supersedes the older asset/optimization brief in the IDE selection.

Preserve M1–5 longitudinal, linear bicycle, nonlinear tire/quasi-static, track/lap and recorded replay
systems, including the independently runnable M5.5 passive sprung-body model. New unilateral
contact physics lives alongside those systems. C++ remains the physics authority. No separate
browser vehicle dynamics, multiplayer or unrequested detailed differential expansion.

Existing planar optimizer code and artifacts are preserved. Their results do not validate the new
3D contact model. The native contact/resource models, P1/MCL36 calibration, interactive driving, force audit,
UI and multi-objective policy studies are implemented; see the
[final report](docs/experiments/milestone-6-dynamics-results.md). The new optimizer remains restricted
to spline path/speed policies; full-state optimal control, robust open-loop Spa replay and a
converged fine-grid solution are outstanding. Do not mark the expanded milestone fully complete
or relabel those policies as unrestricted minimum-time solutions.
