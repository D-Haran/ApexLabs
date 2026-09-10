# Native path-and-speed policy optimization

The preserved CasADi/IPOPT spatial collocation optimizer remains available for its
validated planar model. The new 3D contact model contains unilateral contact,
gear transitions, fuel/ERS budgets and thermal state. Its additional optimizer uses
SciPy's mature PRIMA COBYLA implementation and **single shooting** through the native
C++ model. No second tire-force approximation or browser physics is used.

Decision variables are periodic Catmull–Rom lateral-offset and speed-scale nodes.
A native steering/speed feedback policy maps these curves to steering, drive and
brake. All body, wheel, fuel, energy and tire states are then integrated at 500 Hz.
The start node is fixed to the warm-start state. The initial guess is a completed
native reference lap with curvature-based speed planning, not an arbitrary zero
state. Candidate path offsets are free; no inside/outside/apex rule is supplied.

This is a **locally improved policy in a restricted spline family**; post-solve rejection may select an earlier feasible candidate, so stationarity of the accepted candidate is not claimed. It is not the
requested unrestricted full-state collocation problem, and should not be presented
as one. A four-node solve in particular cannot resolve individual Spa corner apexes.
Grid refinement adds policy nodes, while timestep refinement tests integration.
The earlier planar solver's direct-collocation results are not relabeled as results
for the new contact model.

Constraints are evaluated on the actual simulated trajectory: complete forward lap,
CG clearance minus configured half-width and 0.25 m margin, maximum lap duration
(default 115% of reference), and mechanical closure. Half-width is an approximate
footprint constraint, not a yaw-dependent full body collision envelope. The tire
law enforces the force circle; contact is unilateral; inputs and energy are bounded.
Mechanical closure tolerances are 0.5 m/s body velocity, 0.03 rad track-relative
heading, 0.05 rad/s body angular rate and 10 mm suspension extension. These finite
closure tolerances do not amount to exact periodic collocation. Fuel, SOC, wear and
temperature evolve across a lap and are not forced to be periodic.

Objective components are divided by their native reference values (with documented
positive floors in `optimize.py`): time, path length, negative minimum clearance,
front-minus-rear normalized utilization squared, body tilt/rate platform cost,
fuel consumption and summed wear. Custom weights are nonnegative and normalized
to sum to one. Minimum fuel and wear remain subject to the time ceiling. Platform
cost includes body tilt and angular rates, not merely steering. Neutral handling
means matching average front/rear tire utilization, not an OEM balance rating.

The optimizer records every candidate, solver termination, wall time, native mass
matrix residual, constraints, resource costs and exact initial physical state.
Acceptance requires solver convergence, a complete feasible lap, mechanical closure
and independent time-domain replay with halved integration timestep. Both recorded
open-loop control replay and recomputed trajectory-feedback replay are evaluated.
Open-loop divergence is retained and disclosed; accepted policy replay must meet
0.5 m maximum position, 0.5 m/s speed and 0.03 rad yaw error, meet mechanical closure, complete the lap and stay inside the
approximate footprint margin. A converged solver alone never makes a valid ghost.

```sh
cmake --build build -j 6
.cache/optimizer-venv/bin/pip install -r tools/dynamics/requirements.txt
.cache/optimizer-venv/bin/python tools/dynamics/prepare_track.py
.cache/optimizer-venv/bin/python tools/dynamics/optimize.py --vehicle mclaren-p1 --track technical_test_circuit
.cache/optimizer-venv/bin/python tools/dynamics/optimize.py --vehicle mcl36 --track spa-francorchamps
.cache/optimizer-venv/bin/python tools/dynamics/study.py
```

Spa dynamics uses a separate 35 m Gaussian-filtered DSM elevation profile. Its XY
control points are identical to the original OSM import. Filtering is an explicit
engineering approximation, not surveyed paving; original and filtered elevation
ranges and error statistics are stored in `spa-francorchamps-dynamic.json`.
