# Milestone 6 Phase D — fixed-line minimum-time optimization

Status: **implemented; production replay accepted at N=800**.

## Solver and transcription

The offline optimizer uses CasADi 3.8.1 for symbolic automatic differentiation and sparse
transcription, with the bundled IPOPT primal-dual interior-point NLP solver. This avoids placing a
general-purpose nonlinear solver in ApexLab. The dependency is isolated in
`tools/optimization/requirements.txt`; the deterministic C++ simulator has no CasADi dependency.

The periodic centerline is divided into uniform spatial segments. At each node the decisions are:

- squared longitudinal speed;
- elapsed time (with one terminal node);
- signed drive/brake demand;
- steering angle;
- body sideslip;
- longitudinal acceleration.

Trapezoidal defects impose `d(v²)/ds = 2 ax` and `dt/ds = 1/v`. Speed is periodic; elapsed time is
not. Quasi-steady lateral-force and yaw-moment equilibrium determine steering and sideslip. The
four contact patches use the production model's static/longitudinal/lateral load transfer,
load-sensitive friction capacity, nonlinear Fiala-style requested lateral force, drivetrain split,
brake bias, aerodynamic drag and rolling resistance. Each contact has an explicit combined-force
constraint. The fixed-line problem sets lateral offset identically to zero.

This is a differentiable reduced model, not a second production simulator. It replaces transient
planar dynamics with quasi-steady lateral/yaw equilibrium. Grade is carried in output but omitted
from force balance because the independent production planar simulator currently has no grade
force. A `1e-7` cyclic control-smoothness term is numerical regularization.

## Warm start

N=40 through N=400 map speed, steering and drive/brake demand from the existing feasible 50.710 s
reference lap. N=800 initially failed from that raw warm start at the 5,000-iteration limit; the
failed iterate was rejected. Reinitializing N=800 from the converged N=400 trajectory succeeded in
790 iterations. This sensitivity is recorded rather than hidden.

## Grid refinement

| Nodes | Variables | Constraints | Predicted (s) | Production replay (s) | Replay − predicted (s) | IPOPT iterations | Wall (s) |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 40 | 241 | 762 | 40.1701 | 42.2150 | 2.0449 | 383 | 1.894 |
| 100 | 601 | 1,902 | 42.0270 | 42.2100 | 0.1830 | 309 | 1.134 |
| 200 | 1,201 | 3,802 | 42.4995 | 42.7250 | 0.2255 | 471 | 3.214 |
| 400 | 2,401 | 7,602 | 42.8259 | 43.0500 | 0.2241 | 969 | 11.378 |
| 800 | 4,801 | 15,202 | 42.9491 | 43.1750 | 0.2259 | 790 | 20.570 |

N=40 is visibly under-resolved: the production replay reaches 7.09 m centerline error and its lap
time differs by 2.04 s. N=400 and N=800 differ by 0.1232 s (0.29%) in the reduced objective. Full
machine-readable results are in
`data/generated/optimization/technical-fixed/grid-refinement.json`.

## N=800 residuals

- maximum squared-speed defect: `3.837e-13 m²/s²`;
- maximum time defect: `6.266e-15 s`;
- maximum lateral equilibrium residual: `7.105e-15 m/s²`;
- maximum normalized yaw equilibrium residual: `2.741e-16`;
- maximum tire-force constraint violation: `9.996e-9`;
- minimum wheel normal load: `663.81 N`;
- control, track and speed-periodicity violation: zero at reported precision.

An IPOPT success flag alone is not the acceptance criterion.

## Independent production replay

The optimized spatial controls are replayed through `NonlinearPlanarVehicleModel` with RK4 at 5 ms.
The production path/speed tracker combines optimized feed-forward controls with feedback; maximum
feedback corrections are reported so this is not misrepresented as open-loop agreement.

| Metric | N=800 result |
|---|---:|
| Predicted lap | 42.9491 s |
| Replayed lap | 43.1750 s |
| Difference | 0.2259 s / 0.526% |
| Maximum / RMS position error | 0.2718 / 0.0482 m |
| Maximum / RMS speed error | 0.4310 / 0.1479 m/s |
| Maximum / RMS yaw error | 0.2097 / 0.03384 rad |
| Maximum track violation | 0 m |
| Maximum production tire utilization | 1.0000000000000004 |
| Maximum steering correction | 0.2650 rad |
| Maximum signed longitudinal correction | 0.1509 |

The declared replay gate requires completion, less than 1% lap-time discrepancy, less than 0.5 m
maximum position error, less than 1 m/s maximum speed error, less than 0.25 rad maximum and 0.05 rad
RMS yaw error, no track violation, and bounded tire utilization. N=800 passes all checks in
`data/generated/optimization/technical-fixed/validation-n800.json`.

The validated replay improves on the controller-driven 50.710 s reference by 7.535 s (14.86%). It
is described as a **locally optimized fixed-line solution**, not a globally optimal lap.

## Reproduction

```sh
python3 -m venv .cache/optimizer-venv
.cache/optimizer-venv/bin/pip install -r tools/optimization/requirements.txt

.cache/optimizer-venv/bin/python tools/optimization/fixed_line_optimizer.py \
  --vehicle configs/vehicles/generic_nonlinear_performance_car.json \
  --session apps/dashboard/public/demo/baseline \
  --output data/generated/optimization/technical-fixed \
  --nodes 800 \
  --warm-start-trajectory data/generated/optimization/technical-fixed/fixed-line-n400.csv

build/apps/sim-cli/apexlab-sim \
  --vehicle configs/vehicles/generic_nonlinear_performance_car.json \
  --track configs/tracks/technical_test_circuit.json \
  --optimization-profile data/generated/optimization/technical-fixed/fixed-line-n800.csv \
  --replay-diagnostics data/generated/optimization/technical-fixed/replay-n800.json \
  --output data/generated/optimization/technical-fixed/replay-n800.csv \
  --warmup-laps 0 --laps 1 --dt 0.005 --maximum-lap-duration 120
```

