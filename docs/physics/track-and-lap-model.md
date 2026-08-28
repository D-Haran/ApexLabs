# Track geometry and feasible reference lap

## Coordinates and spline

World `X/Y` and vehicle signs are unchanged: body `x` is forward, body/world `y` is left, and
positive yaw is counter-clockwise. Control points are ordered in the forward direction. A periodic
uniform Catmull–Rom cubic evaluates each segment from four neighboring points. At the seam the
same wrapped neighbors are used, producing continuous position and first derivative.

The raw spline parameter `u` is not treated as distance. A dense deterministic table accumulates
chord lengths and provides the monotonic map `u(s)`, where `s` wraps into `[0,L)`. Queries evaluate
the cubic and derivatives at the interpolated parameter. Track length is reproducible for fixed
control geometry and cache resolution.

For derivatives `p'=(x',y')` and `p''=(x'',y'')`, tangent, left normal, heading, and signed
curvature are

```text
t = p' / |p'|                 n_left = (-t_y, t_x)
psi = atan2(t_y, t_x)
kappa = (x' y'' - y' x'') / |p'|^3
```

Positive curvature turns left. Elevation and bank API values are zero in this milestone.

## Track-relative coordinates and boundaries

The local representation is `(s,e_y,e_psi)`. Positive `e_y` is left of the directed centerline;
`e_psi = wrap(psi_vehicle-psi_reference)` is positive counter-clockwise. Conversion to world is

```text
p_world = p(s) + e_y n_left(s)
```

Projection minimizes `f(u)=1/2 |p(u)-q|^2`. A cache search produces a candidate; Newton iterations
use

```text
u_next = u - ((p-q) dot p') / (p' dot p' + (p-q) dot p'')
```

with bounded steps. Supplying previous progress restricts the coarse search to a wrapped local
window. This temporal coherence is important where Euclidean-near hairpin branches are far apart
in progress. At points outside the tubular neighborhood or exactly at geometrically ambiguous
medial axes, a unique inverse is not mathematically guaranteed.

The point-CG boundary rule is

```text
-right_width(s) <= e_y <= left_width(s)
```

It does not test the vehicle footprint.

## Reference line and controller

A constant left-positive offset `d` forms the first replaceable reference line. Its parallel-curve
curvature is `kappa_ref = kappa/(1-kappa d)`; offsets reaching the singularity are rejected.

The path controller is

```text
delta_ff = atan(L_wheelbase kappa_ref)
delta = clamp(delta_ff - K_y e_y - K_psi e_psi, +/-delta_max)
```

Thus a vehicle left of the line receives a rightward correction. Feedforward is a kinematic
approximation; feedback handles dynamic-model steady-state error. Default gains are `K_y=0.045
rad/m` and `K_psi=0.85`. They are held at a default 20 ms controller period independently of the
physics timestep.

Speed error is `e_v=v_target-v`. The proportional command `K_v e_v` becomes throttle when
positive and brake when negative, so large simultaneous commands never occur. No integral term,
gear model, or preview control is present.

## Conservative speed profile

At uniform spatial samples the initial cap is

```text
v_lat = sqrt(a_y_limit / max(|kappa|,epsilon))
v_i = safety_factor min(v_global_max, v_lat)
```

Then repeated cyclic passes enforce

```text
v_(i+1) <= sqrt(v_i^2 + 2 a_accel ds)
v_i     <= sqrt(v_(i+1)^2 + 2 a_brake ds)
```

Eight forward/backward passes remove the start/finish discontinuity deterministically. The CLI
sets lateral capability from reference tire friction, uses a 0.72 safety factor, and uses fixed
3.5/7.0 m/s² acceleration/braking bounds. The exact available acceleration also depends on speed,
drag, load transfer, and simultaneous tire demand; the planner therefore remains deliberately
conservative and heuristic rather than globally optimal.

## Progress, laps, sectors, and telemetry

Successive wrapped projections are differenced on the shortest periodic branch and accumulated as
directed unwrapped progress. A lap completes only when forward cumulative progress crosses the next
multiple of `L`; proximity to the finish alone cannot complete a lap. Sector boundaries are an
ordered list of progress fractions, not a hard-coded three-sector type.

Version-4 telemetry records position, state, control, accelerations, tire utilization, wrapped and
unwrapped progress, line errors, curvature, target speed, lap/sector state, and the CG boundary
flag. Spatial comparison selects one lap, sorts within `[0,L)`, and interpolates onto a uniform
distance grid without interpolating across start/finish.

## Limits

Reported times are **simulated feasible reference laps**, not optimal or theoretical fastest laps.
They depend on the chosen reference line, heuristic planner, controller, vehicle model, and point-CG
boundary convention. Spline self-intersection is not forbidden by the schema, but projection is
only well-defined within the intended local neighborhood.
