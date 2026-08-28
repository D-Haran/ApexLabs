# Longitudinal point-mass model

## Coordinates and signs

The road is straight, level, and fixed in an inertial frame. Position `x` increases forward.
Velocity `v = dx/dt` is signed. Positive forces act forward. All internal values use SI units and
double precision.

## Equations

For mass `m`, the state is `y = [x, v]`:

```text
dx/dt = v
dv/dt = (F_drive + F_brake + F_drag + F_roll) / m
```

Aerodynamic magnitudes are:

```text
q = 1/2 rho v^2
|F_drag| = q Cd A
F_downforce = q Cl A
```

Drag acts opposite velocity. Rolling resistance has magnitude `Crr m g` and also opposes velocity.
It is zero at rest; static friction and road grade are not yet modeled.

Available aggregate tire force is:

```text
F_tire,max = mu (m g + F_downforce)
```

Drive force is limited by `driven_wheel_static_load_fraction * F_tire,max`. Brake force is limited
by the full aggregate value. This is a deliberately simple traction approximation: it omits load
transfer and assumes the configured driven-wheel fraction remains constant.

Throttle and brake are clamped to `[0, 1]`. Simultaneous commands are permitted and their forces
oppose each other. The simulator locates a forward-velocity zero crossing by bisection and stops
there when braking would otherwise cause an unphysical reversal.

## Assumptions and uncertainty

- Air density and coefficients are constant.
- Drag/downforce coefficients use the same reference area specified in the configuration.
- Drive and brake limits do not vary with speed or temperature.
- Mechanical losses beyond rolling resistance are omitted.
- The generic vehicle values are design assumptions, not identified measurements.

This model is appropriate for integrator, serialization, and elementary analytical validation. It
does not justify claims about a specific road car or racing vehicle.
