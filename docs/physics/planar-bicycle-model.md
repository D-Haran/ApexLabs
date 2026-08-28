# Planar dynamic bicycle model

## Coordinates, signs, and state

The inertial frame has world axes `X` and `Y`. The vehicle body frame is attached to the center of
gravity (CG): `x` points forward, `y` points left, and `z` points upward. Yaw `psi`, yaw rate `r`,
front steering angle `delta`, lateral force, and slip angle are positive counter-clockwise/left.
All calculations use SI units and double precision.

The state and control vectors are

```text
q = [X, Y, psi, vx, vy, r]
u = [delta, throttle, brake]
```

where `vx` and `vy` are body-frame CG velocities. A single equivalent front wheel represents both
front tires; Ackermann geometry is not modeled. The front axle is `a` metres ahead of the CG and
the rear axle is `b` metres behind it, with the validated constraint `a + b = L` (wheelbase).

World kinematics are

```text
X_dot   = vx cos(psi) - vy sin(psi)
Y_dot   = vx sin(psi) + vy cos(psi)
psi_dot = r
```

## Axle velocities and slip angles

Rigid-body kinematics give lateral velocities `vy + a r` at the front axle and `vy - b r` at the
rear axle. Slip angle is defined as wheel heading minus the local velocity heading, so a positive
slip produces a positive (leftward) tire force:

```text
alpha_f = delta - atan2(vy + a r, vx_bar)
alpha_r =       - atan2(vy - b r, vx_bar)
```

This convention passes two direct sign checks: a stationary positive steer produces positive
front force and yaw moment, while positive `vy` with zero steer produces negative axle forces that
oppose that motion.

The dynamic bicycle equations are singular as longitudinal speed approaches zero. ApexLab uses
an explicit numerical transition rather than pretending that the model is valid at parking speed:

```text
vx_bar = sign(vx) max(|vx|, 0.5 m/s)
s       = smoothstep(clamp(|vx| / 1.0 m/s, 0, 1))
```

Computed lateral forces are multiplied by `s`. They are finite below 1 m/s and exactly zero at
rest. This is a numerical handoff region, not a low-speed tire model. Validated scenarios use
forward speed; negative initial `vx` is rejected.

## Loads and linear tires

With no load transfer, static axle loads follow moment balance:

```text
Fz_f = m g b / L
Fz_r = m g a / L
```

The replaceable `LateralTireModel` interface receives slip angle and normal load. This milestone's
`LinearTireModel` implements independent axle stiffnesses:

```text
Fy_f = s Cf alpha_f
Fy_r = s Cr alpha_r
```

`Cf` and `Cr` are axle cornering stiffnesses in N/rad, not per-tire values. The linear law has no
saturation; its normal-load argument is reserved for later nonlinear/load-sensitive models.

## Coupled forces and equations of motion

The existing longitudinal model supplies drive, brake, aerodynamic drag, and rolling resistance.
Their net `Fx_long` is applied at the CG. The front lateral force acts in the steered axle frame and
is resolved into body coordinates:

```text
Fx = Fx_long - Fy_f sin(delta)
Fy = Fy_f cos(delta) + Fy_r
Mz = a Fy_f cos(delta) - b Fy_r
```

The body-frame rigid-body equations are

```text
vx_dot = Fx/m + r vy
vy_dot = Fy/m - r vx
r_dot  = Mz/Iz
```

equivalent to `m(vx_dot - r vy) = Fx`, `m(vy_dot + r vx) = Fy`, and `Iz r_dot = Mz`.
Telemetry longitudinal/lateral accelerations are the specific-force components `Fx/m` and `Fy/m`;
the state derivatives also contain the rotating-frame terms shown above.

No combined-slip constraint is imposed in this milestone. Longitudinal traction limiting remains
the preserved aggregate longitudinal approximation, while linear lateral forces are independent.

## Linear steady state and handling balance

For small angles, constant forward speed `V`, curvature `1/R`, and steady yaw rate `r = V/R`,
force and moment balance give

```text
Fy_f = (b/L) m V r
Fy_r = (a/L) m V r
```

Using `Fy = C alpha` and the slip definitions yields

```text
delta = L/R + K ay
K     = (m/L) (b/Cf - a/Cr)
r/delta = V / (L + K V^2)
```

Here `K` has units `s^2/m` (equivalently rad per `m/s^2`, since radians are dimensionless):

- `K > 0`: understeer; steering demand increases with lateral acceleration.
- `K = 0`: neutral steer; the geometric relation is speed-independent.
- `K < 0`: oversteer; yaw-rate gain rises toward the linear critical speed
  `Vcrit = sqrt(-L/K)`.

The independent Python validation implements these closed forms separately from production C++.

## Operating envelope and omissions

The intended envelope is forward motion above the 1 m/s transition, modest steering (scenario
validation limits `|delta| <= 0.35 rad`), modest slip, and conditions where a linear unsaturated
tire is defensible. This milestone does **not** model nonlinear tire saturation, combined slip,
dynamic longitudinal/lateral load transfer, individual wheels, suspension kinematics, camber, toe,
tire temperature or pressure, road banking or elevation, differential behavior, aero-balance
variation, or real F1-specific dynamics. It is a planar research baseline, not a limit-handling
vehicle prediction.
