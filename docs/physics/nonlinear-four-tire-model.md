# Nonlinear four-tire planar model

## Model level and coordinates

This schema-v3 model is separate from the preserved linear bicycle model. The body remains the
same planar rigid body with state `[X,Y,psi,vx,vy,r]`; there are no pitch, roll, suspension, or
wheel-speed states. Four contact patches are nevertheless evaluated so quasi-static transfer and
load sensitivity affect individual forces. Body `x` is forward, `y` is left, and `z` is up.

Wheel centers are at `(a,+tf/2)`, `(a,-tf/2)`, `(-b,+tr/2)`, and `(-b,-tr/2)`. Positive steering
and slip produce leftward force. Both front wheels use the same steering angle; Ackermann is
omitted. Contact velocity follows rigid-body kinematics:

```text
vx_i = vx - r yi
vy_i = vy + r xi
[vx_w, vy_w] = R(-delta_i) [vx_i, vy_i]
alpha_i = -atan2(vy_w, vx_w_regularized)
```

The Milestone 2 low-speed regularization and smooth force fade below 1 m/s are retained.

## Fiala/brush-inspired pure-lateral tire

Each configured axle stiffness is divided equally between its two tires. For one tire with
cornering stiffness `C`, normal load `Fz`, and effective friction `mu`, define `q = tan(alpha)` and
`Q = mu Fz`. The adhesion region is

```text
Fy0 = C q - C^2 |q| q / (3 Q) + C^3 q^3 / (27 Q^2)
```

while the sliding region is

```text
Fy0 = sign(q) Q
```

The transition occurs at

```text
|q| = 3 Q / C
alpha_sat = atan(3 mu Fz / C)
```

Thus `dFy/dalpha -> C` at zero slip, the transition is progressive, and force is finite. This is a
steady-state, zero-camber brush approximation; its generic parameters are engineering choices,
not a fitted or proprietary tire data set.

## Load sensitivity and combined force

Effective friction is

```text
mu(Fz) = mu_ref (Fz / Fz_ref)^p,   -0.5 <= p <= 0
```

The reference vehicle uses `p=-0.08`. Increasing load therefore increases capacity `mu Fz` but
decreases `mu`. Zero load produces zero force; negative load is rejected.

The requested force vector is `(Fx_request,Fy0)`. With circular capacity `Q`, requested and
delivered utilization are

```text
u_request = hypot(Fx_request,Fy0) / Q
s = min(1, 1/u_request)
(Fx,Fy) = s (Fx_request,Fy0)
u_delivered = hypot(Fx,Fy) / Q <= 1
```

Radial scaling gives braking and cornering equal priority when their joint request exceeds the
budget. It is deliberately symmetric and does not claim the longitudinal/lateral asymmetry of a
measured tire.

## Normal loads

Ignoring aerodynamic downforce, static loads are

```text
Fzf0 = m g b/L
Fzr0 = m g a/L
```

The signed amount shifted from front to rear by physical body longitudinal acceleration is

```text
dFx_load = m ax h/L
Fzf = Fzf0 - dFx_load
Fzr = Fzr0 + dFx_load
```

so braking (`ax<0`) loads the front. The approximate roll moment is `m ay h`. A configured fraction
`lambda` is assigned to the front axle:

```text
dFzf_lat = lambda m ay h / tf
dFzr_lat = (1-lambda) m ay h / tr

Fz_FL = Fzf/2 - dFzf_lat     Fz_FR = Fzf/2 + dFzf_lat
Fz_RL = Fzr/2 - dFzr_lat     Fz_RR = Fzr/2 + dFzr_lat
```

For positive lateral acceleration (a left turn), right/outside tires gain load. The four loads sum
to `mg` to floating-point tolerance. `lambda` represents roll-stiffness distribution without
introducing roll dynamics. Any negative contact load reports an operating-envelope failure.

## Longitudinal distribution and force transformation

Brake request is split by `front_brake_bias` and equally left/right on each axle. Drive request is
split by `drive_front_fraction` (0 RWD, 1 FWD, strictly between for AWD) and equally left/right.
No differential, wheel rotation, ABS, traction control, or wheelspin state is present.

Each delivered wheel-frame force is rotated to the body frame:

```text
Fx_b = Fx_w cos(delta) - Fy_w sin(delta)
Fy_b = Fx_w sin(delta) + Fy_w cos(delta)
Mz_i = xi Fy_b - yi Fx_b
```

Wheel contributions are summed with aerodynamic drag and rolling resistance at the CG. Body
dynamics are the unchanged Milestone 2 rigid-body equations.

## Coupled force/load iteration

Loads depend on acceleration and forces depend on loads. Every force evaluation starts with
`ax=ay=0`, evaluates all loads and tire forces, computes new body accelerations, and under-relaxes:

```text
a_guess <- a_guess + omega (a_force - a_guess)
```

The reference configuration uses `omega=0.5`, tolerance `1e-6 m/s^2`, and at most 80 iterations.
Both acceleration components must converge. Iterations, final residual, and convergence status are
telemetry fields. Exceeding the limit throws an error instead of returning an arbitrary force.

## Operating envelope and omissions

The model is intended for level-road forward motion with all four tires in contact. It omits tire
relaxation, temperature, pressure, camber, surface detail, suspension geometry and dynamics,
aerodynamic balance migration, wheel angular velocity, differential mechanics, ABS, traction and
stability control. The quasi-static transfer equations become invalid near wheel lift; the model
reports negative loads rather than simulating lift. It is a progressively validated engineering
model, not a high-fidelity F1 or production-car tire simulation.
