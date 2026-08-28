# Road-constrained sprung-body model (Milestone 5.5)

## Authority and preservation

`RoadVehicleModel` is an optional C++ model selected by `normal_load_model: "sprung_body"` in the
vehicle configuration. Missing/default `quasi_static` retains the previous M1–4 model and its
regression results. `mclaren-p1-sprung-approx.json` is separate from the existing P1 configuration.
The additive `forces_with_loads` adapter reuses the exact production slip, brush saturation,
load-sensitive friction and combined-force capacity equations. Supplied suspension loads bypass
quasi-static transfer, so it is not counted twice. No second physics engine is used.

## Road coordinates and gravity

World X/Y are horizontal and Z is up. The physics body uses x forward, y left, z up. For horizontal
track heading psi and grade dz/ds_horizontal, q=1/sqrt(1+grade²):

```
P = (X,Y,z)
T = (cos(psi)*q, sin(psi)*q, grade*q)
L = (-sin(psi), cos(psi), 0)
N = T cross L
G_world = (0,0,-9.80665) m/s²
G_road = (G dot T, G dot L, G dot N)
```

Bank remains exactly zero. These frames are orthonormal without Frenet normal flips. The existing
track parameter is **horizontal arc length**, retained for projection/controller compatibility;
road speed is projected through T/L for horizontal world kinematics. On a straight slope alpha,
zero applied tire/resistance forces give dv/dt=-g*sin(alpha), not a speed modifier.

The recorded yaw remains the horizontal heading. For heading difference d, road-plane heading
error e=atan2(q*sin(d),cos(d)); rotate T/L by e to obtain body axes. Gravity is projected again into
those axes. The road-normal yaw rate maps to horizontal yaw derivative by
`yaw_dot=r*q/(cos²(e)*q²+sin²(e))`. This is a local-road approximation: reference-frame transport
from changing grade and full 3D angular momentum coupling are not represented.

## Passive body balance

All configured vehicle mass is lumped into the sprung body; unsprung mass/wheel vertical DOFs
are omitted. Wheels are constrained to the mathematical road. There is no jump/contact solver.
The body is a small-angle 3-DOF system relative to the local road frame:

```
q_body = (h, phi, theta), qdot_body = (hdot, phidot, thetadot)
```

h is upward heave; phi is right-handed roll about forward x (left side rises); theta is nose-up
(rotation about minus physical y). Contact coordinates are (a,+tf/2), (a,-tf/2), (-b,+tr/2),
(-b,-tr/2). Per corner:

```
d_i = h + y_i*phi + x_i*theta
compression_i = -d_i
compression_rate_i = -hdot - y_i*phidot - x_i*thetadot
F_i = F_i0 + k_i*compression_i + c_i*compression_rate_i
```

`F_i0` is the **constant level-road static preload**, mg*b/(2L) front and mg*a/(2L) rear. It is
not reset every time grade changes. Thus the body settles to the new road-normal support under
m*g*cos(alpha). c_i selects compression or rebound damping by compression-rate sign. Nominal
ride height bounds the stated small-travel envelope; no bump stops or suspension kinematics are
invented. Negative contact force, nonfinite states or compression beyond nominal ride height
raise an operating-envelope error rather than silently clipping forces.

```
m*hdd = sum(F_i) - m*g_normal - D
Ix*phidd = sum(y_i*F_i) + h_CG*sum(Fy_tire_i)
Iy*thetadd = sum(x_i*F_i) + h_CG*sum(Fx_tire_i)
```

Tire forces are resolved into the body frame. The horizontal moments are due to tire forces
acting below CG, not throttle/brake/steering animation curves. Braking loads the front and lowers
the nose; accelerating raises it; a left turn lowers the right/outside side. Drag/downforce act
at CG; no unsupported aero balance is introduced. Downforce loads the springs and therefore feeds
back into tire capacity through F_i. Rolling resistance uses current total support. Gravity is
added to longitudinal/lateral vehicle acceleration but, acting through CG, creates no direct
body moment. Tire moments supply the slope holding/braking pitch response when applicable.

RK4 integrates vehicle and body states together, evaluating suspension and production tires at
each substage. Lap mode retains the 20 ms held path controller and 5 ms physics period. There is
no body animation or dynamics integration in the renderer.

## Approximate parameter values

Every added suspension/inertia parameter is **estimated**, not McLaren hydraulic suspension data:

| Parameter | FL/FR | RL/RR |
|---|---:|---:|
| Spring rate | 65,000 N/m | 75,000 N/m |
| Compression damping | 3,500 Ns/m | 3,500 Ns/m |
| Rebound damping | 4,500 Ns/m | 4,500 Ns/m |
| Nominal travel/ride-height envelope | 0.12 m | 0.12 m |

Roll inertia 650 kg m² and pitch inertia 2,200 kg m² are estimated. Mass 1,490 kg and existing P1
anchors/estimates retain their previous provenance. This passive model is deliberately not a claim
to reproduce P1's real interconnected hydraulic suspension. Mesh-derived tire radii/scale do not
change force parameters. No F1 physical model is inferred from the F1 artwork.

## Replay and display

Schema-v4 telemetry and the original wheel companion remain compatible. Optional `chassis.csv`
v1 has an exact timestamp/step join and records heave/roll/pitch, their rates, all four suspension
compressions, and projected gravity. Missing/misaligned chassis data is rejected for a sprung-body
session. Old planar sessions have no invented chassis values. Interpolation blends recorded
chassis channels; the frontend never reintegrates them.

Renderer mapping is `(X,Y,Z)->(X,Z,-Y)`. Base vehicle quaternion comes from road T/N/right and
horizontal heading. Body transform is road/yaw base × pitch/roll about the physical CG, with heave
along road normal. The normalized visual axle midpoint is placed relative to the physics CG.
Wheel steer/spin groups remain outside the body pivot and are adjusted to the same piecewise
linear road surface used by the mesh. This is road-constrained contact, not suspension-linkage
simulation. Mesh and physical contact coordinates may differ; the debug overlay distinguishes
the mesh wheels from the physical force application locations.

Visual spin is `-integral(v_patch_long dt)/radius` about renderer +Z (right). Using physical
contact coordinates, `v_patch_long=(vx-r*y)*cos(delta)+(vy+r*x)*sin(delta)`; rear delta is zero,
front delta is the actual equivalent steering angle for both wheels (no Ackermann model).
Thus inside/outside wheels use distinct rigid-body patch velocities. The prefix integral is built
from recorded samples and interpolated at the replay cursor, making seek, pause and rate changes
deterministic. It is not a physical wheel-speed/slip state. Calipers steer without spinning.

## Limits

No wheel rotational dynamics, unsprung DOFs, suspension geometry, active hydraulics, roll centers,
anti-dive/squat, gyro terms, banking, road-curvature normal acceleration, jump/contact transitions,
curb impacts, ABS/TC, thermal tires, detailed differential or collision response. Road curvature
and fast reference-frame rotation do not excite heave; this limits high-speed crest predictions.
The 30 m DSM-derived Spa profile remains terrain-derived and can have unrealistic local grade;
it is not survey-grade paving. Off-road terrain is an interpolated visual field from that profile,
not independently sampled surface elevation. Existing optimization results still use the preserved
planar model and must not be treated as validated sprung-body solutions.
