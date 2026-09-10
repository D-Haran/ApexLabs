# Unilateral contact model (Milestone 6 dynamics, Phase A)

This is a new, independently runnable model. `LongitudinalModel`, `PlanarVehicleModel`,
`NonlinearPlanarVehicleModel`, `RoadVehicleModel`, the reference controller and existing optimizer
are unchanged. In particular, an old planar optimized trajectory is **not** validated against this
model merely because it can be drawn with the same car mesh.

## Coordinates and degrees of freedom

World and body axes are right handed: +X forward, +Y left, +Z up. The sprung-body state is world
position, Hamilton quaternion `[w,x,y,z]` mapping body to world, world linear velocity and body
angular velocity. Four prismatic wheel carriers have extension `l_i` and rate `ldot_i`, positive
downward along body Z. These are four real unsprung DOFs, rather than offsets set from road height.

With fixed body mount `a_i`, wheel center `c_i = x + R r_i`, `r_i = a_i - e_z l_i`:

```
v_i = v + R (omega cross r_i - e_z ldot_i)
a_i = J_i nu_dot + b_i
J_i = [ I, -R [r_i]_cross, ..., -R e_z, ... ]
b_i = R (omega cross (omega cross r_i) - 2 omega cross e_z ldot_i)
nu  = [v_world, omega_body, ldot_FL, ldot_FR, ldot_RL, ldot_RR]
M   = diag(m_s I, I_s, 0_4) + sum(m_ui J_i^T J_i)
```

The coupled 10×10 mass matrix is solved for acceleration at every RK4 stage. Gravity and contact
forces on each unsprung mass enter through `J_i^T`. The RHS subtracts `sum(m_ui J_i^T b_i)` and the
sprung body's `omega cross (I_s omega)` term. This retains wheel inertia, Coriolis terms, sprung /
unsprung momentum exchange and gravity in flight. Springs act only in their extension generalized
coordinate; their equal and opposite forces are internal, so do not accelerate the total COM.

The body quaternion evolves by `qdot = 0.5 q ⊗ [0,omega]`. Force transforms use unit orientations;
the integrated quaternion is normalized after each full step. A requested outer step up to 50 ms
is divided into RK4 steps no longer than 2 ms. This is not an implicit stiff-system solver and is
not certified for arbitrary spring/damper values. The included properties are checked at 2, 1 and
0.5 ms. Invalid/nonfinite inputs fail rather than resetting the car.

## Road / tire contact

Each wheel independently queries its world XY. The response has surface point, unit normal,
material, tire-friction multiplier, rolling resistance and roughness properties. For a locally
planar road query, sphere radius `r`, wheel center `c` and road point `p`:

```
delta     = max(0, r - (c - p) dot n)
delta_dot = -v_wheel dot n
Fz        = delta > 0 ? max(0, kt delta + ct delta_dot) : 0
contact   = Fz > 0
```

This is a unilateral Kelvin–Voigt penalty law: clipping removes tensile road force to **zero**,
never to an artificial positive support load. The road cannot pull a tire back. Tires can have
zero force while geometrically compressed during sufficiently fast unloading. Compliance and
damping handle landing; no state or height is teleported to the surface.

Suspension force in the positive extension direction is
`ks(l_rest-l) - c(ldot) ldot`. Compression / rebound damping differs. Soft bump and droop stops add
spring force and one-sided damping beyond their nominal limits. Stops are not hard constraints;
large impacts may exceed nominal travel. The tire itself is a linear vertical compliance surrogate,
not a calibrated impact/sidewall/bottoming model.

Forces use the **existing nonlinear brush tire law** with the actual measured positive normal
load. The reference μ is multiplied by the selected material scale before applying the existing
load-sensitivity relation and combined-force saturation. Wheel forward direction is the steered
body direction projected into the wheel's own road tangent plane. Lateral direction is
`n cross forward`. Contact-carrier velocity includes body rotation from wheel center to patch.
Slip retains low-speed regularization; longitudinal force remains pedal demand limited by the
friction circle. There is no wheel spin/slip-ratio drivetrain model, ABS or traction control.

The full world force is `forward Fx + left Fy + normal Fz`, exerted by the road **on the vehicle**.
Its application point is the contact patch. The wheel-center force Jacobian plus the patch-offset
couple accounts for that application point. The ideal carrier transfers that couple; wheel spin
inertia and gyroscopic moments are not included. Rotational dynamics of driven/braked wheels are a
remaining approximation, so this is not a full multibody suspension/driveline model.

Airborne wheels bypass the tire law and return exactly zero Fx/Fy/Fz, even with pedal demand. All
telemetry remains finite; unavailable force capacity is zero rather than an infinite utilization.
Gravity and inherited simple aero remain active. In Phase A bench runs aero is explicitly disabled
to isolate mechanics. The **total vehicle COM** is ballistic; the sprung-body COM can move relative
to it while the suspension extends. Tests check total COM position and angular momentum in flight.

## Road and rendering contract

`TrackRoadSurface` wraps the existing metric `PeriodicTrack`, with separate projections for all four
wheels. The centerline/elevation is never changed by rendering. Curbs are explicit longitudinal
spans, side ±1, width 0.9 m and height 0.045 m by default. Their transverse sin² profile and cubic
end ramps join the asphalt continuously. These are estimated **synthetic** curb properties, not
surveyed Spa data. Wrapped spans must be split explicitly at the lap seam.

Defaults: asphalt friction scale 1.0 / rolling resistance 0.012; curb 0.9 / 0.016; asphalt runoff
0.95 / 0.018; grass 0.45 / 0.06. Grass has 6 mm deterministic smooth roughness with 3 m wavelength,
fading in outside runoff. All are editable C++ parameter values, not measured circuit materials.
Far-off-track elevation extends the closest road's elevation; it is not a full facility DEM query.
Nearest-XY projection does not resolve stacked roads/bridges. Barrier and body collisions are absent.

The validation viewer consumes vertices, normals, body quaternions, wheel centers and world forces
exported by C++. It does not reproduce the equations in TypeScript and does not ground wheels to
the road. Rendering uses `(x,z,-y)` and quaternion `(qx,qz,-qy,qw)` for Three.js coordinates. Tire
meshes are rescaled to the configured contact radii; body placement uses the preserved physical
axle midpoint. Visual wheel rotation remains kinematic and is labeled as such.

The sampled render mesh is checked against native road queries at every triangle centroid, with
a 1 mm vertical tolerance for these benches. This is a sampling error check, not proof of exact
agreement everywhere inside every triangle. The legacy Spa renderer still belongs to the legacy
sprung-body replay and its decorative curbs are not silently given physical collision meaning.

## Parameter provenance and interpretation

`estimated_contact_parameters` derives a mechanical fixture from the existing approximate sprung
P1 configuration. It is **not** the calibrated P1 requested in Phase B. Every newly introduced
vertical parameter is estimated: 40 kg unsprung mass per corner, 0.34 m wheel radius, 240 kN/m
vertical tire stiffness, 1 kN·s/m tire damping, 250 kN/m stop stiffness and 3.5 kN·s/m stop damping.
Spring/damper and inertia estimates are inherited from the old approximate configuration.

Sprung mass is derived as total mass minus four unsprung masses. Rest extensions are derived from
flat static balance; bump/droop limits are estimated at −80/+120 mm from that position. Static
spring loads use the *sprung-body* CG axle distances. Adding wheel masses moves the total COM
slightly relative to that point, so the old planar per-wheel weight distribution is not silently
assumed identical. No proprietary suspension geometry, camber kinematics or aero map is claimed.
`flat_equilibrium` is an initializer for this estimated fixture, not a general trim solver for
arbitrary changed suspension geometry.

## Method references

The generalized-coordinate mass-matrix formulation follows standard multibody mechanics; compare
[MuJoCo's equations-of-motion documentation](https://mujoco.readthedocs.io/en/latest/computation/).
The independent height/normal/material query and vertical spring/damper force-element structure are
also used in [Project Chrono terrain models](https://api.projectchrono.org/vehicle_terrain.html) and
its [force-element tire interface](https://api.chrono.projectchrono.org/classchrono_1_1vehicle_1_1_ch_t_measy_tire.html).
These references motivate the model structure, not its numerical calibration. ApexLab does not
embed either physics engine or claim their validation for this implementation.
