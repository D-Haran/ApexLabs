# Milestone 3 implementation plan

## Preserved model hierarchy

Milestone 3 adds a separate schema-v3 nonlinear model. The validated schema-v2
`PlanarVehicleModel`, its linear axle tires, telemetry-v2 writer, and all Milestone 2 scenarios
remain unchanged and independently runnable.

```text
schema-v2 vehicle -> linear bicycle model -> telemetry v2
schema-v3 vehicle -> nonlinear four-tire model -> telemetry v3
                                      ^
                         common state, controls, integrators, scenarios
```

The nonlinear body still has only `[X,Y,psi,vx,vy,r]`. Four contact patches are used to calculate
forces and quasi-static normal loads; pitch, roll, suspension, and wheel rotation are not states.

## Tire and force architecture

`NonlinearTireModel` receives normal load, slip angle, and requested longitudinal force and returns
requested/delivered force, effective friction, utilization, and saturation. Its pure-lateral law
is the adhesion/sliding Fiala brush approximation. A circular combined-force boundary is applied
to the requested longitudinal force and the Fiala lateral force. This interface can later be
replaced by a measured or Magic Formula model without changing rigid-body dynamics.

The four-tire vehicle computes contact-patch velocity from `v_CG + omega cross r`, rotates it into
each wheel frame, evaluates each tire, rotates delivered forces back to the body, and sums the
general moment `rx*Fy - ry*Fx`. Equal front steer is retained.

## Quasi-static load solution

Static axle loads follow CG moment balance. Longitudinal acceleration shifts
`m*ax*h/L` from front to rear. The roll moment `m*ay*h` is split using
`front_roll_moment_fraction`; the load shifted between the two tires on an axle is the assigned
roll moment divided by that axle's track. Positive body lateral acceleration (left turn) unloads
the left tires and loads the right tires.

Forces and loads are algebraically coupled. Each force evaluation starts from zero acceleration
and uses deterministic under-relaxed fixed-point iteration. Iteration stops when both acceleration
components change by less than the configured tolerance, or reports failure at the configured
limit. Negative contact load is a model-envelope error, never silently clamped.

## Configuration and compatibility

Vehicle schema v3 adds track widths, CG height, roll-moment distribution, brake bias, static drive
split, Fiala/load-sensitivity parameters, and solver controls. A generic reference performance car
uses explicitly illustrative engineering choices rather than OEM data. Scenario schema v1 remains
shared. Telemetry v3 contains all v2 body quantities plus per-wheel load, slip, requested and
delivered forces, utilization/saturation, transfer, and iteration diagnostics.

## Verification plan

1. Preserve all Milestone 1/2 tests and black-box validations.
2. Add focused tire, load, kinematics, transform, moment, iteration, failure, telemetry, and
   determinism tests.
3. Implement an independent Python Fiala/load-transfer reference and all ten required experiment
   groups, emitting CSV, JSON, SVG, and a generated Markdown report.
4. Benchmark linear and nonlinear RK4 workloads separately and report nonlinear solver iteration
   statistics.
5. Run Release tests, both legacy validations, Milestone 3 validation, formatting checks, and
   sanitizer builds where supported.

