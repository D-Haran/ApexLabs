# Milestone 2 implementation plan

## Existing architecture to preserve

The validated longitudinal path remains intact:

```text
vehicle-v1 JSON -> LongitudinalModel -> generic Euler/RK4 -> telemetry-v1 CSV
```

Milestone 2 adds a parallel planar path rather than replacing that path. Both models use the same
fixed-step integrator templates, vehicle configuration loader, and library/CLI boundary.

## Planned architecture

```text
vehicle-v2 JSON ----> PlanarVehicleModel ----> generic Euler/RK4 ----> telemetry-v2 CSV
                           ^       ^
scenario-v1 JSON -> profiles       +---- LateralTireModel interface
                           |
                           +------------ existing longitudinal force model
```

1. Extend vehicle schema version 2 with wheelbase, CG-to-axle distances, yaw inertia, and
   independent front/rear axle cornering stiffness. Continue loading schema-v1 longitudinal
   vehicles unchanged. Reject inconsistent `a + b != wheelbase` geometry.
2. Introduce a forward-speed planar bicycle state `[x, y, psi, vx, vy, r]`, input
   `[delta, throttle, brake]`, explicit body/world transforms, static axle loads, and a replaceable
   lateral-tire interface with a linear implementation.
3. Reuse `LongitudinalModel` for drive, brake, drag, and rolling force. Apply its net force at the
   CG while the steered front lateral force is resolved into body axes. Keep longitudinal tests
   and APIs independently usable.
4. Add scenario schema version 1 with initial state, duration, timestep, integrator, and constant,
   step, ramp, or sine profiles. The CLI selects planar simulation when `--scenario` is supplied;
   its existing longitudinal flags remain compatible.
5. Add telemetry CSV version 2 for planar state, controls, slip angles, axle forces, body
   accelerations, and the longitudinal force decomposition. Keep the version-1 writer and header
   unchanged so old recordings remain replayable.
6. Add focused C++ tests for geometry, signs, moments, transforms, low-speed finiteness,
   determinism, telemetry, scenario validation, and legacy behavior.
7. Add an independent Python validation series covering zero-steer invariance, analytical steady
   cornering, step steer, handling-balance variants, speed sensitivity, and timestep convergence.
   Generate full-precision CSV/JSON results, engineering SVGs, and a report from actual runs.
8. Extend the microbenchmark with a separately labelled planar RK4 workload, then run Release,
   formatting, static analysis, UBSan, and (where the Apple toolchain permits) ASan checks.

## Numerical and modeling boundaries

The dynamic bicycle tire equations are a forward-motion model. Slip angles use a documented
low-speed denominator regularization and smoothly fade lateral force to zero near rest; this
prevents a singular tire law without claiming accurate parking-speed behavior. Steering remains a
single equivalent front-wheel angle. Loads are static, tire forces are linear and unsaturated, and
longitudinal/lateral combined-slip limits are intentionally deferred.

## Acceptance criteria

- All initial-milestone build, unit, CLI, and analytical validation checks continue to pass.
- The planar C++ implementation and an independent closed-form reference agree in the linear
  steady-state operating envelope with quantified errors.
- The three handling configurations have understeer gradients with mathematically verified
  positive, approximately zero, and negative signs and exhibit the corresponding speed trends.
- RK4 convergence and straight-line invariance are measured rather than inferred.
- Generated documentation states actual results and the model's operating envelope without
  implying real-vehicle fidelity.
