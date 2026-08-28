# Milestone 4 closed-track architecture

## Preserved boundaries

Milestones 1–3 remain unchanged. The new track/lap layer calls the schema-v3 nonlinear model
through its public derivative/force interfaces; geometry and controllers do not enter tire or
rigid-body code.

```text
track-v1 JSON -> PeriodicTrack -> ReferenceLine -> SpeedProfile
                       |                |               |
                       +------ projection/frame queries+
                                        |
vehicle-v3 JSON -> nonlinear model <- deterministic lap controller
                                        |
                              telemetry-v4 CSV -> spatial resampling/validation
```

## Decisions

1. A closed uniform Catmull–Rom cubic interpolates concise, source-controlled control points. It
   is periodic, local, interpolating, dependency-free, and provides analytical first/second
   derivatives. It is not parameterized by distance directly.
2. A deterministic dense chord-length table maps physical arc length to spline parameter. Linear
   table interpolation is followed by analytical spline evaluation. The table is a runtime cache,
   not track source data.
3. Projection first searches the sampled spatial cache, restricted to a wrapped progress window
   when a prior `s` exists, then minimizes squared distance with Newton refinement. Sequential lap
   simulation always supplies the prior progress to avoid jumping to a nearby hairpin branch.
4. Track centerline and reference line are distinct. Milestone 4 implements a validated constant
   lateral offset; the zero offset is the default. The API can be replaced by a future optimized
   line without changing vehicle physics.
5. A curvature feedforward plus lateral/heading feedback controller is held between independent
   controller updates. The speed controller maps a proportional speed error to mutually exclusive
   throttle or brake. Controllers are simulation tools, not driver models.
6. The speed profile applies a conservative curvature cap and cyclic forward/backward spatial
   acceleration passes. It is intentionally approximate, because load transfer and combined grip
   make exact acceleration limits state-dependent.
7. Lap completion uses unwrapped directed progress, not start-point proximity. Generalized sector
   fractions advance in order. Telemetry records both wrapped and unwrapped progress.
8. Telemetry schema v4 is spatially resampleable. The previous three schemas and writers remain
   available and unchanged.

## Work products and acceptance

- `track.hpp/.cpp`: geometry, arc length, coordinates, projection, boundaries, generated tracks,
  schema loader.
- `lap_simulation.hpp/.cpp`: reference line, speed planner, controllers, timing, telemetry, spatial
  resampling.
- `configs/tracks`: track schema and generated technical circuit.
- CLI lap mode and sampled geometry export.
- focused C++ tests, geometry/full-lap benchmarks, and independent Python experiment series.

The milestone is accepted only when the nonlinear vehicle completes the generated circuit without
its CG leaving the modeled boundaries and all Milestone 1–4 verification remains passing.

## Explicit exclusions

The simulated lap is not a minimum-time solution. There is no line optimization, optimal control,
wheel rotation, differential, ABS/TC, tire temperature, suspension state, full-body boundary
collision, rendering mesh, dashboard, or engine integration. Mathematical geometry and rendering
assets share a future coordinate frame but remain separate representations.
