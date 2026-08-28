# ApexLab telemetry schema

## CSV version 1

The first line is the exact header below. Every subsequent line is one sample. Values are decimal
SI quantities written with 17 significant digits so a binary `double` can round-trip.

```text
schema_version,time_s,step,position_m,velocity_mps,acceleration_mps2,throttle,brake,drive_force_n,brake_force_n,drag_force_n,rolling_resistance_n,downforce_n,traction_limit_n
```

Force fields are signed except `downforce_n` and `traction_limit_n`, which are nonnegative
magnitudes. `step` starts at zero. Time is simulation time and is monotonic; the final braking
sample may occur between nominal fixed-step boundaries because it represents a stop event.

A version-1 CSV is a replay artifact: longitudinal position, velocity, controls, and force state
can be visualized without rerunning physics. Consumers must reject unsupported schema versions
rather than guessing field meaning.

## CSV version 2

Version 2 is emitted only by planar simulations. Version 1 files and their writer remain unchanged.
The exact version-2 header is:

```text
schema_version,time_s,step,position_x_m,position_y_m,yaw_rad,yaw_rate_rad_s,vx_m_s,vy_m_s,speed_m_s,steering_angle_rad,throttle,brake,front_slip_angle_rad,rear_slip_angle_rad,front_lateral_force_n,rear_lateral_force_n,longitudinal_accel_m_s2,lateral_accel_m_s2,yaw_accel_rad_s2,body_longitudinal_force_n,body_lateral_force_n,yaw_moment_nm,front_normal_load_n,rear_normal_load_n,drive_force_n,brake_force_n,drag_force_n,rolling_resistance_n,downforce_n,traction_limit_n
```

World position and yaw are inertial-frame quantities. `vx_m_s` and `vy_m_s`, axle forces, and the
two translational accelerations use the vehicle body frame (`x` forward, `y` left). Axle lateral
forces are positive left in their axle frames. `speed_m_s = hypot(vx, vy)`. The acceleration
fields equal total body force divided by mass; they are not simply `d(vx)/dt` and `d(vy)/dt`, which
also contain rotating-frame terms.

Replay consumers should select a decoder using the `schema_version` column/header contract. They
must not treat version 2 as an append-only version 1 row: planar position/velocity field names and
semantics are deliberately explicit. Both formats use 17 significant digits.

## CSV version 3

Version 3 is emitted only by schema-v3 nonlinear four-tire simulations. Versions 1 and 2 and their
writers are unchanged. It retains the explicit planar state/control/body-force fields and adds:

- `longitudinal_load_transfer_n`, `front_lateral_load_transfer_n`, and
  `rear_lateral_load_transfer_n`; lateral values are the signed load shifted from left to right
  on that axle, while longitudinal is signed front-to-rear;
- `solver_iterations`, `solver_residual_m_s2`, and `solver_converged`;
- for each suffix `fl`, `fr`, `rl`, `rr`: `fz_*_n`, `slip_angle_*_rad`, `requested_fx_*_n`,
  `requested_fy_*_n`, delivered `fx_*_n`/`fy_*_n`, `force_capacity_*_n`, `effective_mu_*`,
  `requested_friction_utilization_*`, `friction_utilization_*`, and `tire_saturated_*`.

Wheel `Fx` and `Fy` are in the local wheel frame. Requested lateral force is the pure-lateral
Fiala result before combined-force scaling. Requested utilization may exceed one; delivered
utilization remains at or below one. Saturation is encoded as `0` or `1`. All quantities are
computed by the physics model; no inferred visualization-only fields are recorded.

## CSV version 4

Version 4 is the closed-track lap stream. It preserves the nonlinear state, controls,
accelerations, and four delivered utilization values while adding spatial synchronization:
`track_s_m`, monotonic `unwrapped_track_s_m`, `track_progress_fraction`, `lateral_error_m`,
`heading_error_rad`, `reference_curvature_1_m`, `target_speed_m_s`, `speed_error_m_s`,
`lap_number`, `lap_elapsed_time_s`, generalized `sector_index`, and `on_track`.

Positive lateral error is left of the reference line and positive heading error is counter-
clockwise from its tangent. `lap_number=0` is the first traversal; callers distinguish settling
and timed laps using their run configuration. Spatial comparison utilities interpolate a chosen
lap onto uniform `track_s_m` samples and never interpolate directly across start/finish.

## Milestone 5 replay sessions

The schema-v4 writer remains unchanged. A session-v1 manifest adds exact lap/sector timing,
configuration metadata, track geometry, spatial samples and a separately versioned wheel CSV.
The companion serializes already-computed forces with exact timestamp/step alignment; it does
not infer forces from the utilization-only v4 columns.

[`schema-v4.ts`](schema-v4.ts) centrally defines TypeScript field names, types and interpolation
categories. The strict loader and real-export tests validate the boundary. See the
[replay/session contract](../../docs/architecture/milestone-5-replay.md) for formats, coordinate
mapping, discrete behavior, seam handling, units and delta-time derivation.
