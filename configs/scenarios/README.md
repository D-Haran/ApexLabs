# Deterministic planar scenarios

Scenario schema v1 defines simulation duration/timestep, Euler or RK4 integration, the complete
initial planar state, and steering/throttle/brake profiles. Supported profile types are `constant`,
`step`, `ramp`, and `sine`. Steering values are radians; throttle and brake are fractions in
`[0,1]`; all times are seconds.

The checked-in zero, constant, step, ramp, and sine examples are reusable CLI inputs. Generated
validation variations are written under `data/generated/planar-validation/scenarios` so every
numerical artifact retains its exact input configuration.

The same schema drives both the preserved linear bicycle and schema-v3 nonlinear four-tire model;
the selected vehicle schema chooses the model. Milestone 3 generated scenarios are stored under
`data/generated/milestone-3/scenarios`.
