# Initial milestone architecture

```text
vehicle JSON -> config loader -> longitudinal model
                                      |
controls -> simulation runner -> Euler/RK4 -> telemetry samples -> CSV
                                      |                            |
                                      +------ tests       Python validation
```

`apexlab_sim_core` has no UI or Python dependency. Its public surface is split into:

- `units.hpp`: explicit scalar quantity types and SI conversion helpers
- `vehicle_parameters.hpp`: validated, versioned parameter data
- `longitudinal_model.hpp`: force decomposition and state derivative
- `integrator.hpp`: generic fixed-step Euler and RK4 algorithms
- `simulation.hpp`: deterministic stepping, stop-event handling, and sampling
- `telemetry.hpp`: schema-v1 sample definition and CSV serialization

The CLI is a process boundary suitable for scripting. It does not change physics behavior. The
Python validation tool treats it as a black box, which tests configuration loading, integration,
force evaluation, and serialization together.

The future real-time server should consume this library rather than move physics into a renderer.
