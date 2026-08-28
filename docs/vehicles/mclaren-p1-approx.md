# McLaren P1 — approximate physical model

This configuration is an engineering approximation for ApexLab. It is not McLaren proprietary
data, is not endorsed by McLaren, and must never be presented as an official P1 dynamics model.
The dashboard-facing name deliberately includes “approximate model”.

Primary public anchors are McLaren's [P1 specification page](https://cars.mclaren.com/us_en/legacy/mclaren-p1)
(DIN kerb mass 1,490 kg, 916 PS, 900 Nm, 100–0 km/h in 30 m, peak 600 kg downforce) and the
published technical dimensions reproduced in [Car Watch's launch specification](https://car.watch.impress.co.jp/docs/news/601263.html)
(2,670 mm wheelbase, 1,658/1,604 mm track). The 0.34 drag coefficient is a
[published secondary specification](https://www.km77.com/coches/mclaren/ultimate-series/2014/p1/estandar/p1/datos)
and is not claimed as a fixed value across active-aero modes.

## Parameter classifications

| Configuration field | Value | Class | Basis / limitation |
|---|---:|---|---|
| `mass_kg` | 1490 | published | McLaren DIN kerb weight. |
| `air_density_kgpm3` | 1.225 | estimated | Standard sea-level modeling condition, not vehicle data. |
| `drag_coefficient` | 0.34 | published | Public technical specification; active-aero variation is not modeled. |
| `reference_area_m2` | 2.0 | estimated | Effective reference area; no official frontal-area value located. |
| `lift_coefficient_down` | 0.9425 | fitted | Combined with 2.0 m² to reproduce 600 kg at 257 km/h; the real car trims aero above that speed, which this model cannot represent. |
| `maximum_drive_force_n` | 12000 | estimated | Coarse force ceiling; no gear/engine/motor map exists in the current simulator. |
| `driven_wheel_static_load_fraction` | 0.58 | estimated | Approximate rear static weight fraction; not official data. |
| `maximum_brake_force_n` | 19162 | derived | `m v²/(2d)` from 1,490 kg and published 100–0 km/h in 30 m, neglecting aero/rolling load and rounded. |
| `tire_mu_reference` | 1.25 | estimated | Engineering estimate for road-legal performance tires, not Pirelli data. |
| `tire_reference_load_n` | 3653 | derived | `mass × g / 4`. |
| `tire_load_sensitivity_exponent` | -0.08 | estimated | Generic tire-model estimate. |
| `rolling_resistance_coefficient` | 0.012 | estimated | Generic performance-road-tire estimate. |
| `wheelbase_m` | 2.670 | published | Public vehicle dimension. |
| `cg_to_front_axle_m` / `cg_to_rear_axle_m` | 1.5486 / 1.1214 | estimated | From the estimated 42/58 static distribution; constrained to sum exactly to wheelbase. |
| `yaw_moment_of_inertia_kg_m2` | 2400 | estimated | No official value available. |
| front/rear cornering stiffness | 130/150 kN/rad | estimated | Axle-level small-slip values, not tire measurements. |
| front/rear track | 1.658 / 1.604 m | published | Public vehicle dimensions. |
| `cg_height_m` | 0.45 | estimated | No official value available. |
| `front_roll_moment_fraction` | 0.48 | estimated | No official roll stiffness distribution available. |
| `front_brake_bias` | 0.59 | estimated | Static model choice; the real control system is not represented. |
| drivetrain | RWD | published | Production layout. |
| nonlinear solver controls | listed values | fitted | Numerical controls inherited from the validated generic model, not vehicle properties. |

Unknown and intentionally unclaimed values include the detailed tire curves, speed-dependent aero
map, yaw inertia, CG height, roll stiffness distribution, suspension geometry, differential logic,
and brake-control strategy.
