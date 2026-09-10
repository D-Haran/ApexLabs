# McLaren P1 — approximate calibrated model

The dynamic configuration is `configs/vehicles/mclaren-p1-dynamic.json`. The older planar and passive
sprung-body configurations remain unchanged. This model is not McLaren proprietary simulation data.

[McLaren's P1 specification](https://cars.mclaren.com/us_en/legacy/mclaren-p1) supplies the anchors:
916 PS system power, 900 N·m peak system torque, 350 km/h maximum speed, 2.8/6.8 s to 100/200 km/h,
30/116 m braking from 100/200 km/h, 1395 kg lightest dry mass and 1490 kg DIN mass. McLaren also
states peak downforce equivalent to 600 kg and identifies the seven-speed dual-clutch gearbox.

Every added configuration value has a `parameter_classes` entry. Combined power is derived using
735.49875 W/PS. The ICE/electric split is approximate; gearing, drivetrain efficiency, aerodynamic
areas, brake bias, tire law, inertia and suspension remain engineering estimates unless explicitly
marked calibrated. The inherited 42/58 sprung mass distribution is an approximate anchor, not an
identified suspension or full-car CG model. Unsprung masses shift total mass distribution slightly.

Calibration uses bounded SciPy least squares, calling the native 3D-contact simulation for every
objective evaluation. Unknown effective grip, drivetrain efficiency, brake force and road CdA are
fitted to acceleration and braking anchors. The fit is underidentified as a physical identification
problem; matching these four scalars does not validate cornering or every speed/gear condition.
The independent validation repeats at 1 ms, compared with the 2 ms calibration step. Results and
solver evaluations are retained in `data/generated/dynamics/phase-b-validation.json`.

Drive demand is limited by the smaller of the torque/gear force and power/speed force. Gear shifts
have an estimated torque interruption. Clutch behavior is idealized and wheel rotational dynamics
are absent. A P1 speed governor is explicitly configured from the published maximum; its top-speed
match is not an aerodynamic calibration. The F1 configuration has no such governor.

Active aero interpolates road/race effective CdA and ClA with a 0.2 s response. A braking airbrake
surrogate adds 0.12 m² CdA and 0.2 m² ClA; these constants and the fixed 42% front aero fraction are
**estimated**, not OEM logic. Downforce is capped at the published 600 kg-equivalent anchor. Detailed
aero, tire and hydraulic-suspension maps, exact yaw inertia and true suspension geometry remain
**unknown**. The new contact model's vertical/unsprung parameters remain estimated as documented in
`docs/physics/contact-3d-model.md`.

Run `.cache/optimizer-venv/bin/python tools/dynamics/calibrate.py --fit` after building the native
benchmark. Without `--fit`, the command validates the current configuration without fitting it.
