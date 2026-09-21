# Milestone 6 — dynamics brief and phase gates

The [latest attached request](milestone-6-dynamics-request.txt) supersedes the older IDE-selected
M6 asset/optimization brief. Earlier M6 reports describe earlier work and are preserved as history;
they do not validate the new 3D model or imply completion of this expanded milestone.

| Phase | Required gate | Status |
|---|---|---|
| A: unilateral 3D contact | Static balance, ballistic COM, momentum, compliant landing, independent surface queries, crest speed sweep, timestep convergence, replay alignment | Implemented; see dynamics Phase A report |
| B: P1 / MCL36 models | Published-anchor calibration residuals, distinct aero/powertrain models and parameter provenance | Passed: 13 native tests; phase-b-validation.json |
| C: tire/fuel/energy state | Conservation/bounds, power/fuel/ERS limits, thermal/wear coupling | Passed: 14 native tests; resources.md |
| D: interactive driving | Same native core, ≥200 Hz, keyboard/gamepad, reset/off-track, ghost/time delta | Implemented: 500 Hz native host, keyboard/Gamepad API, accepted ghosts; hardware gamepad not manually tested |
| E: multi-objective free-path optimization | Physical/body-envelope constraints, all objectives normalized, independent replay, sensitivity | Restricted policy optimizer and 21-case study pass feedback replay; full-state/control OCP and robust open-loop Spa replay remain incomplete |
| F: force audit | Mirrored steady turns, world contact-patch transforms and force-frame inspection | Passed: mirrored steady circles, world reconstruction, real rear lateral force and UI frame selection |
| G: UI/environment fidelity | Target comparison, real vehicle-dependent controls and modeled telemetry, performance | New target-layout DRIVE view, actual car assets, local environment, screenshots; measured comparison rendering below 60 fps |
| H: final studies/report | Vehicle comparison, Spa contact/optimized laps, objective and manual-mode verification | Reports, 17 native tests, 52 frontend tests, 23 browser tests; unrestricted OCP/convergence criteria explicitly outstanding |

Each phase must pass before the next is accepted. The Phase A contact lab is a validation tool,
not a replacement for the default Spa session or a claim that interactive driving is implemented.
No old solver result is relabeled as an optimum for the new physics. No tire temperature, wear,
fuel/SOC, gear or pressure has been invented for the validation viewer.

[Final report and reproduction commands](../experiments/milestone-6-dynamics-results.md).
Phases D–H have concrete implementation and validation artifacts, but the expanded milestone is
not fully complete while the Phase E scope/convergence requirements remain outstanding. The
old planar fixed/free-line collocation solver remains preserved and is not a substitute for that work.
