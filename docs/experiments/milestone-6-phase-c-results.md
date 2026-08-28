# Milestone 6 Phase C — UI redesign

Status: **implemented and gated**.

The workspace was rebuilt around the supplied `docs/design/apexlab-target-ui.png` rather than
merely recoloring the Milestone 5 cards. The central 3D view now dominates a compact three-column
engineering workspace, with a session rail at left and circuit/car/aero/G data stacked at right.
The telemetry dock and setup/optimization panel share the lower workspace.

## Changes

- Restrained two-level header with the ApexLab wordmark, DRIVE / ANALYZE / IMPROVE modes, Track /
  Car Setup / Telemetry / Analysis / Compare views, current-circuit selector, file open and settings.
- Dark navy/charcoal palette, thin blue-gray dividers, blue interaction accent and orange identity;
  square, dense controls replace the sparse M5 hierarchy.
- Compact Reference / Optimized / Compare session modes. Unavailable optimized modes are disabled
  rather than populated with fictional results.
- Larger viewport with subtle lap-state, speed, throttle, brake, steering and lateral-acceleration
  overlays. No gear, RPM, temperature, pressure or fabricated weather channels were added.
- Circuit map, top-down car/load schematic, per-tire operating point, configured aero/load and an
  actual `ax` / `ay` G-circle occupy the right rail.
- Speed, throttle, brake and steering are the four default synchronized telemetry traces.
- Read-only Vehicle / Aero / Tires / Brakes / Optimization setup tabs show only recorded model
  values. The reference action is live; fixed/free optimization actions remain visibly disabled
  until Phases D/E provide real solvers.

The aero readout evaluates the same configured `0.5 rho v² C A` relations at the current recorded
speed. It is labelled “configured approx.” for the P1 model. The G panel uses exported longitudinal
and lateral acceleration and makes no claim about vertical acceleration.

## Visual regression

The principal deterministic capture is:

```text
docs/screenshots/milestone-6-spa-p1.png
```

Compared with the M5 capture, the viewport is larger, navigation and session hierarchy match the
reference more closely, the map/status stack is denser, all four primary traces are visible, and a
real setup/optimization area replaces ornamental whitespace. The exact reference fonts/assets and
the authenticated McLaren mesh remain known differences.

## Gate

- C++/Python CTest: 8/8 passing.
- Dashboard unit tests: 31/31 passing.
- TypeScript production build: passing (the existing large-bundle warning remains).
- 1024 px and 1920 px layout checks: passing with no horizontal overflow.
- Complete browser suite: 9/9 passing, including Spa visual smoke with no page or scene errors.
