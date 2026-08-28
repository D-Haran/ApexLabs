# ApexLab project specification

## Active milestone: 5.5

The latest attached request is the physical vehicle integration / 3D chassis / environment bridge.
See [implementation plan](docs/architecture/milestone-5_5-plan.md) and
[road/sprung-body equations](docs/physics/road-sprung-body-model.md).

Preserve M1–5 longitudinal, linear bicycle, nonlinear tire/quasi-static, track/lap and recorded replay
systems. New physics is selectable through an explicit sprung-body vehicle configuration; default
legacy behavior remains independently runnable. C++ is the physics authority. No separate game
physics engine, ABS/TC, thermal tires, differential expansion or multiplayer.

The workspace already contained M6 optimization code and artifacts when this request began. Those
are retained without new solves or optimizer development. Their planar-model results do not validate
the new grade/suspension mode. The active deliverable includes supplied P1/F1 semantic visual rigs,
road frames/grounding, recorded chassis telemetry, deterministic wheel spin, passive body force
feedback, camera dynamics, improved bounded environment and reproducible numerical/visual validation.
