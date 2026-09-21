# Dynamic vehicle calibration and resource gates

The full native suite passes 14/14 cases, including the preserved models. Bench
results are in `data/generated/dynamics/phase-b-validation.json`. Reproduce:

```sh
cmake --build build -j 6
ctest --test-dir build --output-on-failure
.cache/optimizer-venv/bin/python tools/dynamics/calibrate.py
# Explicitly refit parameters only when desired:
.cache/optimizer-venv/bin/python tools/dynamics/calibrate.py --fit
```

| P1 fixed-condition anchor | Published | Native 2 ms | Native 1 ms |
|---|---:|---:|---:|
| 0–100 km/h, s | 2.8 | 2.799695 | 2.798324 |
| 0–200 km/h, s | 6.8 | 6.800065 | 6.796413 |
| 100–0 km/h, m | 30 | 30.024405 | 30.024355 |
| 200–0 km/h, m | 116 | 115.927269 | 115.927254 |
| Top speed, km/h | 350 | 351.003 | 351.003 |

SciPy bounded least-squares fit four unknown effective parameters to four performance
anchors. Thirteen solver evaluations (65 native evaluations including derivatives),
35.42 s wall time; gradient termination. These anchors are **calibration**, not
out-of-sample validation. The soft speed governor permits about 1 km/h overshoot.
Braking uses a documented constant-speed rolling trim before the free braking run.
Fuel/SOC/thermal state is held fixed on this calibration bench. Road aero mode is
used. Resource-enabled laps must be assessed separately.

MCL36-inspired predictions (not published performance targets): 0–100 2.105 s,
0–200 3.922 s; braking 16.987 / 48.488 m; power/drag-limited top speed 349.059 km/h.
At 70 m/s the estimated aero map produces 20.818 kN downforce, versus P1's capped
5.884 kN. DRS changes drag 3.602→2.881 kN, downforce 20.818→18.837 kN and front
balance 45.0→49.7%. Raising the floor 150 mm reduces downforce to 7.243 kN.
These values test model behavior, not fidelity to private MCL36 aero data.

[Resource equations and limitations](../physics/resources.md) document fuel mass,
ERS limits, two-node tire heat, wear and nonlinear-tire grip coupling. Tests independently
sum consumption/discharge/recovery rates and compare them with integrated states;
they also cover exhaustion, limits, thermal feedback and timestep refinement.
