#!/usr/bin/env python3
"""Independent linear spring balance, analytic oscillator and convergence validation."""

import csv, json, subprocess
from pathlib import Path
import numpy as np
from scipy.signal import find_peaks

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "data/generated/milestone-5_5"
OUT.mkdir(parents=True, exist_ok=True)
subprocess.run(
    [str(ROOT / "build/cpp/tests/apexlab-road-tests"), str(OUT / "body.csv")],
    check=True,
)
p = json.loads((ROOT / "configs/vehicles/mclaren-p1-sprung-approx.json").read_text())
m = p["mass_kg"]
a = p["planar"]["cg_to_front_axle_m"]
b = p["planar"]["cg_to_rear_axle_m"]
n = p["nonlinear_planar"]
h = n["cg_height_m"]
s = p["suspension"]
x = np.array([a, a, -b, -b])
y = np.array(
    [
        n["front_track_m"] / 2,
        -n["front_track_m"] / 2,
        n["rear_track_m"] / 2,
        -n["rear_track_m"] / 2,
    ]
)
B = np.array([np.ones(4), y, x]).T
k = np.array([s[w + "_spring_rate_n_m"] for w in ["fl", "fr", "rl", "rr"]])
K = B.T @ np.diag(k) @ B
rows = {}
for r in csv.DictReader((OUT / "body.csv").open()):
    r = {key: float(v) for key, v in r.items()}
    rows.setdefault((int(r["scenario"]), r["dt"]), []).append(r)
report = {
    "static_and_grade_cpp": "passed: -10,-5,0,5,10 degrees; 1e-10 m/s² analytical tolerance",
    "equilibrium": {},
    "timestep_convergence": {},
    "damping": {},
}
for scenario in range(8):
    r = rows[scenario, 0.00125][-1]
    expected = np.linalg.solve(
        K, [-r["down"] + m * (9.80665 - r["gn"]), h * r["fy"], h * r["fx"]]
    )
    measured = np.array([r["heave"], r["roll"], r["pitch"]])
    error = float(np.max(np.abs(expected - measured)))
    assert error < 1e-8, (scenario, error)
    report["equilibrium"][scenario] = {
        "heave_m": r["heave"],
        "roll_rad": r["roll"],
        "pitch_rad": r["pitch"],
        "max_analytic_error": error,
        "loads_n": [r[w] for w in ["fl", "fr", "rl", "rr"]],
    }
    if scenario in (1, 2):
        transfer = m * 9.80665 * b / (a + b) - r["fl"] - r["fr"]
        expected_transfer = r["fx"] * h / (a + b)
        assert abs(transfer - expected_transfer) < 1e-6
for scenario in (1, 3, 5):
    reference = rows[scenario, 0.00125]
    errors = []
    for dt in (0.01, 0.005, 0.0025):
        actual = rows[scenario, dt]
        ratio = round(dt / 0.00125)
        error = max(
            abs(r[f] - reference[i * ratio][f])
            for i, r in enumerate(actual)
            for f in ("heave", "roll", "pitch")
        )
        errors.append(error)
    assert errors[0] > errors[1] > errors[2], errors
    report["timestep_convergence"][scenario] = dict(
        zip(["0.01", "0.005", "0.0025"], errors)
    )
r = rows[8, 0.00125]
t = np.array([v["time"] for v in r])
z = np.array([v["heave"] for v in r])
v = np.array([v["heave_rate"] for v in r])
wn = np.sqrt(240000 / m)
zeta = 8000 / (2 * np.sqrt(240000 * m))
wd = wn * np.sqrt(1 - zeta * zeta)
analytical = (
    0.01 * np.exp(-zeta * wn * t) * (np.cos(wd * t) + zeta * wn / wd * np.sin(wd * t))
)
error = float(np.max(np.abs(z - analytical)))
assert error < 1e-9
peaks, _ = find_peaks(z)
period = float(t[peaks[1]] - t[peaks[0]])
logdec = float(np.log(z[peaks[0]] / z[peaks[1]]))
observed_zeta = logdec / np.sqrt(4 * np.pi**2 + logdec**2)
energy = 0.5 * m * v * v + 0.5 * 240000 * z * z
assert np.max(np.diff(energy)) < 1e-9
report["damping"] = {
    "analytic_natural_frequency_hz": float(wn / (2 * np.pi)),
    "analytic_damping_ratio": float(zeta),
    "measured_damped_frequency_hz": 1 / period,
    "analytic_damped_frequency_hz": float(wd / (2 * np.pi)),
    "measured_damping_ratio": float(observed_zeta),
    "max_analytic_displacement_error_m": error,
    "initial_energy_j": float(energy[0]),
    "final_energy_j": float(energy[-1]),
    "max_energy_increase_j": float(np.max(np.diff(energy))),
}
# Keep full transient evidence compressed; no lossy decimation.
import gzip, shutil

with (OUT / "body.csv").open("rb") as src, gzip.open(OUT / "body.csv.gz", "wb") as dest:
    shutil.copyfileobj(src, dest)
(OUT / "body.csv").unlink()
(OUT / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2))
