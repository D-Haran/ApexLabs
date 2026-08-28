#!/usr/bin/env python3
"""Generate seven C++ physical test intervals and mark them explicitly as validation scenes."""

import hashlib, json, subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
out = ROOT / "apps/dashboard/public/demo/chassis-scenes"
vehicle = ROOT / "configs/vehicles/mclaren-p1-sprung-approx.json"
subprocess.run(
    [str(ROOT / "build/apps/sim-cli/apexlab-chassis-scenes"), str(vehicle), str(out)],
    check=True,
)
v = json.loads(vehicle.read_text())
p = v["planar"]
n = v["nonlinear_planar"]
for name in ["flat", "uphill", "downhill", "left", "right", "braking", "acceleration"]:
    d = out / name
    title = {
        "flat": "Flat straight",
        "uphill": "10° uphill",
        "downhill": "10° downhill",
        "left": "70 m left turn",
        "right": "70 m right turn",
        "braking": "Braking zone",
        "acceleration": "Accelerating exit",
    }[name]
    aero = (
        v["aero"]
        if name not in ["flat", "uphill", "downhill"]
        else {k: 0 for k in v["aero"]}
    )
    manifest = {
        "schema_version": 1,
        "telemetry_schema_version": 4,
        "wheel_schema_version": 1,
        "spatial_schema_version": 1,
        "session_kind": "validation_scene",
        "session_name": title + " · physical validation interval",
        "files": {
            k: k + ".csv"
            for k in ["telemetry", "wheels", "geometry", "spatial", "chassis"]
        },
        "vehicle": {
            "name": v["name"],
            "mass_kg": v["mass_kg"],
            "mu_reference": v["tires"]["tire_mu_reference"],
            "front_axle_m": p["cg_to_front_axle_m"],
            "rear_axle_m": p["cg_to_rear_axle_m"],
            "front_track_m": n["front_track_m"],
            "rear_track_m": n["rear_track_m"],
            "cg_height_m": n["cg_height_m"],
            "normal_load_model": "sprung_body",
            "aero": aero,
            "suspension": v["suspension"],
            "provenance": v["provenance"]
            + "; grade/flat scenes disable aero and rolling resistance for analytical coastdown.",
        },
        "track": {
            "name": title,
            "length_m": 300,
            "reference_offset_m": 0,
            "geometry_sha256": hashlib.sha256(
                (d / "geometry.csv").read_bytes()
            ).hexdigest(),
            "sector_boundaries_fraction": [1],
            "kind": "synthetic_validation",
            "provenance": "Analytical open test road, not a completed racing lap.",
        },
        "simulation": {
            "timestep_s": 0.005,
            "controller_timestep_s": 0.005,
            "integrator": "rk4",
            "warmup_laps": 0,
        },
        "laps": [
            {
                "number": 0,
                "start_time_s": 0,
                "end_time_s": 6,
                "lap_time_s": 6,
                "sector_times_s": [6],
            }
        ],
    }
    (d / "session.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n"
    )
print(
    "Scenes exported; open /?scene=flat|uphill|downhill|left|right|braking|acceleration"
)

# Independent end-state checks against analytical grade motion and expected moment signs.
import csv
import math

results = {}
for name in ["flat", "uphill", "downhill", "left", "right", "braking", "acceleration"]:
    with (out / name / "telemetry.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    with (out / name / "chassis.csv").open() as stream:
        body = list(csv.DictReader(stream))
    end = rows[-1]
    result = {
        "end_speed_m_s": float(end["speed_m_s"]),
        "roll_rad": float(body[-1]["roll_rad"]),
        "pitch_rad": float(body[-1]["pitch_rad"]),
        "max_lateral_error_m": max(abs(float(r["lateral_error_m"])) for r in rows),
        "all_on_track": all(int(r["on_track"]) == 1 for r in rows),
    }
    assert result["all_on_track"], name
    if name in ["flat", "uphill", "downhill"]:
        angle = {"flat": 0, "uphill": 10, "downhill": -10}[name]
        expected = 20 - 9.80665 * math.sin(math.radians(angle)) * float(end["time_s"])
        result["analytic_speed_error_m_s"] = abs(result["end_speed_m_s"] - expected)
        assert result["analytic_speed_error_m_s"] < 1e-10, (name, result)
    if name in ["left", "right"]:
        assert result["roll_rad"] * (1 if name == "left" else -1) > 0.01
    if name in ["braking", "acceleration"]:
        assert result["pitch_rad"] * (1 if name == "acceleration" else -1) > 0.002
    results[name] = result
report = ROOT / "data/generated/milestone-5_5/scene-validation.json"
report.parent.mkdir(parents=True, exist_ok=True)
report.write_text(json.dumps(results, indent=2) + "\n")
print("All seven physical intervals passed independent analytical/sign checks.")
