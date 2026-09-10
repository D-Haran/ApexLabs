#!/usr/bin/env python3
"""Run native 3D-contact benches, check convergence and package honest interval replays."""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
WHEELS = ("fl", "fr", "rl", "rr")


def read_csv(path):
    with path.open() as stream:
        return [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]


def summarize(rows, run):
    dt = float(run["dt_s"])
    all_air = [all(row[f"{w}_contact"] == 0 for w in WHEELS) for row in rows]
    first_air = next((i for i, value in enumerate(all_air) if value), None)
    landing = next((i for i in range((first_air or 0) + 1, len(rows))
                    if all_air[i - 1] and not all_air[i]), None) if first_air is not None else None
    apex = min(rows, key=lambda r: abs(r["x_m"]))
    for row in rows:
        if not all(math.isfinite(v) for v in row.values()):
            raise ValueError("Nonfinite contact telemetry")
        for w in WHEELS:
            if row[f"{w}_fz"] < 0 or not 0 <= row[f"{w}_utilization"] <= 1 + 1e-10:
                raise ValueError("Invalid unilateral/friction force")
            if row[f"{w}_contact"] not in (0, 1):
                raise ValueError("Invalid contact flag")
            if not row[f"{w}_contact"] and any(row[f"{w}_{axis}"] != 0 for axis in ("fx", "fy", "fz")):
                raise ValueError("Airborne tire force")
        qnorm = sum(row[k] ** 2 for k in ("qw", "qx", "qy", "qz"))
        if abs(qnorm - 1) > 1e-8:
            raise ValueError("Quaternion drift")
    result = {
        "initial_speed_mps": float(run["initial_speed_mps"]), "dt_s": dt,
        "simulated_duration_s": rows[-1]["time_s"], "steps": int(run["steps"]),
        "wall_s_including_csv": float(run["wall_s"]),
        "min_contact_count": min(sum(row[f"{w}_contact"] for w in WHEELS) for row in rows),
        "all_airborne_duration_s": sum(all_air[:-1]) * dt,
        "first_all_airborne_time_s": rows[first_air]["time_s"] if first_air is not None else None,
        "first_landing_time_s": rows[landing]["time_s"] if landing is not None else None,
        "max_body_height_above_static_road_m": max(r["z_m"] - r["road_height_m"] - .45 for r in rows),
        "peak_wheel_force_n": max(r[f"{w}_fz"] for r in rows for w in WHEELS),
        "peak_total_normal_force_n": max(sum(r[f"{w}_fz"] for w in WHEELS) for r in rows),
        "peak_landing_total_force_n": max(sum(r[f"{w}_fz"] for w in WHEELS) for r in rows[landing:]) if landing is not None else None,
        "max_suspension_compression_m": max(r[f"{w}_suspension_compression"] for r in rows for w in WHEELS),
        "max_tire_compression_m": max(r[f"{w}_tire_compression"] for r in rows for w in WHEELS),
        "max_equation_residual": max(r["equation_residual"] for r in rows),
        "final_contact_count": sum(rows[-1][f"{w}_contact"] for w in WHEELS),
    }
    if run["scene"].startswith("crest-"):
        radius = float(run["crest_radius_m"])
        result["crest_radius_m"] = radius
        result["nominal_crest_vertical_demand_mps2"] = -float(run["nominal_crest_demand_mps2"])
        result["sample_nearest_crest"] = {
            "x_m": apex["x_m"], "speed_mps": math.hypot(apex["vx_mps"], apex["vy_mps"], apex["vz_mps"]),
            "constraint_vertical_demand_mps2": -apex["vx_mps"] ** 2 / radius,
            "actual_body_vertical_acceleration_mps2": apex["az_mps2"],
            "wheel_fz_n": [apex[f"{w}_fz"] for w in WHEELS],
        }
    return result


def vector(row, fields):
    return [row[key] for key in fields]


def replay(directory, destination, name, summary, parameters):
    rows = read_csv(directory / name / "contact.csv")
    stride = max(1, round(.02 / summary["dt_s"]))
    selected = rows[::stride]
    if selected[-1] is not rows[-1]:
        selected.append(rows[-1])
    samples = []
    for r in selected:
        samples.append({
            "time": r["time_s"], "position": vector(r, ("x_m", "y_m", "z_m")),
            "quaternion": vector(r, ("qw", "qx", "qy", "qz")),
            "velocity": vector(r, ("vx_mps", "vy_mps", "vz_mps")),
            "acceleration": vector(r, ("ax_mps2", "ay_mps2", "az_mps2")),
            "steering": r["steering_rad"], "throttle": r["throttle"], "brake": r["brake"],
            "wheels": [{
                "center": vector(r, [f"{w}_center_{a}" for a in "xyz"]),
                "patch": vector(r, [f"{w}_patch_{a}" for a in "xyz"]),
                "force": vector(r, [f"{w}_force_{a}" for a in "xyz"]),
                "contact": bool(r[f"{w}_contact"]), "fz": r[f"{w}_fz"],
                "fx": r[f"{w}_fx"], "fy": r[f"{w}_fy"],
                "slip": r[f"{w}_slip"], "utilization": r[f"{w}_utilization"],
                "compression": r[f"{w}_suspension_compression"],
                "tireCompression": r[f"{w}_tire_compression"],
                "damperVelocity": -r[f"{w}_extension_rate"], "damperForce": r[f"{w}_damper_force"],
                "material": int(r[f"{w}_material"]),
            } for w in WHEELS],
        })
    surface = read_csv(directory / name / "surface.csv")
    mesh = {
        "columns": int(max(r["iy"] for r in surface)) + 1,
        "positions": [v for r in surface for v in vector(r, ("x_m", "y_m", "z_m"))],
        "normals": [v for r in surface for v in vector(r, ("nx", "ny", "nz"))],
        "materials": [int(r["material"]) for r in surface],
    }
    # Values come from native queries, including curb heights; TS doesn't rebuild a road model.
    accuracy = json.loads((directory / name / "surface-accuracy.json").read_text())
    if accuracy["maximum_triangle_centroid_height_error_m"] > .001:
        raise ValueError("Render/physics surface alignment exceeds 1 mm")
    data = {"schema_version": 1, "kind": "contact_validation_interval", "name": name,
            "source": "C++ ContactVehicleModel; synthetic geometry; estimated vertical parameters",
            "coordinates": "world +X forward, +Y left, +Z up; quaternion w,x,y,z",
            "summary": summary, "parameters": parameters, "samples": samples, "surface": mesh,
            "surface_validation": accuracy}
    (destination / f"{name}.json").write_text(json.dumps(data, separators=(",", ":"), allow_nan=False) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=ROOT / "build/apps/sim-cli/apexlab-contact-scenes")
    parser.add_argument("--output", type=Path, default=ROOT / "data/generated/contact-3d")
    parser.add_argument("--reuse", action="store_true")
    args = parser.parse_args()
    all_results = {}
    for dt, folder in ((.002, "dt-002"), (.001, "dt-001"), (.0005, "dt-0005")):
        directory = args.output / folder
        if not args.reuse:
            subprocess.run([str(args.binary), str(ROOT / "configs/vehicles/mclaren-p1-sprung-approx.json"),
                            str(directory), str(dt)], check=True)
        with (directory / "runs.csv").open() as stream:
            runs = list(csv.DictReader(stream))
        all_results[folder] = {run["scene"]: summarize(read_csv(directory / run["scene"] / "contact.csv"), run)
                               for run in runs}
    baseline = all_results["dt-002"]
    for folder, results in all_results.items():
        if results["crest-12"]["min_contact_count"] != 4:
            raise ValueError(f"{folder}: low-speed contact failure")
        if results["crest-40"]["all_airborne_duration_s"] < .05:
            raise ValueError(f"{folder}: high-speed contact failure")
        for name, row in results.items():
            comparison = baseline[name]
            if abs(row["all_airborne_duration_s"] - comparison["all_airborne_duration_s"]) > .01:
                raise ValueError(f"{folder}/{name}: contact duration convergence failure")
            for key, tolerance in (("max_body_height_above_static_road_m", .005),
                                   ("peak_total_normal_force_n", comparison["peak_total_normal_force_n"] * .03)):
                if abs(row[key] - comparison[key]) > tolerance:
                    raise ValueError(f"{folder}/{name}: {key} convergence failure")
    report = {"schema_version": 1, "status": "passed", "scope": "Phase A synthetic contact validation only; no lap optimization or vehicle calibration",
              "grid": all_results}
    (args.output / "validation.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    destination = ROOT / "apps/dashboard/public/demo/contact-3d"
    destination.mkdir(parents=True, exist_ok=True)
    parameters = json.loads((args.output / "dt-002/parameters.json").read_text())
    legacy = json.loads((ROOT / "configs/vehicles/mclaren-p1-sprung-approx.json").read_text())
    parameters["body_axle_midpoint_m"] = (legacy["planar"]["cg_to_front_axle_m"] - legacy["planar"]["cg_to_rear_axle_m"]) / 2
    for name in ("crest-12", "crest-40", "drop", "curb", "grass"):
        replay(args.output / "dt-002", destination, name, baseline[name], parameters)
    print(json.dumps({name: {k: row[k] for k in ("all_airborne_duration_s", "peak_total_normal_force_n",
                                                "max_body_height_above_static_road_m")} for name, row in baseline.items()}, indent=2))
    print("PASS: all contact benches at 2, 1 and 0.5 ms; finite forces and timestep convergence")


if __name__ == "__main__":
    main()
