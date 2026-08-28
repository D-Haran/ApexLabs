#!/usr/bin/env python3
"""Run reproducible fixed-line grip and mass sensitivity experiments."""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python", type=Path, required=True)
    parser.add_argument("--optimizer", type=Path, required=True)
    parser.add_argument("--vehicle", type=Path, required=True)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--nodes", type=int, default=100)
    args = parser.parse_args()
    base = json.loads(args.vehicle.read_text(encoding="utf-8"))
    base_mass = float(base["mass_kg"])
    base_mu = float(base["tires"]["tire_mu_reference"])
    cases = [
        ("grip-0.90", base_mass, 0.90),
        (f"grip-{base_mu:.2f}", base_mass, base_mu),
        ("grip-1.35", base_mass, 1.35),
        ("mass-1200", 1200.0, base_mu),
        (f"mass-{base_mass:.0f}", base_mass, base_mu),
        ("mass-1600", 1600.0, base_mu),
    ]
    rows = []
    args.output.mkdir(parents=True, exist_ok=True)
    for name, mass, mu in cases:
        case_dir = args.output / name
        case_dir.mkdir(parents=True, exist_ok=True)
        vehicle = json.loads(json.dumps(base))
        vehicle["name"] = f'{base["name"]} · sensitivity {name}'
        vehicle["mass_kg"] = mass
        vehicle["tires"]["tire_mu_reference"] = mu
        vehicle["provenance"] += "; generated sensitivity perturbation, not a vehicle claim"
        config = case_dir / "vehicle.json"
        config.write_text(json.dumps(vehicle, indent=2) + "\n", encoding="utf-8")
        command = [
            str(args.python), str(args.optimizer), "--vehicle", str(config),
            "--session", str(args.session), "--output", str(case_dir),
            "--nodes", str(args.nodes), "--maximum-iterations", "3000",
            "--print-level", "0",
        ]
        completed = subprocess.run(command, text=True, capture_output=True)
        diagnostic_path = case_dir / f"fixed-line-n{args.nodes}.json"
        if completed.returncode == 0 and diagnostic_path.is_file():
            diagnostic = json.loads(diagnostic_path.read_text(encoding="utf-8"))
            rows.append({
                "case": name, "mass_kg": mass, "mu_reference": mu,
                "success": True,
                "lap_time_s": diagnostic["objective"]["predicted_lap_time_s"],
                "iterations": diagnostic["solver"]["iterations"],
                "wall_time_s": diagnostic["solver"]["wall_time_s"],
                "maximum_tire_force_violation":
                    diagnostic["residuals"]["maximum_tire_force_violation"],
            })
        else:
            rows.append({
                "case": name, "mass_kg": mass, "mu_reference": mu,
                "success": False, "return_code": completed.returncode,
                "failure_tail": (completed.stdout + completed.stderr)[-1000:],
            })
    result = {
        "schema_version": 1,
        "problem": "fixed_line_grip_and_mass_sensitivity",
        "nodes": args.nodes,
        "base_vehicle": str(args.vehicle),
        "rows": rows,
        "note": "These are reduced-model local solutions; sensitivity trends are not OEM claims.",
    }
    summary = args.output / "summary.json"
    summary.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if all(row["success"] for row in rows) else 2


if __name__ == "__main__":
    raise SystemExit(main())
