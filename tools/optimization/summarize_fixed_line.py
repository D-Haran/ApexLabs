#!/usr/bin/env python3
"""Create a machine-readable grid-refinement table from optimizer/replay outputs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--nodes", type=int, nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = []
    for nodes in args.nodes:
        optimizer = json.loads((args.directory / f"fixed-line-n{nodes}.json").read_text())
        replay = json.loads((args.directory / f"replay-n{nodes}.json").read_text())
        predicted = optimizer["objective"]["predicted_lap_time_s"]
        replayed = replay["replayed_lap_time_s"]
        rows.append({
            "nodes": nodes,
            "spacing_m": optimizer["mesh"]["spacing_m"],
            "decision_variables": optimizer["size"]["decision_variables"],
            "constraints": optimizer["size"]["constraints"],
            "predicted_lap_time_s": predicted,
            "replayed_lap_time_s": replayed,
            "replay_difference_s": replayed - predicted,
            "iterations": optimizer["solver"]["iterations"],
            "solver_wall_time_s": optimizer["solver"]["wall_time_s"],
            "maximum_dynamic_defect_m2_s2":
                optimizer["residuals"]["maximum_speed_dynamic_defect_m2_s2"],
            "maximum_tire_force_violation":
                optimizer["residuals"]["maximum_tire_force_violation"],
            "maximum_replay_position_error_m": replay["maximum_position_error_m"],
            "rms_replay_position_error_m": replay["rms_position_error_m"],
        })
    result = {
        "schema_version": 1,
        "problem": "fixed_reference_line_grid_refinement",
        "rows": rows,
        "note": "N=800 required a hierarchical N=400 warm start; a raw-reference N=800 attempt reached the iteration limit and was rejected.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
