#!/usr/bin/env python3
"""Apply declared acceptance limits to a free-line solve and production replay."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--optimizer", type=Path, required=True)
    parser.add_argument("--replay", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    optimizer = json.loads(args.optimizer.read_text(encoding="utf-8"))
    replay = json.loads(args.replay.read_text(encoding="utf-8"))
    predicted = optimizer["objective"]["predicted_lap_time_s"]
    replayed = replay["replayed_lap_time_s"]
    difference = replayed - predicted
    difference_percent = 100.0 * difference / predicted
    checks = {
        "solver_success": optimizer["solver"]["success"],
        "replay_completed": replay["completed"],
        "lap_time_difference_within_2_percent": abs(difference_percent) <= 2.0,
        "maximum_position_error_within_2_m": replay["maximum_position_error_m"] <= 2.0,
        "maximum_speed_error_within_2_mps": replay["maximum_speed_error_mps"] <= 2.0,
        "maximum_yaw_error_within_0_5_rad": replay["maximum_yaw_error_rad"] <= 0.5,
        "rms_yaw_error_within_0_15_rad": replay["rms_yaw_error_rad"] <= 0.15,
        "no_track_violation": replay["maximum_track_violation_m"] <= 1.0e-9,
        "production_tire_utilization_bounded":
            replay["maximum_tire_utilization"] <= 1.0 + 1.0e-8,
        "optimizer_track_constraint_satisfied":
            optimizer["residuals"]["maximum_track_violation_m"] <= 1.0e-9,
    }
    accepted = all(checks.values())
    result = {
        "schema_version": 1,
        "accepted": accepted,
        "acceptance_checks": checks,
        "optimizer": optimizer,
        "production_replay": replay,
        "comparison": {
            "lap_time_difference_s": difference,
            "lap_time_difference_percent": difference_percent,
        },
        "interpretation": (
            "Validated locally optimized racing-line solution within declared replay tolerances."
            if accepted else
            "Not validated; one or more independent replay tolerances failed."
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if accepted else 2


if __name__ == "__main__":
    raise SystemExit(main())
