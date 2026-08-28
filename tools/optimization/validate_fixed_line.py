#!/usr/bin/env python3
"""Combine reduced-model and independent production-replay diagnostics."""

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
        "lap_time_difference_within_1_percent": abs(difference_percent) <= 1.0,
        "maximum_position_error_within_0_5_m": replay["maximum_position_error_m"] <= 0.5,
        "maximum_speed_error_within_1_mps": replay["maximum_speed_error_mps"] <= 1.0,
        "maximum_yaw_error_within_0_25_rad": replay["maximum_yaw_error_rad"] <= 0.25,
        "rms_yaw_error_within_0_05_rad": replay["rms_yaw_error_rad"] <= 0.05,
        "no_track_violation": replay["maximum_track_violation_m"] <= 1.0e-9,
        "production_tire_utilization_bounded": replay["maximum_tire_utilization"] <= 1.0 + 1.0e-8,
    }
    result = {
        "schema_version": 1,
        "accepted": all(checks.values()),
        "acceptance_checks": checks,
        "optimizer": optimizer,
        "production_replay": replay,
        "comparison": {
            "lap_time_difference_s": difference,
            "lap_time_difference_percent": difference_percent,
        },
        "interpretation": (
            "Validated locally optimized fixed-line solution within declared replay tolerances."
            if all(checks.values()) else
            "Not validated; one or more independent replay tolerances failed."
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["accepted"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
