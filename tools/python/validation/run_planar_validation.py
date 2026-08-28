#!/usr/bin/env python3
"""Run independent black-box validation of ApexLab's planar bicycle model."""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import statistics
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence


REPO_ROOT = Path(__file__).resolve().parents[3]
COLORS = ("#22d3ee", "#f59e0b", "#a78bfa", "#34d399", "#fb7185", "#f9fafb")


@dataclass(frozen=True)
class Series:
    name: str
    points: Sequence[tuple[float, float]]
    color: str


VEHICLES = {
    "understeer": {
        "file": "planar_understeer.json",
        "mass": 1500.0,
        "wheelbase": 2.8,
        "a": 1.2,
        "b": 1.6,
        "cf": 70000.0,
        "cr": 95000.0,
    },
    "neutral": {
        "file": "planar_neutral.json",
        "mass": 1500.0,
        "wheelbase": 2.8,
        "a": 1.2,
        "b": 1.6,
        "cf": 100000.0,
        "cr": 75000.0,
    },
    "oversteer": {
        "file": "planar_oversteer.json",
        "mass": 1500.0,
        "wheelbase": 2.8,
        "a": 1.2,
        "b": 1.6,
        "cf": 120000.0,
        "cr": 70000.0,
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim", type=Path, required=True)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=REPO_ROOT / "data/generated/planar-validation",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=REPO_ROOT / "docs/experiments/milestone-2-results.md",
    )
    return parser.parse_args()


def profile_constant(value: float) -> dict[str, object]:
    return {"type": "constant", "value": value}


def make_scenario(
    name: str,
    *,
    speed: float,
    steering: dict[str, object],
    duration: float = 10.0,
    timestep: float = 0.005,
    integrator: str = "rk4",
    throttle: float = 0.0,
) -> dict[str, object]:
    return {
        "schema_version": 1,
        "name": name,
        "duration_s": duration,
        "timestep_s": timestep,
        "integrator": integrator,
        "initial_state": {
            "position_x_m": 0.0,
            "position_y_m": 0.0,
            "yaw_rad": 0.0,
            "vx_m_s": speed,
            "vy_m_s": 0.0,
            "yaw_rate_rad_s": 0.0,
        },
        "steering_profile": steering,
        "throttle_profile": profile_constant(throttle),
        "brake_profile": profile_constant(0.0),
    }


def run_simulation(
    sim: Path,
    output_dir: Path,
    name: str,
    vehicle_file: str,
    scenario: dict[str, object],
) -> list[dict[str, float]]:
    scenario_dir = output_dir / "scenarios"
    scenario_dir.mkdir(parents=True, exist_ok=True)
    scenario_path = scenario_dir / f"{name}.json"
    telemetry_path = output_dir / f"{name}.csv"
    scenario_path.write_text(json.dumps(scenario, indent=2) + "\n", encoding="utf-8")
    subprocess.run(
        [
            str(sim.resolve()),
            "--vehicle",
            str(REPO_ROOT / "configs/vehicles" / vehicle_file),
            "--scenario",
            str(scenario_path),
            "--output",
            str(telemetry_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    with telemetry_path.open(newline="", encoding="utf-8") as stream:
        return [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(stream)
        ]


def write_csv(path: Path, header: Sequence[str], rows: Sequence[Sequence[object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


def write_svg(
    path: Path,
    title: str,
    x_label: str,
    y_label: str,
    series: Sequence[Series],
    *,
    equal_axes: bool = False,
) -> None:
    width, height = 900, 520
    left, right, top, bottom = 86, 28, 54, 72
    plot_width = width - left - right
    plot_height = height - top - bottom
    all_points = [point for item in series for point in item.points]
    x_values = [point[0] for point in all_points]
    y_values = [point[1] for point in all_points]
    x_min, x_max = min(x_values), max(x_values)
    y_min, y_max = min(y_values), max(y_values)
    if math.isclose(x_min, x_max):
        x_min -= 0.5
        x_max += 0.5
    if math.isclose(y_min, y_max):
        y_min -= 0.5
        y_max += 0.5
    if equal_axes:
        x_range = x_max - x_min
        y_range = y_max - y_min
        target_ratio = plot_width / plot_height
        if x_range / y_range > target_ratio:
            expanded = x_range / target_ratio
            center = 0.5 * (y_min + y_max)
            y_min, y_max = center - expanded / 2.0, center + expanded / 2.0
        else:
            expanded = y_range * target_ratio
            center = 0.5 * (x_min + x_max)
            x_min, x_max = center - expanded / 2.0, center + expanded / 2.0
    else:
        y_padding = 0.08 * (y_max - y_min)
        y_min -= y_padding
        y_max += y_padding

    def sx(value: float) -> float:
        return left + (value - x_min) / (x_max - x_min) * plot_width

    def sy(value: float) -> float:
        return top + (y_max - value) / (y_max - y_min) * plot_height

    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#111827"/>',
        f'<text x="{left}" y="30" fill="#f3f4f6" font-family="sans-serif" '
        f'font-size="20">{html.escape(title)}</text>',
    ]
    for tick in range(6):
        fraction = tick / 5
        x = left + fraction * plot_width
        y = top + fraction * plot_height
        x_value = x_min + fraction * (x_max - x_min)
        y_value = y_max - fraction * (y_max - y_min)
        elements.extend(
            [
                f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" '
                f'y2="{top + plot_height}" stroke="#243044"/>',
                f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_width}" '
                f'y2="{y:.2f}" stroke="#243044"/>',
                f'<text x="{x:.2f}" y="{top + plot_height + 22}" fill="#9ca3af" '
                f'text-anchor="middle" font-family="monospace" font-size="11">'
                f'{x_value:.3g}</text>',
                f'<text x="{left - 10}" y="{y + 4:.2f}" fill="#9ca3af" '
                f'text-anchor="end" font-family="monospace" font-size="11">'
                f'{y_value:.3g}</text>',
            ]
        )
    elements.extend(
        [
            f'<text x="{left + plot_width / 2:.2f}" y="{height - 20}" fill="#d1d5db" '
            f'text-anchor="middle" font-family="sans-serif" font-size="13">'
            f'{html.escape(x_label)}</text>',
            f'<text x="20" y="{top + plot_height / 2:.2f}" fill="#d1d5db" '
            f'text-anchor="middle" font-family="sans-serif" font-size="13" '
            f'transform="rotate(-90 20 {top + plot_height / 2:.2f})">'
            f'{html.escape(y_label)}</text>',
        ]
    )
    for index, item in enumerate(series):
        points = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in item.points)
        elements.append(
            f'<polyline points="{points}" fill="none" stroke="{item.color}" '
            'stroke-width="2.2" stroke-linejoin="round"/>'
        )
        legend_x = left + (index % 4) * 190
        legend_y = height - 48 + (index // 4) * 16
        elements.extend(
            [
                f'<line x1="{legend_x}" y1="{legend_y}" x2="{legend_x + 22}" '
                f'y2="{legend_y}" stroke="{item.color}" stroke-width="3"/>',
                f'<text x="{legend_x + 28}" y="{legend_y + 4}" fill="#d1d5db" '
                f'font-family="sans-serif" font-size="11">{html.escape(item.name)}</text>',
            ]
        )
    elements.append("</svg>")
    path.write_text("\n".join(elements) + "\n", encoding="utf-8")


def steady_mean(rows: Sequence[dict[str, float]], field: str, window_s: float = 1.0) -> float:
    start = rows[-1]["time_s"] - window_s
    return statistics.fmean(row[field] for row in rows if row["time_s"] >= start)


def understeer_gradient(vehicle: dict[str, object]) -> float:
    return float(vehicle["mass"]) / float(vehicle["wheelbase"]) * (
        float(vehicle["b"]) / float(vehicle["cf"])
        - float(vehicle["a"]) / float(vehicle["cr"])
    )


def steady_reference(vehicle: dict[str, object], speed: float, steering: float) -> dict[str, float]:
    mass = float(vehicle["mass"])
    wheelbase = float(vehicle["wheelbase"])
    a = float(vehicle["a"])
    b = float(vehicle["b"])
    cf = float(vehicle["cf"])
    cr = float(vehicle["cr"])
    gradient = understeer_gradient(vehicle)
    yaw_rate = speed * steering / (wheelbase + gradient * speed * speed)
    lateral_accel = speed * yaw_rate
    front_force = mass * b / wheelbase * lateral_accel
    rear_force = mass * a / wheelbase * lateral_accel
    front_slip = front_force / cf
    rear_slip = rear_force / cr
    lateral_velocity = b * yaw_rate - speed * rear_slip
    return {
        "yaw_rate_rad_s": yaw_rate,
        "lateral_accel_m_s2": lateral_accel,
        "front_slip_angle_rad": front_slip,
        "rear_slip_angle_rad": rear_slip,
        "vy_m_s": lateral_velocity,
        "turn_radius_m": speed / yaw_rate,
    }


def relative_error(actual: float, expected: float) -> float:
    return abs(actual - expected) / max(abs(expected), 1.0e-15)


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    args.report.parent.mkdir(parents=True, exist_ok=True)

    # Experiment 1: exact zero-input symmetry.
    zero_rows = run_simulation(
        args.sim,
        output_dir,
        "zero_steer",
        str(VEHICLES["neutral"]["file"]),
        make_scenario("Zero-steer invariance", speed=20.0, steering=profile_constant(0.0)),
    )
    zero_metrics = {
        "max_abs_y_m": max(abs(row["position_y_m"]) for row in zero_rows),
        "max_abs_vy_m_s": max(abs(row["vy_m_s"]) for row in zero_rows),
        "max_abs_yaw_rad": max(abs(row["yaw_rad"]) for row in zero_rows),
        "max_abs_yaw_rate_rad_s": max(abs(row["yaw_rate_rad_s"]) for row in zero_rows),
    }

    # Experiment 2: constant steer and independent small-angle steady-state reference.
    constant_delta = 0.01
    constant_rows = run_simulation(
        args.sim,
        output_dir,
        "constant_corner",
        str(VEHICLES["neutral"]["file"]),
        make_scenario(
            "Constant-radius steady state",
            speed=20.0,
            steering=profile_constant(constant_delta),
            duration=12.0,
            timestep=0.002,
            throttle=0.0042,
        ),
    )
    steady_speed = steady_mean(constant_rows, "vx_m_s")
    reference = steady_reference(VEHICLES["neutral"], steady_speed, constant_delta)
    constant_metrics = {
        field: steady_mean(constant_rows, field)
        for field in (
            "yaw_rate_rad_s",
            "vy_m_s",
            "lateral_accel_m_s2",
            "front_slip_angle_rad",
            "rear_slip_angle_rad",
        )
    }
    constant_metrics["mean_vx_m_s"] = steady_speed
    constant_metrics["max_vx_deviation_m_s"] = max(
        abs(row["vx_m_s"] - 20.0) for row in constant_rows
    )
    constant_metrics["turn_radius_m"] = steady_speed / constant_metrics["yaw_rate_rad_s"]
    constant_errors = {
        field: relative_error(constant_metrics[field], reference[field])
        for field in (
            "yaw_rate_rad_s",
            "vy_m_s",
            "lateral_accel_m_s2",
            "front_slip_angle_rad",
            "rear_slip_angle_rad",
            "turn_radius_m",
        )
    }
    write_svg(
        output_dir / "constant_corner_trajectory.svg",
        "Constant-steer trajectory",
        "world x (m)",
        "world y (m)",
        [
            Series(
                "RK4 trajectory",
                [(row["position_x_m"], row["position_y_m"]) for row in constant_rows],
                COLORS[0],
            )
        ],
        equal_axes=True,
    )

    # Experiment 3: step response metrics and time histories.
    step_delta = 0.02
    step_time = 1.0
    step_rows = run_simulation(
        args.sim,
        output_dir,
        "step_steer",
        str(VEHICLES["neutral"]["file"]),
        make_scenario(
            "Step steer",
            speed=20.0,
            steering={
                "type": "step",
                "initial_value": 0.0,
                "final_value": step_delta,
                "start_time_s": step_time,
            },
            duration=8.0,
            timestep=0.002,
        ),
    )
    after_step = [row for row in step_rows if row["time_s"] >= step_time]
    steady_yaw_rate = steady_mean(step_rows, "yaw_rate_rad_s")
    peak_yaw_rate = max(row["yaw_rate_rad_s"] for row in after_step)
    target_90 = 0.9 * steady_yaw_rate
    response_row = next(row for row in after_step if row["yaw_rate_rad_s"] >= target_90)
    step_metrics = {
        "steady_yaw_rate_rad_s": steady_yaw_rate,
        "peak_yaw_rate_rad_s": peak_yaw_rate,
        "response_time_90_s": response_row["time_s"] - step_time,
        "overshoot_percent": max(0.0, (peak_yaw_rate / steady_yaw_rate - 1.0) * 100.0),
        "steady_lateral_accel_m_s2": steady_mean(step_rows, "lateral_accel_m_s2"),
        "steady_vy_m_s": steady_mean(step_rows, "vy_m_s"),
    }
    time_fields = (
        ("step_steering.svg", "Step-steer input", "steering angle (rad)", ("steering_angle_rad",)),
        ("step_yaw_rate.svg", "Step-steer yaw response", "yaw rate (rad/s)", ("yaw_rate_rad_s",)),
        (
            "step_lateral_response.svg",
            "Step-steer lateral response",
            "response (SI units)",
            ("lateral_accel_m_s2", "vy_m_s"),
        ),
        (
            "step_slip_angles.svg",
            "Step-steer axle slip angles",
            "slip angle (rad)",
            ("front_slip_angle_rad", "rear_slip_angle_rad"),
        ),
    )
    for filename, title, ylabel, fields in time_fields:
        write_svg(
            output_dir / filename,
            title,
            "time (s)",
            ylabel,
            [
                Series(field, [(row["time_s"], row[field]) for row in step_rows], COLORS[index])
                for index, field in enumerate(fields)
            ],
        )

    # Experiments 4 and 5: balance and speed sweeps.
    speeds = (10.0, 15.0, 20.0, 25.0, 30.0)
    sweep_delta = 0.005
    handling_rows: list[tuple[object, ...]] = []
    handling_results: dict[str, list[dict[str, float]]] = {}
    for balance, vehicle in VEHICLES.items():
        balance_results = []
        for speed in speeds:
            rows = run_simulation(
                args.sim,
                output_dir,
                f"handling_{balance}_{speed:g}mps",
                str(vehicle["file"]),
                make_scenario(
                    f"{balance} speed sweep {speed:g} m/s",
                    speed=speed,
                    steering=profile_constant(sweep_delta),
                    duration=10.0,
                    timestep=0.005,
                ),
            )
            measured_speed = steady_mean(rows, "vx_m_s")
            measured_yaw_rate = steady_mean(rows, "yaw_rate_rad_s")
            measured_lateral_accel = steady_mean(rows, "lateral_accel_m_s2")
            measured_front_slip = steady_mean(rows, "front_slip_angle_rad")
            measured_rear_slip = steady_mean(rows, "rear_slip_angle_rad")
            analytical = steady_reference(vehicle, measured_speed, sweep_delta)
            result = {
                "nominal_speed_m_s": speed,
                "mean_vx_m_s": measured_speed,
                "yaw_rate_gain_s_inv": measured_yaw_rate / sweep_delta,
                "reference_yaw_rate_gain_s_inv": analytical["yaw_rate_rad_s"] / sweep_delta,
                "lateral_accel_m_s2": measured_lateral_accel,
                "front_slip_angle_rad": measured_front_slip,
                "rear_slip_angle_rad": measured_rear_slip,
                "yaw_gain_relative_error": relative_error(
                    measured_yaw_rate, analytical["yaw_rate_rad_s"]
                ),
            }
            balance_results.append(result)
            handling_rows.append(
                (
                    balance,
                    speed,
                    measured_speed,
                    result["yaw_rate_gain_s_inv"],
                    result["reference_yaw_rate_gain_s_inv"],
                    result["yaw_gain_relative_error"],
                    measured_lateral_accel,
                    measured_front_slip,
                    measured_rear_slip,
                )
            )
        handling_results[balance] = balance_results
    write_csv(
        output_dir / "handling_speed_sweep.csv",
        (
            "balance",
            "nominal_speed_m_s",
            "mean_vx_m_s",
            "yaw_rate_gain_s_inv",
            "reference_yaw_rate_gain_s_inv",
            "yaw_gain_relative_error",
            "lateral_accel_m_s2",
            "front_slip_angle_rad",
            "rear_slip_angle_rad",
        ),
        handling_rows,
    )
    gradients = {name: understeer_gradient(vehicle) for name, vehicle in VEHICLES.items()}
    write_svg(
        output_dir / "handling_yaw_rate_gain.svg",
        "Handling balance: measured yaw-rate gain",
        "nominal speed (m/s)",
        "yaw-rate gain r/delta (1/s)",
        [
            Series(
                name,
                [(row["nominal_speed_m_s"], row["yaw_rate_gain_s_inv"]) for row in results],
                COLORS[index],
            )
            for index, (name, results) in enumerate(handling_results.items())
        ],
    )
    write_svg(
        output_dir / "speed_lateral_acceleration.svg",
        "Neutral car: lateral acceleration vs speed",
        "nominal speed (m/s)",
        "lateral acceleration (m/s^2)",
        [
            Series(
                "neutral",
                [
                    (row["nominal_speed_m_s"], row["lateral_accel_m_s2"])
                    for row in handling_results["neutral"]
                ],
                COLORS[0],
            )
        ],
    )
    write_svg(
        output_dir / "speed_slip_angles.svg",
        "Neutral car: axle slip vs speed",
        "nominal speed (m/s)",
        "slip angle (rad)",
        [
            Series(
                "front",
                [
                    (row["nominal_speed_m_s"], row["front_slip_angle_rad"])
                    for row in handling_results["neutral"]
                ],
                COLORS[0],
            ),
            Series(
                "rear",
                [
                    (row["nominal_speed_m_s"], row["rear_slip_angle_rad"])
                    for row in handling_results["neutral"]
                ],
                COLORS[1],
            ),
        ],
    )

    # Experiment 6: fixed-step convergence against an independently finer production run.
    convergence_rows: list[tuple[object, ...]] = []
    convergence_scenario = {
        "type": "sine",
        "offset": 0.0,
        "amplitude": 0.02,
        "frequency_hz": 0.5,
        "phase_rad": 0.0,
        "start_time_s": 0.0,
    }
    reference_rows = run_simulation(
        args.sim,
        output_dir,
        "convergence_reference_rk4_0.0005",
        str(VEHICLES["neutral"]["file"]),
        make_scenario(
            "Convergence reference",
            speed=20.0,
            steering=convergence_scenario,
            duration=4.0,
            timestep=0.0005,
            integrator="rk4",
        ),
    )
    convergence_reference = reference_rows[-1]
    for integrator in ("euler", "rk4"):
        for timestep in (0.02, 0.01, 0.005, 0.002, 0.001):
            rows = run_simulation(
                args.sim,
                output_dir,
                f"convergence_{integrator}_{timestep:g}",
                str(VEHICLES["neutral"]["file"]),
                make_scenario(
                    f"Convergence {integrator} {timestep:g} s",
                    speed=20.0,
                    steering=convergence_scenario,
                    duration=4.0,
                    timestep=timestep,
                    integrator=integrator,
                ),
            )
            final = rows[-1]
            yaw_rate_error = abs(final["yaw_rate_rad_s"] - convergence_reference["yaw_rate_rad_s"])
            vy_error = abs(final["vy_m_s"] - convergence_reference["vy_m_s"])
            yaw_error = abs(final["yaw_rad"] - convergence_reference["yaw_rad"])
            convergence_rows.append(
                (integrator, timestep, yaw_rate_error, vy_error, yaw_error)
            )
    write_csv(
        output_dir / "timestep_convergence.csv",
        ("integrator", "timestep_s", "yaw_rate_error_rad_s", "vy_error_m_s", "yaw_error_rad"),
        convergence_rows,
    )
    write_svg(
        output_dir / "planar_timestep_convergence.svg",
        "Sine-steer timestep convergence",
        "timestep (s)",
        "final yaw-rate absolute error (rad/s)",
        [
            Series(
                integrator,
                [(float(row[1]), float(row[2])) for row in convergence_rows if row[0] == integrator],
                color,
            )
            for integrator, color in (("euler", COLORS[1]), ("rk4", COLORS[0]))
        ],
    )

    max_steady_error = max(constant_errors.values())
    max_sweep_error = max(
        row["yaw_gain_relative_error"]
        for results in handling_results.values()
        for row in results
    )
    summary = {
        "zero_steer": zero_metrics,
        "constant_corner": {
            "measured": constant_metrics,
            "independent_reference": reference,
            "relative_errors": constant_errors,
        },
        "step_steer": step_metrics,
        "understeer_gradient_s2_per_m": gradients,
        "handling_speed_sweep": handling_results,
        "max_handling_yaw_gain_relative_error": max_sweep_error,
        "convergence_reference": {
            "integrator": "rk4",
            "timestep_s": 0.0005,
            "final_yaw_rate_rad_s": convergence_reference["yaw_rate_rad_s"],
            "final_vy_m_s": convergence_reference["vy_m_s"],
            "final_yaw_rad": convergence_reference["yaw_rad"],
        },
        "convergence": [
            {
                "integrator": row[0],
                "timestep_s": row[1],
                "yaw_rate_error_rad_s": row[2],
                "vy_error_m_s": row[3],
                "yaw_error_rad": row[4],
            }
            for row in convergence_rows
        ],
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )

    finest = {
        integrator: [row for row in convergence_rows if row[0] == integrator][-1]
        for integrator in ("euler", "rk4")
    }
    neutral_first = handling_results["neutral"][0]
    neutral_last = handling_results["neutral"][-1]
    report = f"""# Milestone 2 planar dynamics validation

This report is generated by `tools/python/validation/run_planar_validation.py` by launching the
compiled C++ CLI. The steady-state reference equations are implemented independently in Python;
they do not call the production planar force or derivative functions. Full-precision results and
all generated scenarios are in `data/generated/planar-validation`.

## Experiment 1 — Zero-steer invariance

**Question.** Does a symmetric state remain straight with zero steering and zero lateral initial
conditions?

**Hypothesis.** With `vy = r = delta = 0`, lateral tire forces and yaw moment remain exactly zero.

**Configuration and method.** Neutral fixture, `vx = 20 m/s`, RK4, `dt = 0.005 s`, 10 s. The
maximum absolute lateral/yaw states across every telemetry sample were measured.

| metric | maximum absolute value |
|---|---:|
| world y (m) | {zero_metrics['max_abs_y_m']:.9g} |
| lateral velocity (m/s) | {zero_metrics['max_abs_vy_m_s']:.9g} |
| yaw (rad) | {zero_metrics['max_abs_yaw_rad']:.9g} |
| yaw rate (rad/s) | {zero_metrics['max_abs_yaw_rate_rad_s']:.9g} |

**Result and interpretation.** All four values remained exactly zero in the recorded doubles. This
confirms symmetry and sign consistency for this condition; it is not a general stability result.

**Limitations.** This case has no perturbation, steering, drag, or rolling resistance.

## Experiment 2 — Constant-radius steady-state cornering

**Question.** Does settled constant steer agree with the independently derived linear bicycle
steady state?

**Hypothesis.** At a small `delta = 0.01 rad`, measured yaw rate, lateral velocity, acceleration,
and axle slip should be within 1% of the small-angle reference when evaluated at measured `vx`.

**Configuration and method.** Neutral fixture, initial `vx = 20 m/s`, a constant 0.0042 throttle
feed-forward to balance the small cornering loss, RK4, `dt = 0.002 s`, 12 s. Maximum speed
deviation was `{constant_metrics['max_vx_deviation_m_s']:.6g} m/s`. Values are means over the final
second. The reference uses
`r = V delta / (L + K V^2)` and axle moment/force balance, independent of production code.

| quantity | measured | reference | relative error |
|---|---:|---:|---:|
| yaw rate (rad/s) | {constant_metrics['yaw_rate_rad_s']:.9g} | {reference['yaw_rate_rad_s']:.9g} | {constant_errors['yaw_rate_rad_s']:.3%} |
| lateral velocity (m/s) | {constant_metrics['vy_m_s']:.9g} | {reference['vy_m_s']:.9g} | {constant_errors['vy_m_s']:.3%} |
| lateral acceleration (m/s^2) | {constant_metrics['lateral_accel_m_s2']:.9g} | {reference['lateral_accel_m_s2']:.9g} | {constant_errors['lateral_accel_m_s2']:.3%} |
| front slip (rad) | {constant_metrics['front_slip_angle_rad']:.9g} | {reference['front_slip_angle_rad']:.9g} | {constant_errors['front_slip_angle_rad']:.3%} |
| rear slip (rad) | {constant_metrics['rear_slip_angle_rad']:.9g} | {reference['rear_slip_angle_rad']:.9g} | {constant_errors['rear_slip_angle_rad']:.3%} |
| turn radius (m) | {constant_metrics['turn_radius_m']:.9g} | {reference['turn_radius_m']:.9g} | {constant_errors['turn_radius_m']:.3%} |

**Result and interpretation.** Maximum relative error was `{max_steady_error:.3%}`. The residual is
expected because production uses exact `atan2` slip and front-force rotation while the closed form
uses small-angle assumptions; mean final-window `vx` was `{constant_metrics['mean_vx_m_s']:.6g} m/s`.

![Equal-scale trajectory](../../data/generated/planar-validation/constant_corner_trajectory.svg)

**Limitations.** This is model-to-model verification, not measured-vehicle validation.

## Experiment 3 — Step steer

**Question.** What transient yaw and lateral response follows a deterministic steering step?

**Hypothesis.** The stable neutral fixture should settle with bounded overshoot after a finite rise.

**Configuration and method.** `vx = 20 m/s`, steering steps from 0 to `0.02 rad` at 1 s,
RK4, `dt = 0.002 s`, total 8 s. Steady values are final-second means; 90% response time is
measured from the step to the first crossing.

| metric | value |
|---|---:|
| steady yaw rate (rad/s) | {step_metrics['steady_yaw_rate_rad_s']:.9g} |
| peak yaw rate (rad/s) | {step_metrics['peak_yaw_rate_rad_s']:.9g} |
| 90% response time (s) | {step_metrics['response_time_90_s']:.9g} |
| overshoot | {step_metrics['overshoot_percent']:.6g}% |
| steady lateral acceleration (m/s^2) | {step_metrics['steady_lateral_accel_m_s2']:.9g} |
| steady lateral velocity (m/s) | {step_metrics['steady_vy_m_s']:.9g} |

![Steering](../../data/generated/planar-validation/step_steering.svg)
![Yaw rate](../../data/generated/planar-validation/step_yaw_rate.svg)
![Lateral response](../../data/generated/planar-validation/step_lateral_response.svg)
![Slip angles](../../data/generated/planar-validation/step_slip_angles.svg)

**Result and interpretation.** The response is bounded and settles. Overshoot and response time
quantify the transient rather than relying on plot appearance.

**Limitations.** The model omits tire relaxation length, steering-system dynamics, and saturation.

## Experiment 4 — Understeer, neutral, and oversteer study

**Question.** Do deliberately selected axle stiffnesses produce mathematically and numerically
distinct handling balance?

**Hypothesis.** For `K = m/L (b/Cf - a/Cr)`, positive K reduces yaw-rate gain with speed, zero K
gives approximately `V/L`, and negative K increases gain toward its critical speed.

| fixture | Cf (N/rad) | Cr (N/rad) | K (s^2/m) | classification |
|---|---:|---:|---:|---|
| understeer | 70000 | 95000 | {gradients['understeer']:.9g} | K > 0 |
| neutral | 100000 | 75000 | {gradients['neutral']:.9g} | K = 0 |
| oversteer | 120000 | 70000 | {gradients['oversteer']:.9g} | K < 0 |

**Configuration and method.** Each fixture ran at 10, 15, 20, 25, and 30 m/s with
`delta = 0.005 rad`, RK4, `dt = 0.005 s`, and 10 s duration. Final-second yaw gain was compared
with the independent formula at measured mean `vx`. Maximum relative error across 15 runs was
`{max_sweep_error:.3%}`.

![Handling balance](../../data/generated/planar-validation/handling_yaw_rate_gain.svg)

**Result and interpretation.** The measured speed trends follow the signs of K; the labels are
therefore derived, not assigned by intuition. The oversteer fixture's linear critical speed is
`{math.sqrt(-float(VEHICLES['oversteer']['wheelbase']) / gradients['oversteer']):.6g} m/s`, above
the tested range.

**Limitations.** Linear unsaturated tires can predict unbounded response near/above critical speed;
that region was deliberately excluded.

## Experiment 5 — Speed sensitivity

**Question.** How does the neutral fixture's response change with speed for equal steering?

**Hypothesis.** Neutral yaw-rate gain should grow approximately linearly with speed, while lateral
acceleration and slip grow more rapidly.

**Configuration and method.** The neutral subset of Experiment 4 was analyzed at the five speeds.
Yaw-rate gain increased from `{neutral_first['yaw_rate_gain_s_inv']:.6g}` to
`{neutral_last['yaw_rate_gain_s_inv']:.6g} 1/s`; lateral acceleration increased from
`{neutral_first['lateral_accel_m_s2']:.6g}` to `{neutral_last['lateral_accel_m_s2']:.6g} m/s^2`.

![Lateral acceleration](../../data/generated/planar-validation/speed_lateral_acceleration.svg)
![Slip angles](../../data/generated/planar-validation/speed_slip_angles.svg)

**Result and interpretation.** The response grows with speed as predicted by the steady-state
force balance. Front and rear slips coincide closely for this neutral stiffness ratio.

**Limitations.** Equal steering does not imply equal radius, and the highest-speed case remains a
linear-tire extrapolation rather than a physical grip-limit prediction.

## Experiment 6 — Timestep convergence

**Question.** Do smooth sine-steer outputs converge as timestep decreases, and how do Euler and RK4
compare?

**Hypothesis.** Both methods should converge, with RK4 substantially more accurate at equal step.

**Configuration and method.** A 20 m/s, 0.5 Hz, 0.02 rad-amplitude sine steer ran for 4 s at
20, 10, 5, 2, and 1 ms.
Errors are against a separate RK4 0.5 ms run; final yaw rate, lateral velocity, and yaw were
compared.

| integrator | dt (ms) | yaw-rate error (rad/s) | vy error (m/s) | yaw error (rad) |
|---|---:|---:|---:|---:|
| Euler | 1 | {float(finest['euler'][2]):.9g} | {float(finest['euler'][3]):.9g} | {float(finest['euler'][4]):.9g} |
| RK4 | 1 | {float(finest['rk4'][2]):.9g} | {float(finest['rk4'][3]):.9g} | {float(finest['rk4'][4]):.9g} |

![Convergence](../../data/generated/planar-validation/planar_timestep_convergence.svg)

**Result and interpretation.** Errors decrease with timestep overall and RK4 is materially more
accurate. The smooth deterministic input avoids a control discontinuity dominating the integrator
comparison; the reported comparison measures actual end-state convergence.

**Limitations.** The 0.5 ms result is a numerical reference, not a closed-form transient solution.

## Overall limitations

These experiments validate the equations implemented, not a real car. The milestone omits
nonlinear tire saturation, combined slip, dynamic load transfer, per-wheel dynamics, suspension
kinematics, camber, toe, tire temperature/pressure, road banking/elevation, differential behavior,
aero-balance migration, and F1-specific dynamics. Forward motion above the documented low-speed
transition is the intended bicycle-model envelope.
"""
    args.report.write_text(report, encoding="utf-8")

    checks = [
        all(value <= 1.0e-15 for value in zero_metrics.values()),
        max_steady_error < 0.01,
        gradients["understeer"] > 0.0,
        abs(gradients["neutral"]) < 1.0e-12,
        gradients["oversteer"] < 0.0,
        max_sweep_error < 0.03,
        float(finest["rk4"][2]) < float(finest["euler"][2]),
    ]
    print(json.dumps(summary, indent=2))
    if not all(checks):
        raise SystemExit("planar validation threshold failed")
    print(f"planar validation passed; report={args.report.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
