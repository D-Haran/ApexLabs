#!/usr/bin/env python3
"""Run black-box analytical validation against the ApexLab simulation CLI."""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


REPO_ROOT = Path(__file__).resolve().parents[3]


@dataclass(frozen=True)
class Series:
    name: str
    points: Sequence[tuple[float, float]]
    color: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim", type=Path, required=True, help="Path to apexlab-sim")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=REPO_ROOT / "data/generated/validation",
        help="Directory for CSV, SVG, and JSON artifacts",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=REPO_ROOT / "docs/experiments/initial-milestone-results.md",
        help="Markdown report path",
    )
    return parser.parse_args()


def run_simulation(
    sim: Path,
    vehicle: str,
    output: Path,
    *,
    duration: float,
    timestep: float,
    integrator: str,
    initial_speed: float = 0.0,
    throttle: float = 0.0,
    brake: float = 0.0,
    stop_when_stationary: bool = False,
) -> list[dict[str, float]]:
    command = [
        str(sim.resolve()),
        "--vehicle",
        str(REPO_ROOT / "configs/vehicles" / vehicle),
        "--output",
        str(output),
        "--duration",
        f"{duration:.17g}",
        "--dt",
        f"{timestep:.17g}",
        "--integrator",
        integrator,
        "--initial-speed",
        f"{initial_speed:.17g}",
        "--throttle",
        f"{throttle:.17g}",
        "--brake",
        f"{brake:.17g}",
    ]
    if stop_when_stationary:
        command.append("--stop-when-stationary")
    subprocess.run(command, check=True, capture_output=True, text=True)
    with output.open(newline="", encoding="utf-8") as stream:
        return [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(stream)
        ]


def write_csv(path: Path, header: Sequence[str], rows: Iterable[Sequence[object]]) -> None:
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
) -> None:
    width, height = 900, 520
    left, right, top, bottom = 86, 28, 54, 72
    all_points = [point for item in series for point in item.points]
    x_values = [point[0] for point in all_points]
    y_values = [point[1] for point in all_points]
    x_min, x_max = min(x_values), max(x_values)
    y_min, y_max = min(y_values), max(y_values)
    if math.isclose(x_min, x_max):
        x_max = x_min + 1.0
    if math.isclose(y_min, y_max):
        y_max = y_min + 1.0
    y_padding = 0.08 * (y_max - y_min)
    y_min -= y_padding
    y_max += y_padding
    plot_width = width - left - right
    plot_height = height - top - bottom

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
                f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_height}" '
                'stroke="#243044" stroke-width="1"/>',
                f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_width}" y2="{y:.2f}" '
                'stroke="#243044" stroke-width="1"/>',
                f'<text x="{x:.2f}" y="{top + plot_height + 22}" fill="#9ca3af" '
                f'text-anchor="middle" font-family="monospace" font-size="11">{x_value:.3g}</text>',
                f'<text x="{left - 10}" y="{y + 4:.2f}" fill="#9ca3af" '
                f'text-anchor="end" font-family="monospace" font-size="11">{y_value:.3g}</text>',
            ]
        )
    elements.extend(
        [
            f'<text x="{left + plot_width / 2:.2f}" y="{height - 20}" fill="#d1d5db" '
            f'text-anchor="middle" font-family="sans-serif" font-size="13">{html.escape(x_label)}</text>',
            f'<text x="20" y="{top + plot_height / 2:.2f}" fill="#d1d5db" '
            f'text-anchor="middle" font-family="sans-serif" font-size="13" '
            f'transform="rotate(-90 20 {top + plot_height / 2:.2f})">{html.escape(y_label)}</text>',
        ]
    )
    for item_index, item in enumerate(series):
        points = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in item.points)
        elements.append(
            f'<polyline points="{points}" fill="none" stroke="{item.color}" '
            'stroke-width="2.2" stroke-linejoin="round"/>'
        )
        legend_x = left + item_index * 210
        elements.extend(
            [
                f'<line x1="{legend_x}" y1="{height - 48}" x2="{legend_x + 24}" '
                f'y2="{height - 48}" stroke="{item.color}" stroke-width="3"/>',
                f'<text x="{legend_x + 30}" y="{height - 44}" fill="#d1d5db" '
                f'font-family="sans-serif" font-size="12">{html.escape(item.name)}</text>',
            ]
        )
    elements.append("</svg>")
    path.write_text("\n".join(elements) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    args.report.parent.mkdir(parents=True, exist_ok=True)

    # Constant force: m=1000 kg, F=2000 N, a=2 m/s^2, v0=x0=0.
    acceleration_rows: dict[str, list[dict[str, float]]] = {}
    acceleration_summary: dict[str, dict[str, float]] = {}
    for integrator in ("euler", "rk4"):
        rows = run_simulation(
            args.sim,
            "validation_constant_force.json",
            output_dir / f"constant_force_{integrator}.csv",
            duration=10.0,
            timestep=0.1,
            integrator=integrator,
            throttle=1.0,
        )
        acceleration_rows[integrator] = rows
        final = rows[-1]
        acceleration_summary[integrator] = {
            "velocity_error_mps": abs(final["velocity_mps"] - 20.0),
            "position_error_m": abs(final["position_m"] - 100.0),
        }

    analytic_acceleration = [(index / 20, (index / 20) ** 2) for index in range(201)]
    write_svg(
        output_dir / "constant_force_position.svg",
        "Constant-force acceleration",
        "time (s)",
        "position (m)",
        [
            Series("analytic", analytic_acceleration, "#f9fafb"),
            Series(
                "Euler dt=0.1 s",
                [(row["time_s"], row["position_m"]) for row in acceleration_rows["euler"]],
                "#f59e0b",
            ),
            Series(
                "RK4 dt=0.1 s",
                [(row["time_s"], row["position_m"]) for row in acceleration_rows["rk4"]],
                "#22d3ee",
            ),
        ],
    )

    # Coast-down: dv/dt=-k*v^2 and v(t)=v0/(1+k*v0*t).
    rho, drag_coefficient, area, mass = 1.225, 0.35, 2.10, 1200.0
    drag_k = 0.5 * rho * drag_coefficient * area / mass
    coast_rows: dict[str, list[dict[str, float]]] = {}
    coast_summary: dict[str, dict[str, float]] = {}
    for integrator in ("euler", "rk4"):
        rows = run_simulation(
            args.sim,
            "validation_coastdown.json",
            output_dir / f"coastdown_{integrator}.csv",
            duration=20.0,
            timestep=0.05,
            integrator=integrator,
            initial_speed=40.0,
        )
        coast_rows[integrator] = rows
        final = rows[-1]
        analytic_velocity = 40.0 / (1.0 + drag_k * 40.0 * final["time_s"])
        analytic_position = math.log(1.0 + drag_k * 40.0 * final["time_s"]) / drag_k
        coast_summary[integrator] = {
            "velocity_error_mps": abs(final["velocity_mps"] - analytic_velocity),
            "position_error_m": abs(final["position_m"] - analytic_position),
        }
    analytic_coast = [
        (index / 10, 40.0 / (1.0 + drag_k * 40.0 * (index / 10)))
        for index in range(201)
    ]
    write_svg(
        output_dir / "coastdown_velocity.svg",
        "Quadratic-drag coast-down",
        "time (s)",
        "velocity (m/s)",
        [
            Series("analytic", analytic_coast, "#f9fafb"),
            Series(
                "Euler dt=0.05 s",
                [(row["time_s"], row["velocity_mps"]) for row in coast_rows["euler"]],
                "#f59e0b",
            ),
            Series(
                "RK4 dt=0.05 s",
                [(row["time_s"], row["velocity_mps"]) for row in coast_rows["rk4"]],
                "#22d3ee",
            ),
        ],
    )

    # Braking distance: constant 8000 N on 1000 kg, so d=v0^2/(2*8).
    braking_rows = []
    for initial_speed in (10.0, 20.0, 30.0):
        rows = run_simulation(
            args.sim,
            "validation_braking.json",
            output_dir / f"braking_{int(initial_speed)}mps.csv",
            duration=10.0,
            timestep=0.01,
            integrator="rk4",
            initial_speed=initial_speed,
            brake=1.0,
            stop_when_stationary=True,
        )
        final = rows[-1]
        analytical_distance = initial_speed**2 / 16.0
        braking_rows.append(
            (
                initial_speed,
                analytical_distance,
                final["position_m"],
                abs(final["position_m"] - analytical_distance),
                initial_speed / 8.0,
                final["time_s"],
            )
        )
    write_csv(
        output_dir / "braking_summary.csv",
        [
            "initial_speed_mps",
            "analytical_distance_m",
            "simulated_distance_m",
            "absolute_error_m",
            "analytical_stop_time_s",
            "simulated_stop_time_s",
        ],
        braking_rows,
    )
    write_svg(
        output_dir / "braking_distance.svg",
        "Constant-force braking distance",
        "initial velocity (m/s)",
        "stopping distance (m)",
        [
            Series("analytic", [(row[0], row[1]) for row in braking_rows], "#f9fafb"),
            Series("RK4 dt=0.01 s", [(row[0], row[2]) for row in braking_rows], "#22d3ee"),
        ],
    )

    # A terminal-speed equilibrium checks the complete drive/rolling/drag force balance.
    rolling_force = 0.01 * 1000.0 * 9.80665
    terminal_velocity = math.sqrt((1200.0 - rolling_force) / (0.5 * 1.2 * 0.5 * 2.0))
    terminal_rows = run_simulation(
        args.sim,
        "validation_terminal_velocity.json",
        output_dir / "terminal_velocity_equilibrium.csv",
        duration=10.0,
        timestep=0.05,
        integrator="rk4",
        initial_speed=terminal_velocity,
        throttle=1.0,
    )
    terminal_error = abs(terminal_rows[-1]["velocity_mps"] - terminal_velocity)

    # Timestep convergence uses the same coast-down case and exact final speed.
    convergence_rows = []
    previous_errors: dict[str, float | None] = {"euler": None, "rk4": None}
    for integrator in ("euler", "rk4"):
        for timestep in (2.0, 1.0, 0.5, 0.25, 0.125):
            rows = run_simulation(
                args.sim,
                "validation_coastdown.json",
                output_dir / f"convergence_{integrator}_{timestep:g}.csv",
                duration=20.0,
                timestep=timestep,
                integrator=integrator,
                initial_speed=40.0,
            )
            expected = 40.0 / (1.0 + drag_k * 40.0 * 20.0)
            error = abs(rows[-1]["velocity_mps"] - expected)
            previous = previous_errors[integrator]
            observed_order = ""
            if previous is not None and error > 0.0:
                observed_order = f"{math.log(previous / error, 2.0):.9g}"
            convergence_rows.append((integrator, timestep, rows[-1]["velocity_mps"], error, observed_order))
            previous_errors[integrator] = error
    write_csv(
        output_dir / "timestep_convergence.csv",
        ["integrator", "timestep_s", "final_velocity_mps", "absolute_error_mps", "observed_order"],
        convergence_rows,
    )
    write_svg(
        output_dir / "timestep_convergence.svg",
        "Timestep convergence: final-velocity error",
        "timestep (s)",
        "absolute error (m/s)",
        [
            Series(
                integrator,
                [(row[1], row[3]) for row in convergence_rows if row[0] == integrator],
                color,
            )
            for integrator, color in (("euler", "#f59e0b"), ("rk4", "#22d3ee"))
        ],
    )

    summary = {
        "constant_force": acceleration_summary,
        "coastdown": coast_summary,
        "braking_max_distance_error_m": max(row[3] for row in braking_rows),
        "terminal_velocity_mps": terminal_velocity,
        "terminal_equilibrium_error_mps": terminal_error,
        "convergence": [
            {
                "integrator": row[0],
                "timestep_s": row[1],
                "final_velocity_mps": row[2],
                "absolute_error_mps": row[3],
                "observed_order": None if row[4] == "" else float(row[4]),
            }
            for row in convergence_rows
        ],
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )

    finest_euler = [row for row in convergence_rows if row[0] == "euler"][-1]
    finest_rk4 = [row for row in convergence_rows if row[0] == "rk4"][-1]
    report = f"""# Initial milestone validation results

This report is generated by `tools/python/validation/run_validation.py` from the compiled C++ CLI.
The vehicle files used here are analytical fixtures, not real-vehicle calibrations. Results were
regenerated locally; full-precision values are in `data/generated/validation/summary.json`.

## Constant-force acceleration

Setup: `m = 1000 kg`, `F = 2000 N`, `v0 = 0`, duration `10 s`, and `dt = 0.1 s`.
The analytical result is `v = 20 m/s` and `x = 100 m`.

| Integrator | final velocity error (m/s) | final position error (m) |
|---|---:|---:|
| Euler | {acceleration_summary['euler']['velocity_error_mps']:.9g} | {acceleration_summary['euler']['position_error_m']:.9g} |
| RK4 | {acceleration_summary['rk4']['velocity_error_mps']:.9g} | {acceleration_summary['rk4']['position_error_m']:.9g} |

Euler integrates velocity exactly for constant acceleration but uses beginning-of-step velocity for
position, producing the expected `1 m` error. RK4 reproduces the quadratic position solution to
floating-point precision.

![Constant-force position](../../data/generated/validation/constant_force_position.svg)

## Quadratic-drag coast-down

Setup: `m = 1200 kg`, `rho = 1.225 kg/m^3`, `Cd = 0.35`, `A = 2.10 m^2`,
`v0 = 40 m/s`, duration `20 s`, and `dt = 0.05 s`. Rolling resistance is disabled so the closed
form `v(t) = v0 / (1 + k v0 t)` applies.

| Integrator | final velocity error (m/s) | final position error (m) |
|---|---:|---:|
| Euler | {coast_summary['euler']['velocity_error_mps']:.9g} | {coast_summary['euler']['position_error_m']:.9g} |
| RK4 | {coast_summary['rk4']['velocity_error_mps']:.9g} | {coast_summary['rk4']['position_error_m']:.9g} |

![Coast-down velocity](../../data/generated/validation/coastdown_velocity.svg)

## Constant-force braking

Setup: `m = 1000 kg`, brake force `8000 N`, RK4, and `dt = 0.01 s`. The stop event is
located inside the final nominal step and clamped at zero speed.

| initial speed (m/s) | analytical distance (m) | simulated distance (m) | error (m) |
|---:|---:|---:|---:|
"""
    for row in braking_rows:
        report += f"| {row[0]:.0f} | {row[1]:.9g} | {row[2]:.9g} | {row[3]:.9g} |\n"
    report += f"""

Maximum distance error: `{max(row[3] for row in braking_rows):.9g} m`.

![Braking distance](../../data/generated/validation/braking_distance.svg)

## Terminal-velocity equilibrium

For `1200 N` drive force and `98.0665 N` rolling resistance, the analytical force balance gives
`v_terminal = {terminal_velocity:.9g} m/s`. Starting exactly at this speed changed velocity by
`{terminal_error:.9g} m/s` over 10 seconds, within floating-point evaluation error.

## Timestep convergence

The coast-down simulation was repeated from `dt = 2 s` through `0.125 s`. At the finest tested
step, Euler's final-speed error was `{finest_euler[3]:.9g} m/s` and RK4's was
`{finest_rk4[3]:.9g} m/s`. Pairwise observed orders are recorded in
`data/generated/validation/timestep_convergence.csv`; they approach first order for Euler and
fourth order for RK4 as expected for this smooth ODE.

![Timestep convergence](../../data/generated/validation/timestep_convergence.svg)

## Interpretation and limitations

These experiments validate implementation behavior against equations the model was designed to
solve. They do not validate fidelity to a real vehicle. In particular, constant force fixtures
remove power curves, tire slip, load transfer, road grade, and environmental variability. The
terminal-speed case checks equilibrium consistency, not a measured top speed. Later model changes
must retain these fixtures as regression tests and add independent measured-data validation.
"""
    args.report.write_text(report, encoding="utf-8")

    checks = [
        acceleration_summary["rk4"]["position_error_m"] < 1.0e-8,
        coast_summary["rk4"]["velocity_error_mps"] < 1.0e-8,
        max(row[3] for row in braking_rows) < 1.0e-8,
        terminal_error < 1.0e-9,
        all(
            later[3] < earlier[3]
            for integrator in ("euler", "rk4")
            for earlier, later in zip(
                [row for row in convergence_rows if row[0] == integrator],
                [row for row in convergence_rows if row[0] == integrator][1:],
            )
        ),
    ]
    print(json.dumps(summary, indent=2))
    if not all(checks):
        raise SystemExit("validation threshold failed")
    print(f"validation passed; report={args.report.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
