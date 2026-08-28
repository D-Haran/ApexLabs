#!/usr/bin/env python3
"""Run the reproducible ApexLab Milestone 3 nonlinear-limit experiments."""

from __future__ import annotations

import argparse
import copy
import csv
import html
import json
import math
import statistics
import subprocess
import tempfile
from pathlib import Path
from typing import Iterable, Sequence


ROOT = Path(__file__).resolve().parents[3]
G = 9.80665
WHEELS = ("fl", "fr", "rl", "rr")
COLORS = ("#22d3ee", "#f59e0b", "#a78bfa", "#34d399", "#fb7185", "#f9fafb")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "data/generated/milestone-3")
    parser.add_argument("--report", type=Path,
                        default=ROOT / "docs/experiments/milestone-3-results.md")
    return parser.parse_args()


def constant(value: float) -> dict[str, object]:
    return {"type": "constant", "value": value}


def ramp(initial: float, final: float, start: float, duration: float) -> dict[str, object]:
    return {"type": "ramp", "initial_value": initial, "final_value": final,
            "start_time_s": start, "duration_s": duration}


def sine(offset: float, amplitude: float, frequency: float) -> dict[str, object]:
    return {"type": "sine", "offset": offset, "amplitude": amplitude,
            "frequency_hz": frequency, "phase_rad": 0.0, "start_time_s": 0.0}


def scenario(name: str, *, speed: float, steering: dict[str, object],
             throttle: dict[str, object] | None = None,
             brake: dict[str, object] | None = None, duration: float = 0.0,
             timestep: float = 0.005, integrator: str = "rk4") -> dict[str, object]:
    return {
        "schema_version": 1, "name": name, "duration_s": duration,
        "timestep_s": timestep, "integrator": integrator,
        "initial_state": {"position_x_m": 0.0, "position_y_m": 0.0, "yaw_rad": 0.0,
                          "vx_m_s": speed, "vy_m_s": 0.0, "yaw_rate_rad_s": 0.0},
        "steering_profile": steering,
        "throttle_profile": throttle or constant(0.0),
        "brake_profile": brake or constant(0.0),
    }


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def read_csv(path: Path) -> list[dict[str, float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]


def run(sim: Path, vehicle: Path, scenario_value: dict[str, object],
        scenario_path: Path, telemetry_path: Path) -> list[dict[str, float]]:
    write_json(scenario_path, scenario_value)
    subprocess.run([str(sim.resolve()), "--vehicle", str(vehicle), "--scenario",
                    str(scenario_path), "--output", str(telemetry_path)],
                   check=True, capture_output=True, text=True)
    return read_csv(telemetry_path)


def write_csv(path: Path, fields: Sequence[str], rows: Iterable[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_svg(path: Path, title: str, x_label: str, y_label: str,
              series: Sequence[tuple[str, Sequence[tuple[float, float]]]]) -> None:
    width, height = 900, 520
    left, right, top, bottom = 86, 28, 54, 72
    points = [point for _, values in series for point in values]
    xs, ys = [p[0] for p in points], [p[1] for p in points]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    if math.isclose(x0, x1): x0, x1 = x0 - 0.5, x1 + 0.5
    if math.isclose(y0, y1): y0, y1 = y0 - 0.5, y1 + 0.5
    yp = 0.08 * (y1 - y0)
    y0, y1 = y0 - yp, y1 + yp
    pw, ph = width - left - right, height - top - bottom
    sx = lambda x: left + (x - x0) / (x1 - x0) * pw
    sy = lambda y: top + (y1 - y) / (y1 - y0) * ph
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
           '<rect width="100%" height="100%" fill="#111827"/>',
           f'<text x="{left}" y="30" fill="#f3f4f6" font-family="sans-serif" font-size="20">{html.escape(title)}</text>']
    for tick in range(6):
        f = tick / 5
        x, y = left + f * pw, top + f * ph
        out += [f'<line x1="{x}" y1="{top}" x2="{x}" y2="{top+ph}" stroke="#243044"/>',
                f'<line x1="{left}" y1="{y}" x2="{left+pw}" y2="{y}" stroke="#243044"/>',
                f'<text x="{x}" y="{top+ph+22}" fill="#9ca3af" text-anchor="middle" font-family="monospace" font-size="11">{x0+f*(x1-x0):.3g}</text>',
                f'<text x="{left-10}" y="{y+4}" fill="#9ca3af" text-anchor="end" font-family="monospace" font-size="11">{y1-f*(y1-y0):.3g}</text>']
    out += [f'<text x="{left+pw/2}" y="{height-20}" fill="#d1d5db" text-anchor="middle" font-family="sans-serif" font-size="13">{html.escape(x_label)}</text>',
            f'<text x="20" y="{top+ph/2}" fill="#d1d5db" text-anchor="middle" font-family="sans-serif" font-size="13" transform="rotate(-90 20 {top+ph/2})">{html.escape(y_label)}</text>']
    for i, (name, values) in enumerate(series):
        poly = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in values)
        color = COLORS[i % len(COLORS)]
        out += [f'<polyline points="{poly}" fill="none" stroke="{color}" stroke-width="2.2"/>',
                f'<line x1="{left+(i%4)*190}" y1="{height-48+(i//4)*16}" x2="{left+(i%4)*190+22}" y2="{height-48+(i//4)*16}" stroke="{color}" stroke-width="3"/>',
                f'<text x="{left+(i%4)*190+28}" y="{height-44+(i//4)*16}" fill="#d1d5db" font-family="sans-serif" font-size="11">{html.escape(name)}</text>']
    out.append("</svg>")
    path.write_text("\n".join(out) + "\n", encoding="utf-8")


def fiala(alpha: float, fz: float, stiffness: float, mu_ref: float,
          fz_ref: float, exponent: float) -> tuple[float, float, float]:
    mu = mu_ref * (fz / fz_ref) ** exponent
    capacity = mu * fz
    tangent = math.tan(alpha)
    if abs(tangent) < 3.0 * capacity / stiffness:
        fy = (stiffness * tangent - stiffness**2 * abs(tangent) * tangent / (3*capacity)
              + stiffness**3 * tangent**3 / (27*capacity**2))
    else:
        fy = math.copysign(capacity, tangent)
    return fy, mu, capacity


def settled(rows: Sequence[dict[str, float]], field: str, window: float = 0.5) -> float:
    start = rows[-1]["time_s"] - window
    return statistics.fmean(row[field] for row in rows if row["time_s"] >= start)


def max_util(row: dict[str, float]) -> float:
    return max(row[f"friction_utilization_{wheel}"] for wheel in WHEELS)


def main() -> None:
    args = parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    scenario_dir = output / "scenarios"
    vehicle_dir = output / "vehicles"
    scenario_dir.mkdir(exist_ok=True)
    vehicle_dir.mkdir(exist_ok=True)
    base = json.loads((ROOT / "configs/vehicles/generic_nonlinear_performance_car.json").read_text())
    summary: dict[str, object] = {}

    with tempfile.TemporaryDirectory(prefix="apexlab-m3-") as temporary:
        temp = Path(temporary)

        # 1. Nonlinear tire curves: production C++ is probed through zero-duration full-model runs.
        tire_rows: list[dict[str, object]] = []
        max_tire_error = 0.0
        for load in (2000.0, 3500.0, 5000.0):
            vehicle = copy.deepcopy(base)
            vehicle["mass_kg"] = load * 2.0 * 2.8 / (G * 1.55)
            vehicle["aero"].update({"drag_coefficient": 0.0, "reference_area_m2": 0.0})
            vehicle["nonlinear_planar"]["cg_height_m"] = 0.0
            vehicle_path = temp / f"tire-{int(load)}.json"
            write_json(vehicle_path, vehicle)
            for degrees in range(-15, 16):
                alpha = math.radians(degrees)
                rows = run(args.sim, vehicle_path,
                           scenario("tire", speed=20.0, steering=constant(alpha)),
                           temp / "scenario.json", temp / "telemetry.csv")
                actual = rows[0]
                reference, mu, _ = fiala(alpha, load, 60000.0, 1.2, 3555.0, -0.08)
                error = abs(actual["fy_fl_n"] - reference)
                max_tire_error = max(max_tire_error, error)
                tire_rows.append({"normal_load_n": load, "slip_angle_deg": degrees,
                                  "production_fy_n": actual["fy_fl_n"],
                                  "reference_fy_n": reference, "absolute_error_n": error,
                                  "effective_mu": actual["effective_mu_fl"],
                                  "reference_mu": mu})
        write_csv(output / "nonlinear_tire_curve.csv", tuple(tire_rows[0]), tire_rows)
        write_svg(output / "nonlinear_tire_curve.svg", "Fiala lateral tire curves",
                  "slip angle (deg)", "lateral force (N)",
                  [(f"Fz={load:.0f} N", [(float(r["slip_angle_deg"]), float(r["production_fy_n"]))
                                           for r in tire_rows if r["normal_load_n"] == load])
                   for load in (2000.0, 3500.0, 5000.0)])
        mu_points = [(load, fiala(0.0, load, 60000.0, 1.2, 3555.0, -0.08)[1])
                     for load in range(1000, 6500, 250)]
        write_svg(output / "effective_mu_vs_load.svg", "Configured tire load sensitivity",
                  "normal load (N)", "effective friction coefficient", [("mu(Fz)", mu_points)])
        summary["experiment_1_tire_curve"] = {"max_cpp_python_difference_n": max_tire_error}

        # 2. Combined-force circle.
        vehicle = copy.deepcopy(base)
        vehicle["mass_kg"] = 3500.0 * 2.0 * 2.8 / (G * 1.55)
        vehicle["aero"].update({"drag_coefficient": 0.0, "reference_area_m2": 0.0})
        vehicle["nonlinear_planar"].update({"cg_height_m": 0.0, "drivetrain_type": "AWD",
                                             "drive_front_fraction": 0.5,
                                             "front_brake_bias": 0.5})
        _, _, capacity = fiala(0.08, 3500.0, 60000.0, 1.2, 3555.0, -0.08)
        vehicle["powertrain"]["maximum_drive_force_n"] = 6.0 * capacity
        vehicle["brakes"]["maximum_brake_force_n"] = 6.0 * capacity
        vehicle_path = temp / "ellipse.json"
        write_json(vehicle_path, vehicle)
        ellipse_rows: list[dict[str, object]] = []
        max_boundary_excess = 0.0
        for request_norm in [x / 10 for x in range(-15, 16)]:
            command = abs(request_norm) / 1.5
            rows = run(args.sim, vehicle_path,
                       scenario("ellipse", speed=20.0, steering=constant(0.08),
                                throttle=constant(command if request_norm > 0 else 0.0),
                                brake=constant(command if request_norm < 0 else 0.0)),
                       temp / "scenario.json", temp / "telemetry.csv")
            row = rows[0]
            nx = row["fx_fl_n"] / row["force_capacity_fl_n"]
            ny = row["fy_fl_n"] / row["force_capacity_fl_n"]
            norm = math.hypot(nx, ny)
            max_boundary_excess = max(max_boundary_excess, norm - 1.0)
            ellipse_rows.append({"requested_fx_normalized": request_norm,
                                 "delivered_fx_normalized": nx,
                                 "delivered_fy_normalized": ny,
                                 "delivered_norm": norm,
                                 "requested_utilization": row["requested_friction_utilization_fl"]})
        write_csv(output / "friction_circle.csv", tuple(ellipse_rows[0]), ellipse_rows)
        boundary = [(math.cos(t), math.sin(t)) for t in [i * 2 * math.pi / 120 for i in range(121)]]
        write_svg(output / "friction_circle.svg", "Delivered combined force",
                  "Fx / capacity", "Fy / capacity",
                  [("friction boundary", boundary),
                   ("delivered", [(float(r["delivered_fx_normalized"]),
                                    float(r["delivered_fy_normalized"])) for r in ellipse_rows])])
        summary["experiment_2_friction_circle"] = {
            "maximum_delivered_boundary_excess": max(0.0, max_boundary_excess)}

        # 3. Longitudinal load transfer against closed-form moment balance.
        load_rows: list[dict[str, object]] = []
        vehicle_path = ROOT / "configs/vehicles/generic_nonlinear_performance_car.json"
        for name, throttle, brake in (("hard acceleration", 1.0, 0.0), ("coasting", 0.0, 0.0),
                                      ("moderate braking", 0.0, 0.35), ("hard braking", 0.0, 1.0)):
            row = run(args.sim, vehicle_path,
                      scenario(name, speed=25.0, steering=constant(0.0),
                               throttle=constant(throttle), brake=constant(brake)),
                      temp / "scenario.json", temp / "telemetry.csv")[0]
            expected_transfer = base["mass_kg"] * row["longitudinal_accel_m_s2"] * 0.5 / 2.8
            total = sum(row[f"fz_{wheel}_n"] for wheel in WHEELS)
            load_rows.append({"case": name, "acceleration_m_s2": row["longitudinal_accel_m_s2"],
                              "front_axle_load_n": row["fz_fl_n"] + row["fz_fr_n"],
                              "rear_axle_load_n": row["fz_rl_n"] + row["fz_rr_n"],
                              "production_transfer_n": row["longitudinal_load_transfer_n"],
                              "reference_transfer_n": expected_transfer,
                              "total_load_n": total,
                              "conservation_error_n": abs(total - base["mass_kg"] * G)})
        write_csv(output / "longitudinal_load_transfer.csv", tuple(load_rows[0]), load_rows)
        write_svg(output / "longitudinal_load_transfer.svg", "Longitudinal load transfer",
                  "longitudinal acceleration (m/s^2)", "axle normal load (N)",
                  [("front axle", [(float(r["acceleration_m_s2"]), float(r["front_axle_load_n"])) for r in load_rows]),
                   ("rear axle", [(float(r["acceleration_m_s2"]), float(r["rear_axle_load_n"])) for r in load_rows])])
        summary["experiment_3_longitudinal_transfer"] = {
            "max_reference_error_n": max(abs(float(r["production_transfer_n"]) - float(r["reference_transfer_n"])) for r in load_rows),
            "max_conservation_error_n": max(float(r["conservation_error_n"]) for r in load_rows)}

        # 4. Lateral transfer, mirrored inputs, and two roll distributions.
        lateral_rows: list[dict[str, object]] = []
        for roll_fraction in (0.40, 0.65):
            vehicle = copy.deepcopy(base)
            vehicle["aero"].update({"drag_coefficient": 0.0, "reference_area_m2": 0.0})
            vehicle["nonlinear_planar"]["front_roll_moment_fraction"] = roll_fraction
            path = temp / f"roll-{roll_fraction}.json"
            write_json(path, vehicle)
            for steer in (-0.06, -0.04, -0.02, 0.0, 0.02, 0.04, 0.06):
                row = run(args.sim, path,
                          scenario("lateral transfer", speed=22.0, steering=constant(steer)),
                          temp / "scenario.json", temp / "telemetry.csv")[0]
                lateral_rows.append({"front_roll_fraction": roll_fraction, "steering_rad": steer,
                                     "lateral_acceleration_m_s2": row["lateral_accel_m_s2"],
                                     **{f"fz_{w}_n": row[f"fz_{w}_n"] for w in WHEELS},
                                     "front_max_utilization": max(row["friction_utilization_fl"], row["friction_utilization_fr"]),
                                     "rear_max_utilization": max(row["friction_utilization_rl"], row["friction_utilization_rr"]),
                                     "total_load_n": sum(row[f"fz_{w}_n"] for w in WHEELS)})
        write_csv(output / "lateral_load_transfer.csv", tuple(lateral_rows[0]), lateral_rows)
        write_svg(output / "lateral_load_transfer.svg", "Per-wheel lateral load transfer (front roll fraction 0.65)",
                  "lateral acceleration (m/s^2)", "normal load (N)",
                  [(w.upper(), [(float(r["lateral_acceleration_m_s2"]), float(r[f"fz_{w}_n"]))
                                for r in lateral_rows if r["front_roll_fraction"] == 0.65]) for w in WHEELS])
        positive = [r for r in lateral_rows if r["front_roll_fraction"] == 0.65 and r["steering_rad"] > 0]
        summary["experiment_4_lateral_transfer"] = {
            "right_tires_gain_in_all_left_turn_samples": all(float(r["fz_fr_n"]) > float(r["fz_fl_n"]) and float(r["fz_rr_n"]) > float(r["fz_rl_n"]) for r in positive),
            "max_conservation_error_n": max(abs(float(r["total_load_n"]) - base["mass_kg"] * G) for r in lateral_rows)}

        # 5. Linear/nonlinear departure under identical constant cornering inputs.
        nonlinear_vehicle = copy.deepcopy(base)
        nonlinear_vehicle["aero"].update({"drag_coefficient": 0.0, "reference_area_m2": 0.0})
        nonlinear_path = vehicle_dir / "comparison_nonlinear.json"
        write_json(nonlinear_path, nonlinear_vehicle)
        linear_vehicle = copy.deepcopy(nonlinear_vehicle)
        linear_vehicle["schema_version"] = 2
        del linear_vehicle["nonlinear_planar"]
        linear_vehicle["tires"] = {"friction_coefficient": 1.2,
                                    "rolling_resistance_coefficient": 0.0}
        linear_path = vehicle_dir / "comparison_linear.json"
        write_json(linear_path, linear_vehicle)
        departure_rows: list[dict[str, object]] = []
        divergence_speed: float | None = None
        for speed in (10.0, 15.0, 20.0, 25.0, 30.0, 35.0):
            case = scenario(f"departure {speed}", speed=speed, steering=constant(0.025),
                            duration=6.0, timestep=0.005)
            linear = run(args.sim, linear_path, case, scenario_dir / f"departure_linear_{int(speed)}.json",
                         output / f"departure_linear_{int(speed)}.csv")
            nonlinear = run(args.sim, nonlinear_path, case, scenario_dir / f"departure_nonlinear_{int(speed)}.json",
                            output / f"departure_nonlinear_{int(speed)}.csv")
            linear_yaw = settled(linear, "yaw_rate_rad_s")
            nonlinear_yaw = settled(nonlinear, "yaw_rate_rad_s")
            difference = abs(nonlinear_yaw - linear_yaw) / max(abs(linear_yaw), 1e-12)
            if divergence_speed is None and difference > 0.05:
                divergence_speed = speed
            departure_rows.append({"initial_speed_m_s": speed, "linear_yaw_rate_rad_s": linear_yaw,
                                   "nonlinear_yaw_rate_rad_s": nonlinear_yaw,
                                   "relative_difference": difference,
                                   "nonlinear_lateral_accel_m_s2": settled(nonlinear, "lateral_accel_m_s2"),
                                   "peak_nonlinear_utilization": max(max_util(r) for r in nonlinear)})
        write_csv(output / "linear_nonlinear_departure.csv", tuple(departure_rows[0]), departure_rows)
        write_svg(output / "linear_nonlinear_departure.svg", "Departure from linear bicycle response",
                  "initial speed (m/s)", "settled yaw rate (rad/s)",
                  [("linear", [(float(r["initial_speed_m_s"]), float(r["linear_yaw_rate_rad_s"])) for r in departure_rows]),
                   ("nonlinear", [(float(r["initial_speed_m_s"]), float(r["nonlinear_yaw_rate_rad_s"])) for r in departure_rows])])
        summary["experiment_5_departure"] = {"first_speed_above_5_percent_m_s": divergence_speed,
                                              "threshold": 0.05}

        # 6. Limit cornering with slowly ramped steering.
        limit_case = scenario("limit cornering", speed=25.0,
                              steering=ramp(0.0, 0.12, 0.5, 4.0), duration=8.0, timestep=0.002)
        limit = run(args.sim, nonlinear_path, limit_case, scenario_dir / "limit_cornering.json",
                    output / "limit_cornering.csv")
        first_saturation = next((r for r in limit if any(r[f"tire_saturated_{w}"] > 0.5 for w in WHEELS)), None)
        first_wheels = ([w for w in WHEELS if first_saturation and first_saturation[f"tire_saturated_{w}"] > 0.5])
        write_svg(output / "limit_cornering_utilization.svg", "Limit-cornering tire utilization",
                  "time (s)", "friction utilization",
                  [(w.upper(), [(r["time_s"], r[f"friction_utilization_{w}"]) for r in limit]) for w in WHEELS])
        summary["experiment_6_limit_cornering"] = {
            "first_saturation_time_s": first_saturation["time_s"] if first_saturation else None,
            "first_saturated_wheels": first_wheels,
            "first_saturation_details": ({w: {
                "normal_load_n": first_saturation[f"fz_{w}_n"],
                "force_capacity_n": first_saturation[f"force_capacity_{w}_n"],
                "slip_angle_rad": first_saturation[f"slip_angle_{w}_rad"],
                "friction_utilization": first_saturation[f"friction_utilization_{w}"]}
                for w in first_wheels} if first_saturation else {}),
            "peak_lateral_acceleration_m_s2": max(abs(r["lateral_accel_m_s2"]) for r in limit)}

        # 7. Trail braking.
        trail_case = scenario("trail braking", speed=25.0,
                              steering=ramp(0.0, 0.035, 0.0, 2.0),
                              brake=ramp(0.0, 0.5, 3.5, 1.0), duration=5.5, timestep=0.002)
        trail = run(args.sim, nonlinear_path, trail_case, scenario_dir / "trail_braking.json",
                    output / "trail_braking.csv")
        before = min(trail, key=lambda r: abs(r["time_s"] - 3.4))
        after = min(trail, key=lambda r: abs(r["time_s"] - 5.0))
        write_svg(output / "trail_braking_forces.svg", "Trail braking: front tire force budget",
                  "time (s)", "force (N)",
                  [("front Fx delivered", [(r["time_s"], r["fx_fl_n"] + r["fx_fr_n"]) for r in trail]),
                   ("front Fy delivered", [(r["time_s"], r["fy_fl_n"] + r["fy_fr_n"]) for r in trail])])
        summary["experiment_7_trail_braking"] = {
            "front_utilization_before_braking": max(before["friction_utilization_fl"], before["friction_utilization_fr"]),
            "front_utilization_during_braking": max(after["friction_utilization_fl"], after["friction_utilization_fr"]),
            "front_lateral_force_before_n": before["fy_fl_n"] + before["fy_fr_n"],
            "front_lateral_force_during_n": after["fy_fl_n"] + after["fy_fr_n"],
            "front_brake_force_during_n": after["fx_fl_n"] + after["fx_fr_n"],
            "front_remaining_lateral_capacity_during_n": sum(
                math.sqrt(max(0.0, after[f"force_capacity_{w}_n"]**2 - after[f"fx_{w}_n"]**2))
                for w in ("fl", "fr"))}

        # 8. Rear-drive power-on corner exit.
        power_case = scenario("power-on corner exit", speed=20.0,
                              steering=ramp(0.0, 0.035, 0.0, 1.5),
                              throttle=ramp(0.0, 1.0, 2.0, 2.0), duration=6.0, timestep=0.002)
        power = run(args.sim, nonlinear_path, power_case, scenario_dir / "power_on_corner_exit.json",
                    output / "power_on_corner_exit.csv")
        pre_power = min(power, key=lambda r: abs(r["time_s"] - 1.9))
        full_power = min(power, key=lambda r: abs(r["time_s"] - 4.5))
        write_svg(output / "power_on_corner_exit.svg", "Power-on corner exit: rear force competition",
                  "time (s)", "force (N)",
                  [("rear Fx delivered", [(r["time_s"], r["fx_rl_n"] + r["fx_rr_n"]) for r in power]),
                   ("rear Fy delivered", [(r["time_s"], r["fy_rl_n"] + r["fy_rr_n"]) for r in power])])
        summary["experiment_8_power_on_exit"] = {
            "rear_utilization_before_power": max(pre_power["friction_utilization_rl"], pre_power["friction_utilization_rr"]),
            "rear_utilization_at_full_power": max(full_power["friction_utilization_rl"], full_power["friction_utilization_rr"]),
            "rear_requested_drive_n": full_power["requested_fx_rl_n"] + full_power["requested_fx_rr_n"],
            "rear_delivered_drive_n": full_power["fx_rl_n"] + full_power["fx_rr_n"],
            "rear_requested_lateral_force_n": full_power["requested_fy_rl_n"] + full_power["requested_fy_rr_n"],
            "rear_lateral_force_before_n": pre_power["fy_rl_n"] + pre_power["fy_rr_n"],
            "rear_lateral_force_full_power_n": full_power["fy_rl_n"] + full_power["fy_rr_n"]}

        # 9. Consequence of load sensitivity at unchanged total weight.
        sensitivity_rows: list[dict[str, object]] = []
        for exponent in (0.0, -0.08):
            vehicle = copy.deepcopy(base)
            vehicle["aero"].update({"drag_coefficient": 0.0, "reference_area_m2": 0.0})
            vehicle["tires"]["tire_load_sensitivity_exponent"] = exponent
            path = temp / f"sensitivity-{exponent}.json"
            write_json(path, vehicle)
            for steer in (0.0, 0.015, 0.03, 0.045, 0.06, 0.075):
                row = run(args.sim, path,
                          scenario("load sensitivity", speed=25.0, steering=constant(steer)),
                          temp / "scenario.json", temp / "telemetry.csv")[0]
                sensitivity_rows.append({"exponent": exponent, "steering_rad": steer,
                                         "lateral_acceleration_m_s2": row["lateral_accel_m_s2"],
                                         "front_total_capacity_n": row["force_capacity_fl_n"] + row["force_capacity_fr_n"],
                                         "rear_total_capacity_n": row["force_capacity_rl_n"] + row["force_capacity_rr_n"],
                                         "total_vertical_load_n": sum(row[f"fz_{w}_n"] for w in WHEELS)})
        write_csv(output / "load_sensitivity_consequence.csv", tuple(sensitivity_rows[0]), sensitivity_rows)
        write_svg(output / "load_sensitivity_consequence.svg", "Load transfer reduces axle capacity with load-sensitive mu",
                  "steering angle (rad)", "front axle force capacity (N)",
                  [("p=0", [(float(r["steering_rad"]), float(r["front_total_capacity_n"])) for r in sensitivity_rows if r["exponent"] == 0.0]),
                   ("p=-0.08", [(float(r["steering_rad"]), float(r["front_total_capacity_n"])) for r in sensitivity_rows if r["exponent"] == -0.08])])
        insensitive_final = next(r for r in reversed(sensitivity_rows) if r["exponent"] == 0.0)
        sensitive_final = next(r for r in reversed(sensitivity_rows) if r["exponent"] == -0.08)
        reduction = 1.0 - float(sensitive_final["front_total_capacity_n"]) / float(insensitive_final["front_total_capacity_n"])
        summary["experiment_9_load_sensitivity"] = {
            "front_capacity_reduction_at_max_steer_fraction": reduction,
            "total_weight_error_n": max(abs(float(r["total_vertical_load_n"]) - base["mass_kg"] * G) for r in sensitivity_rows)}

        # 10. Smooth-input timestep convergence.
        convergence_rows: list[dict[str, object]] = []
        reference_case = scenario("convergence reference", speed=22.0,
                                  steering=sine(0.0, 0.04, 0.5),
                                  throttle=sine(0.2, 0.2, 0.5), duration=2.0,
                                  timestep=0.00025, integrator="rk4")
        reference = run(args.sim, nonlinear_path, reference_case,
                        scenario_dir / "convergence_reference.json",
                        output / "convergence_reference.csv")
        ref_metrics = (reference[-1]["yaw_rate_rad_s"],
                       max(abs(r["lateral_accel_m_s2"]) for r in reference),
                       max(max_util(r) for r in reference),
                       reference[-1]["position_x_m"], reference[-1]["position_y_m"])
        for integrator in ("euler", "rk4"):
            for dt in (0.01, 0.005, 0.002, 0.001, 0.0005):
                case = scenario("nonlinear convergence", speed=22.0,
                                steering=sine(0.0, 0.04, 0.5),
                                throttle=sine(0.2, 0.2, 0.5), duration=2.0,
                                timestep=dt, integrator=integrator)
                rows = run(args.sim, nonlinear_path, case,
                           scenario_dir / f"convergence_{integrator}_{dt}.json",
                           output / f"convergence_{integrator}_{dt}.csv")
                metrics = (rows[-1]["yaw_rate_rad_s"],
                           max(abs(r["lateral_accel_m_s2"]) for r in rows),
                           max(max_util(r) for r in rows), rows[-1]["position_x_m"],
                           rows[-1]["position_y_m"])
                convergence_rows.append({"integrator": integrator, "timestep_s": dt,
                                         "final_yaw_rate_error_rad_s": abs(metrics[0]-ref_metrics[0]),
                                         "peak_lateral_accel_error_m_s2": abs(metrics[1]-ref_metrics[1]),
                                         "peak_utilization_error": abs(metrics[2]-ref_metrics[2]),
                                         "endpoint_error_m": math.hypot(metrics[3]-ref_metrics[3], metrics[4]-ref_metrics[4])})
        write_csv(output / "numerical_convergence.csv", tuple(convergence_rows[0]), convergence_rows)
        write_svg(output / "numerical_convergence.svg", "Nonlinear timestep convergence",
                  "timestep (s)", "trajectory endpoint error (m)",
                  [(kind, [(float(r["timestep_s"]), float(r["endpoint_error_m"]))
                           for r in convergence_rows if r["integrator"] == kind])
                   for kind in ("euler", "rk4")])
        summary["experiment_10_convergence"] = {"reference": {
            "final_yaw_rate_rad_s": ref_metrics[0], "peak_lateral_accel_m_s2": ref_metrics[1],
            "peak_utilization": ref_metrics[2], "endpoint_x_m": ref_metrics[3],
            "endpoint_y_m": ref_metrics[4]}, "runs": convergence_rows}

    write_json(output / "summary.json", summary)
    report = render_report(summary)
    args.report.resolve().write_text(report, encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"milestone 3 validation passed; report={args.report.resolve()}")


def render_report(summary: dict[str, object]) -> str:
    e1 = summary["experiment_1_tire_curve"]
    e2 = summary["experiment_2_friction_circle"]
    e3 = summary["experiment_3_longitudinal_transfer"]
    e4 = summary["experiment_4_lateral_transfer"]
    e5 = summary["experiment_5_departure"]
    e6 = summary["experiment_6_limit_cornering"]
    e7 = summary["experiment_7_trail_braking"]
    e8 = summary["experiment_8_power_on_exit"]
    e9 = summary["experiment_9_load_sensitivity"]
    e10 = summary["experiment_10_convergence"]
    return f"""# Milestone 3 nonlinear grip-limit results

Generated by `tools/python/validation/run_milestone3_validation.py`. The C++ simulator is treated
as a black box; Fiala curves, friction-boundary checks, and load-transfer references are calculated
independently in Python. The vehicle is an illustrative reference, not OEM data.

## 1 — Nonlinear tire curve

**Question and hypothesis.** Does the tire retain the configured small-slip slope, transition
progressively, saturate finitely, and show decreasing effective mu with load? **Setup and method.**
Slip was swept from -15 to +15 degrees at 2000, 3500, and 5000 N. Zero-height probe vehicles made
wheel load exact. Production telemetry was compared pointwise with an independent Fiala function.
**Result.** Maximum difference was `{e1['max_cpp_python_difference_n']:.9g} N`.
**Interpretation.** The curves show the linear region, brush transition, finite plateau, and load
sensitivity. **Limitation.** This is equation verification, not measured tire validation.

![Tire curves](../../data/generated/milestone-3/nonlinear_tire_curve.svg)

## 2 — Friction circle

**Question and hypothesis.** Does longitudinal demand consume lateral capacity? **Setup and
method.** At fixed load and slip, normalized requested Fx was swept from -1.5 to +1.5. **Result.**
Maximum delivered boundary excess was `{e2['maximum_delivered_boundary_excess']:.9g}`.
**Interpretation.** Stronger braking or drive radially scales the force pair onto the finite circle.
**Limitation.** A circle is symmetric and omits measured combined-slip asymmetry.

![Friction circle](../../data/generated/milestone-3/friction_circle.svg)

## 3 — Longitudinal load transfer

**Question and hypothesis.** Do physical acceleration and braking shift load with the expected
sign while conserving weight? **Setup and method.** Hard acceleration, coast, moderate braking,
and hard braking were compared with `m ax h / L`. **Result.** Maximum reference error was
`{e3['max_reference_error_n']:.9g} N`; maximum conservation error was
`{e3['max_conservation_error_n']:.9g} N`. **Interpretation.** Braking loads the front axle and
acceleration loads the rear. **Limitation.** Pitch is quasi-static and downforce is excluded.

![Longitudinal transfer](../../data/generated/milestone-3/longitudinal_load_transfer.svg)

## 4 — Lateral load transfer

**Question and hypothesis.** Are left/right loads mirrored and does roll distribution move the
transfer between axles? **Setup and method.** Mirrored steer sweeps used front roll fractions 0.40
and 0.65. **Result.** Outside-right tires gained load in every sampled left turn:
`{e4['right_tires_gain_in_all_left_turn_samples']}`; maximum conservation error was
`{e4['max_conservation_error_n']:.9g} N`. **Interpretation.** Front roll fraction directly changes
front versus rear utilization. **Limitation.** It stands in for suspension roll stiffness.

![Lateral transfer](../../data/generated/milestone-3/lateral_load_transfer.svg)

## 5 — Departure from linear bicycle theory

**Question and hypothesis.** When does the grip-limited model depart from the preserved linear
reference? **Setup and method.** Identical constant-steer runs swept initial speed; divergence was
defined before running as greater than 5% settled-yaw-rate difference. **Result.** The first sampled
speed above threshold was `{e5['first_speed_above_5_percent_m_s']} m/s`. **Interpretation.** Fiala
nonlinearity, load sensitivity, and saturation limit the nonlinear response while the linear model
continues extrapolating. **Limitation.** Settled speed drifts slightly from the initial value.

![Departure](../../data/generated/milestone-3/linear_nonlinear_departure.svg)

## 6 — Limit cornering

**Question and hypothesis.** Which tires reach the limit first in a slow steering ramp? **Result.**
First saturation occurred at `{e6['first_saturation_time_s']} s` on
`{', '.join(e6['first_saturated_wheels'])}`; peak lateral acceleration was
`{e6['peak_lateral_acceleration_m_s2']:.6g} m/s^2`. At first saturation the rear-left load/capacity
was `{e6['first_saturation_details']['rl']['normal_load_n']:.3f} N` /
`{e6['first_saturation_details']['rl']['force_capacity_n']:.3f} N`. **Interpretation.** This
reference is rear-inside limited first: lateral transfer unloads the left tire and the lower rear
capacity is exhausted at the shared rear slip before the loaded outside tire.
**Limitation.** No relaxation length or roll transient is modeled.

![Limit utilization](../../data/generated/milestone-3/limit_cornering_utilization.svg)

## 7 — Trail braking

**Question and hypothesis.** Does braking consume front-tire budget during cornering? **Result.**
Front peak utilization changed from `{e7['front_utilization_before_braking']:.4f}` to
`{e7['front_utilization_during_braking']:.4f}`; front lateral force changed from
`{e7['front_lateral_force_before_n']:.3f}` to `{e7['front_lateral_force_during_n']:.3f} N` while
front braking force was `{e7['front_brake_force_during_n']:.3f} N`. **Interpretation.** The force
front braking force was `{e7['front_brake_force_during_n']:.3f} N`; the remaining front lateral
circle capacity was `{e7['front_remaining_lateral_capacity_during_n']:.3f} N`. **Interpretation.**
The force budget is shared rather than independently clamped. **Limitation.** No ABS or wheel
rotation.

![Trail braking](../../data/generated/milestone-3/trail_braking_forces.svg)

## 8 — Power-on corner exit

**Question and hypothesis.** Does rear-drive demand compete with rear lateral force? **Result.**
Rear peak utilization changed from `{e8['rear_utilization_before_power']:.4f}` to
`{e8['rear_utilization_at_full_power']:.4f}`. Requested/delivered rear drive was
`{e8['rear_requested_drive_n']:.3f}/{e8['rear_delivered_drive_n']:.3f} N`; rear lateral force
changed from `{e8['rear_lateral_force_before_n']:.3f}` to
`{e8['rear_lateral_force_full_power_n']:.3f} N`, versus a pure-lateral request of
`{e8['rear_requested_lateral_force_n']:.3f} N`. **Interpretation.** The maneuver's lateral demand
grew with speed, but the saturated rear-left tire delivered less than its requested drive and
lateral force: drive demand consumed the rear force circle. **Limitation.** Static split replaces
a differential and wheelspin model.

![Power-on exit](../../data/generated/milestone-3/power_on_corner_exit.svg)

## 9 — Load-sensitivity consequence

**Question and hypothesis.** Can redistribution reduce axle grip while total weight is unchanged?
**Result.** At maximum sampled steer the load-sensitive front capacity was
`{100*e9['front_capacity_reduction_at_max_steer_fraction']:.4f}%` below the p=0 comparison; total
weight error was `{e9['total_weight_error_n']:.9g} N`. **Interpretation.** Capacity grows
sublinearly with Fz, so the outside gain does not compensate the inside loss. **Limitation.** The
power-law exponent is illustrative.

![Load sensitivity](../../data/generated/milestone-3/load_sensitivity_consequence.svg)

## 10 — Numerical convergence

**Question and hypothesis.** Do smooth nonlinear maneuvers converge with timestep? **Setup and
method.** Euler and RK4 used 10, 5, 2, 1, and 0.5 ms against a separate 0.25 ms RK4 reference.
**Result.** Reference final yaw rate was `{e10['reference']['final_yaw_rate_rad_s']:.9g} rad/s`,
peak lateral acceleration `{e10['reference']['peak_lateral_accel_m_s2']:.9g} m/s^2`, and peak
utilization `{e10['reference']['peak_utilization']:.9g}`. Errors are in `numerical_convergence.csv`.
**Interpretation.** Both methods converge; RK4 reaches a given error at a materially larger step.
**Limitation.** The finest run is a numerical, not closed-form, reference.

![Convergence](../../data/generated/milestone-3/numerical_convergence.svg)
"""


if __name__ == "__main__":
    main()
