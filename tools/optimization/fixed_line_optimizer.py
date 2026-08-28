#!/usr/bin/env python3
"""Fixed-reference-line minimum-time transcription using CasADi and IPOPT.

The optimizer is intentionally separate from the C++ time-domain simulator.  Its
quasi-steady spatial model mirrors the production four-contact load transfer,
nonlinear lateral tire law, friction capacity, drive/brake split, drag and
rolling resistance.  Optimized output must still pass the independent replay
gate before it is treated as validated.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path

import casadi as ca
import numpy as np


G = 9.80665


def read_csv(path: Path) -> list[dict[str, float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return [{key: float(value) for key, value in row.items()} for row in csv.DictReader(stream)]


def periodic_interpolate(rows: list[dict[str, float]], field: str, query: np.ndarray,
                         length: float, default: float | None = None) -> np.ndarray:
    ordered = sorted(rows, key=lambda row: row["s_m"])
    source_s = np.asarray([row["s_m"] for row in ordered])
    source_v = np.asarray([row.get(field, default) for row in ordered])
    if np.any([value is None for value in source_v]):
        raise KeyError(field)
    keep = np.concatenate(([True], np.diff(source_s) > 1.0e-8))
    source_s, source_v = source_s[keep], source_v[keep]
    extended_s = np.concatenate((source_s - length, source_s, source_s + length))
    extended_v = np.concatenate((source_v, source_v, source_v))
    return np.interp(query, extended_s, extended_v)


def tire_lateral_force(alpha, capacity, stiffness):
    tangent = ca.tan(alpha)
    absolute = ca.fabs(tangent)
    saturation = 3.0 * capacity / stiffness
    unsaturated = (
        stiffness * tangent
        - stiffness * stiffness * absolute * tangent / (3.0 * capacity)
        + stiffness**3 * tangent**3 / (27.0 * capacity**2)
    )
    return ca.if_else(absolute < saturation, unsaturated, ca.sign(tangent) * capacity)


def solve(args: argparse.Namespace) -> dict:
    session_dir = args.session.resolve()
    manifest = json.loads((session_dir / "session.json").read_text(encoding="utf-8"))
    vehicle = json.loads(args.vehicle.read_text(encoding="utf-8"))
    geometry = read_csv(session_dir / manifest["files"]["geometry"])
    spatial = read_csv(session_dir / manifest["files"]["spatial"])
    lap_number = manifest["laps"][0]["number"]
    spatial = [row for row in spatial if int(row["lap_number"]) == lap_number]
    length = float(manifest["track"]["length_m"])
    nodes = args.nodes
    ds = length / nodes
    s_nodes = np.arange(nodes, dtype=float) * ds
    curvature = periodic_interpolate(geometry, "curvature_1_m", s_nodes, length)
    grade = periodic_interpolate(geometry, "grade", s_nodes, length, 0.0)
    center_x = periodic_interpolate(geometry, "x_m", s_nodes, length)
    center_y = periodic_interpolate(geometry, "y_m", s_nodes, length)
    center_heading = periodic_interpolate(geometry, "heading_rad", s_nodes, length)

    mass = vehicle["mass_kg"]
    aero = vehicle["aero"]
    tires = vehicle["tires"]
    planar = vehicle["planar"]
    nonlinear = vehicle["nonlinear_planar"]
    maximum_drive = vehicle["powertrain"]["maximum_drive_force_n"]
    maximum_brake = vehicle["brakes"]["maximum_brake_force_n"]
    wheelbase = planar["wheelbase_m"]
    front_x = planar["cg_to_front_axle_m"]
    rear_x = planar["cg_to_rear_axle_m"]
    front_track = nonlinear["front_track_m"]
    rear_track = nonlinear["rear_track_m"]
    cg_height = nonlinear["cg_height_m"]
    roll_fraction = nonlinear["front_roll_moment_fraction"]
    drive_front = nonlinear["drive_front_fraction"]
    brake_front = nonlinear["front_brake_bias"]
    rho = aero["air_density_kgpm3"]
    drag_area = aero["drag_coefficient"] * aero["reference_area_m2"]
    rolling = tires["rolling_resistance_coefficient"] * mass * G
    mu_reference = tires["tire_mu_reference"]
    reference_load = tires["tire_reference_load_n"]
    load_exponent = tires["tire_load_sensitivity_exponent"]
    front_stiffness = planar["front_cornering_stiffness_n_per_rad"] / 2.0
    rear_stiffness = planar["rear_cornering_stiffness_n_per_rad"] / 2.0

    opti = ca.Opti()
    speed_squared = opti.variable(nodes)
    elapsed = opti.variable(nodes + 1)
    command = opti.variable(nodes)
    steering = opti.variable(nodes)
    sideslip = opti.variable(nodes)
    longitudinal_acceleration = opti.variable(nodes)

    opti.subject_to(opti.bounded(args.minimum_speed**2, speed_squared, args.maximum_speed**2))
    opti.subject_to(opti.bounded(-1.0, command, 1.0))
    opti.subject_to(opti.bounded(-args.maximum_steering, steering, args.maximum_steering))
    opti.subject_to(opti.bounded(-0.35, sideslip, 0.35))
    opti.subject_to(opti.bounded(-15.0, longitudinal_acceleration, 8.0))
    opti.subject_to(elapsed[0] == 0.0)
    opti.subject_to(opti.bounded(0.0, elapsed, 10000.0))

    body_x_values = []
    lateral_residuals = []
    yaw_residuals = []
    normal_loads = []
    utilizations = []
    throttle_values = []
    brake_values = []
    regularization_epsilon = 1.0e-8
    for index in range(nodes):
        z = speed_squared[index]
        velocity = ca.sqrt(z)
        yaw_rate = velocity * curvature[index]
        lateral_acceleration = z * curvature[index]
        smooth_absolute = ca.sqrt(command[index] ** 2 + regularization_epsilon)
        throttle = 0.5 * (command[index] + smooth_absolute)
        brake = 0.5 * (-command[index] + smooth_absolute)
        throttle_values.append(throttle)
        brake_values.append(brake)
        drive_force = throttle * maximum_drive
        brake_force = brake * maximum_brake
        front_fx = 0.5 * (drive_force * drive_front - brake_force * brake_front)
        rear_fx = 0.5 * (drive_force * (1.0 - drive_front) -
                         brake_force * (1.0 - brake_front))

        gravity_load = mass * G
        front_static = gravity_load * rear_x / wheelbase
        rear_static = gravity_load * front_x / wheelbase
        longitudinal_transfer = mass * longitudinal_acceleration[index] * cg_height / wheelbase
        front_axle = front_static - longitudinal_transfer
        rear_axle = rear_static + longitudinal_transfer
        roll_moment = mass * lateral_acceleration * cg_height
        front_transfer = roll_fraction * roll_moment / front_track
        rear_transfer = (1.0 - roll_fraction) * roll_moment / rear_track
        loads = [
            0.5 * front_axle - front_transfer,
            0.5 * front_axle + front_transfer,
            0.5 * rear_axle - rear_transfer,
            0.5 * rear_axle + rear_transfer,
        ]
        contacts = [
            (front_x, front_track / 2.0, steering[index], front_fx, front_stiffness),
            (front_x, -front_track / 2.0, steering[index], front_fx, front_stiffness),
            (-rear_x, rear_track / 2.0, 0.0, rear_fx, rear_stiffness),
            (-rear_x, -rear_track / 2.0, 0.0, rear_fx, rear_stiffness),
        ]
        body_x = -0.5 * rho * drag_area * z - rolling
        body_y = 0.0
        yaw_moment = 0.0
        for load, (contact_x, contact_y, delta, requested_fx, stiffness) in zip(loads, contacts):
            opti.subject_to(load >= 50.0)
            # IPOPT evaluates infeasible trial points while restoring the load
            # constraint. Guard the fractional power outside its domain; this
            # is exactly the production formula for every feasible load >= 50 N.
            load_for_friction = ca.fmax(load, 1.0)
            mu = mu_reference * (load_for_friction / reference_load) ** load_exponent
            capacity = mu * load
            contact_vx = velocity - yaw_rate * contact_y
            contact_vy = sideslip[index] * velocity + yaw_rate * contact_x
            cosine, sine = ca.cos(delta), ca.sin(delta)
            wheel_vx = cosine * contact_vx + sine * contact_vy
            wheel_vy = -sine * contact_vx + cosine * contact_vy
            alpha = -ca.atan2(wheel_vy, wheel_vx)
            lateral_force = tire_lateral_force(alpha, capacity, stiffness)
            utilization = ca.sqrt(requested_fx**2 + lateral_force**2) / capacity
            opti.subject_to(utilization <= 1.0)
            wheel_body_x = cosine * requested_fx - sine * lateral_force
            wheel_body_y = sine * requested_fx + cosine * lateral_force
            body_x += wheel_body_x
            body_y += wheel_body_y
            yaw_moment += contact_x * wheel_body_y - contact_y * wheel_body_x
            normal_loads.append(load)
            utilizations.append(utilization)
        body_x_values.append(body_x)
        lateral_residual = body_y / mass - lateral_acceleration
        yaw_residual = yaw_moment / (mass * G * wheelbase)
        lateral_residuals.append(lateral_residual)
        yaw_residuals.append(yaw_residual)
        opti.subject_to((longitudinal_acceleration[index] - body_x / mass) / G == 0.0)
        opti.subject_to(lateral_residual / G == 0.0)
        opti.subject_to(yaw_residual == 0.0)

    speed_defects = []
    time_defects = []
    for index in range(nodes):
        following = (index + 1) % nodes
        speed_defect = (
            speed_squared[following]
            - speed_squared[index]
            - ds * (longitudinal_acceleration[index] + longitudinal_acceleration[following])
        )
        time_defect = (
            elapsed[index + 1]
            - elapsed[index]
            - 0.5 * ds *
            (1.0 / ca.sqrt(speed_squared[index]) + 1.0 / ca.sqrt(speed_squared[following]))
        )
        speed_defects.append(speed_defect)
        time_defects.append(time_defect)
        opti.subject_to(speed_defect / 100.0 == 0.0)
        opti.subject_to(time_defect == 0.0)

    cyclic_control_change = ca.vertcat(
        *[command[(index + 1) % nodes] - command[index] for index in range(nodes)]
    )
    opti.minimize(elapsed[-1] + 1.0e-7 * ca.sumsqr(cyclic_control_change))

    warm_rows = read_csv(args.warm_start_trajectory) if args.warm_start_trajectory else spatial
    speed_guess = periodic_interpolate(warm_rows, "speed_m_s", s_nodes, length)
    steering_guess = periodic_interpolate(warm_rows, "steering_angle_rad", s_nodes, length)
    throttle_guess = periodic_interpolate(warm_rows, "throttle", s_nodes, length)
    brake_guess = periodic_interpolate(warm_rows, "brake", s_nodes, length)
    command_guess = np.clip(throttle_guess - brake_guess, -1.0, 1.0)
    z_guess = np.maximum(args.minimum_speed, speed_guess) ** 2
    acceleration_guess = np.empty(nodes)
    for index in range(nodes):
        previous = (index - 1) % nodes
        following = (index + 1) % nodes
        acceleration_guess[index] = (z_guess[following] - z_guess[previous]) / (4.0 * ds)
    time_guess = np.zeros(nodes + 1)
    for index in range(nodes):
        following = (index + 1) % nodes
        time_guess[index + 1] = time_guess[index] + 0.5 * ds * (
            1.0 / math.sqrt(z_guess[index]) + 1.0 / math.sqrt(z_guess[following])
        )
    opti.set_initial(speed_squared, z_guess)
    opti.set_initial(elapsed, time_guess)
    opti.set_initial(command, command_guess)
    opti.set_initial(steering, np.clip(steering_guess, -args.maximum_steering,
                                      args.maximum_steering))
    if args.warm_start_trajectory:
        opti.set_initial(sideslip, periodic_interpolate(
            warm_rows, "sideslip_rad", s_nodes, length))
        opti.set_initial(longitudinal_acceleration, np.clip(periodic_interpolate(
            warm_rows, "longitudinal_accel_m_s2", s_nodes, length), -10.0, 5.0))
    else:
        opti.set_initial(sideslip, 0.0)
        opti.set_initial(longitudinal_acceleration, np.clip(acceleration_guess, -10.0, 5.0))

    opti.solver(
        "ipopt",
        {"expand": True, "print_time": False},
        {
            "max_iter": args.maximum_iterations,
            "tol": args.tolerance,
            "acceptable_tol": max(args.tolerance * 10.0, 1.0e-6),
            "print_level": args.print_level,
            "sb": "yes",
        },
    )
    started = time.perf_counter()
    solution = opti.solve()
    wall_time = time.perf_counter() - started
    stats = opti.stats()

    def value(expression) -> np.ndarray:
        return np.asarray(solution.value(expression), dtype=float).reshape(-1)

    z_result = value(speed_squared)
    time_result = value(elapsed)
    command_result = value(command)
    steering_result = value(steering)
    sideslip_result = value(sideslip)
    ax_result = value(longitudinal_acceleration)
    throttle_result = value(ca.vertcat(*throttle_values))
    brake_result = value(ca.vertcat(*brake_values))
    utilization_result = value(ca.vertcat(*utilizations)).reshape(nodes, 4)
    load_result = value(ca.vertcat(*normal_loads)).reshape(nodes, 4)
    speed_defect_result = value(ca.vertcat(*speed_defects))
    time_defect_result = value(ca.vertcat(*time_defects))
    lateral_result = value(ca.vertcat(*lateral_residuals))
    yaw_result = value(ca.vertcat(*yaw_residuals))

    args.output.mkdir(parents=True, exist_ok=True)
    trajectory_path = args.output / f"fixed-line-n{nodes}.csv"
    with trajectory_path.open("w", newline="", encoding="utf-8") as stream:
        fields = [
            "s_m", "time_s", "speed_m_s", "steering_angle_rad", "sideslip_rad",
            "longitudinal_accel_m_s2", "lateral_accel_m_s2", "throttle", "brake",
            "curvature_1_m", "grade", "lateral_offset_m", "utilization_fl",
            "path_heading_rad", "path_curvature_1_m", "position_x_m", "position_y_m",
            "utilization_fr", "utilization_rl", "utilization_rr", "fz_fl_n", "fz_fr_n",
            "fz_rl_n", "fz_rr_n",
        ]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index in range(nodes):
            writer.writerow({
                "s_m": s_nodes[index], "time_s": time_result[index],
                "speed_m_s": math.sqrt(z_result[index]),
                "steering_angle_rad": steering_result[index],
                "sideslip_rad": sideslip_result[index],
                "longitudinal_accel_m_s2": ax_result[index],
                "lateral_accel_m_s2": z_result[index] * curvature[index],
                "throttle": float(np.clip(throttle_result[index], 0.0, 1.0)),
                "brake": float(np.clip(brake_result[index], 0.0, 1.0)),
                "curvature_1_m": curvature[index], "grade": grade[index],
                "lateral_offset_m": 0.0,
                "path_heading_rad": center_heading[index],
                "path_curvature_1_m": curvature[index],
                "position_x_m": center_x[index], "position_y_m": center_y[index],
                **{f"utilization_{wheel}": utilization_result[index, wheel_index]
                   for wheel_index, wheel in enumerate(("fl", "fr", "rl", "rr"))},
                **{f"fz_{wheel}_n": load_result[index, wheel_index]
                   for wheel_index, wheel in enumerate(("fl", "fr", "rl", "rr"))},
            })

    reference_lap_time = float(manifest["laps"][0]["lap_time_s"])
    diagnostics = {
        "schema_version": 1,
        "problem": "fixed_reference_line_minimum_time",
        "claim": "locally optimized reduced-model solution; independent replay required",
        "track": manifest["track"]["name"],
        "vehicle": vehicle["name"],
        "solver": {"modeling": f"CasADi {ca.__version__}", "nlp": "IPOPT",
                   "status": stats.get("return_status"), "success": bool(stats.get("success")),
                   "iterations": int(stats.get("iter_count", -1)), "wall_time_s": wall_time},
        "mesh": {"nodes": nodes, "spacing_m": ds},
        "size": {"decision_variables": int(opti.x.numel()),
                 "constraints": int(opti.g.numel())},
        "objective": {"predicted_lap_time_s": float(time_result[-1]),
                      "reference_lap_time_s": reference_lap_time,
                      "predicted_delta_s": float(time_result[-1] - reference_lap_time)},
        "residuals": {
            "maximum_speed_dynamic_defect_m2_s2": float(np.max(np.abs(speed_defect_result))),
            "maximum_time_dynamic_defect_s": float(np.max(np.abs(time_defect_result))),
            "maximum_lateral_equilibrium_m_s2": float(np.max(np.abs(lateral_result))),
            "maximum_normalized_yaw_equilibrium": float(np.max(np.abs(yaw_result))),
            "maximum_tire_force_violation": float(max(0.0, np.max(utilization_result) - 1.0)),
            "minimum_normal_load_n": float(np.min(load_result)),
            "maximum_control_violation": float(max(0.0, np.max(np.abs(command_result)) - 1.0)),
            "periodicity_speed_m_s": float(abs(math.sqrt(z_result[0]) -
                                                   math.sqrt(z_result[-1] +
                                                             ds * (ax_result[-1] + ax_result[0])))),
            "maximum_track_violation_m": 0.0,
        },
        "model_notes": [
            "Fixed centerline: lateral_offset_m is identically zero.",
            "Spatial trapezoidal direct collocation uses dt/ds = 1/v.",
            "Tire capacity and nonlinear lateral force mirror the production four-contact formulas.",
            "The fractional load-power guard only affects infeasible IPOPT trial loads below 1 N; accepted contacts are constrained above 50 N.",
            "Quasi-steady sideslip/lateral/yaw equilibrium replaces full transient planar dynamics.",
            "Elevation grade is exported but omitted because the current production planar simulator has no grade force.",
            "A 1e-7 cyclic control smoothness term is numerical regularization; reported lap time excludes it.",
        ],
        "warm_start": {
            "source": str(args.warm_start_trajectory.resolve())
            if args.warm_start_trajectory else str(session_dir),
            "mapped_samples": len(warm_rows),
            "kind": "previous_refinement" if args.warm_start_trajectory else "feasible_reference_lap",
        },
        "trajectory": trajectory_path.name,
    }
    diagnostics_path = args.output / f"fixed-line-n{nodes}.json"
    diagnostics_path.write_text(json.dumps(diagnostics, indent=2) + "\n", encoding="utf-8")
    return diagnostics


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", type=Path, required=True)
    parser.add_argument("--session", type=Path, required=True,
                        help="feasible exported session used for geometry and warm start")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--warm-start-trajectory", type=Path)
    parser.add_argument("--nodes", type=int, default=100)
    parser.add_argument("--minimum-speed", type=float, default=2.0)
    parser.add_argument("--maximum-speed", type=float, default=80.0)
    parser.add_argument("--maximum-steering", type=float, default=0.32)
    parser.add_argument("--maximum-iterations", type=int, default=2000)
    parser.add_argument("--tolerance", type=float, default=1.0e-7)
    parser.add_argument("--print-level", type=int, default=3)
    args = parser.parse_args()
    if args.nodes < 20:
        parser.error("--nodes must be at least 20")
    diagnostics = solve(args)
    print(json.dumps(diagnostics, indent=2))
    return 0 if diagnostics["solver"]["success"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
