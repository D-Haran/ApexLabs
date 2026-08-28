#!/usr/bin/env python3
"""Locally optimize lap time with a bounded lateral racing-line degree of freedom."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path

import casadi as ca
import numpy as np

from fixed_line_optimizer import G, periodic_interpolate, read_csv, tire_lateral_force


def solve(args: argparse.Namespace) -> dict:
    session_dir = args.session.resolve()
    manifest = json.loads((session_dir / "session.json").read_text(encoding="utf-8"))
    vehicle = json.loads(args.vehicle.read_text(encoding="utf-8"))
    geometry = read_csv(session_dir / manifest["files"]["geometry"])
    length = float(manifest["track"]["length_m"])
    nodes = args.nodes
    ds = length / nodes
    s_nodes = np.arange(nodes, dtype=float) * ds
    center_x = periodic_interpolate(geometry, "x_m", s_nodes, length)
    center_y = periodic_interpolate(geometry, "y_m", s_nodes, length)
    center_heading = periodic_interpolate(geometry, "heading_rad", s_nodes, length)
    center_curvature = periodic_interpolate(geometry, "curvature_1_m", s_nodes, length)
    grade = periodic_interpolate(geometry, "grade", s_nodes, length, 0.0)
    left_width = periodic_interpolate(geometry, "left_width_m", s_nodes, length)
    right_width = periodic_interpolate(geometry, "right_width_m", s_nodes, length)
    normal_x = -np.sin(center_heading)
    normal_y = np.cos(center_heading)

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
    lateral_offset = opti.variable(nodes)

    opti.subject_to(opti.bounded(args.minimum_speed**2, speed_squared, args.maximum_speed**2))
    opti.subject_to(opti.bounded(-1.0, command, 1.0))
    opti.subject_to(opti.bounded(-args.maximum_steering, steering, args.maximum_steering))
    opti.subject_to(opti.bounded(-0.35, sideslip, 0.35))
    opti.subject_to(opti.bounded(-15.0, longitudinal_acceleration, 8.0))
    opti.subject_to(elapsed[0] == 0.0)
    opti.subject_to(opti.bounded(0.0, elapsed, 10000.0))
    line_slopes = []
    line_seconds = []
    for index in range(nodes):
        opti.subject_to(opti.bounded(-right_width[index] + args.margin,
                                     lateral_offset[index],
                                     left_width[index] - args.margin))
        following = (index + 1) % nodes
        previous = (index - 1) % nodes
        slope = (lateral_offset[following] - lateral_offset[previous]) / (2.0 * ds)
        second = (lateral_offset[following] - 2.0 * lateral_offset[index] +
                  lateral_offset[previous]) / (ds * ds)
        line_slopes.append(slope)
        line_seconds.append(second)
        opti.subject_to(opti.bounded(-args.maximum_lateral_slope, slope,
                                     args.maximum_lateral_slope))
        opti.subject_to(opti.bounded(-args.maximum_lateral_second_derivative, second,
                                     args.maximum_lateral_second_derivative))

    path_x = [center_x[index] + normal_x[index] * lateral_offset[index]
              for index in range(nodes)]
    path_y = [center_y[index] + normal_y[index] * lateral_offset[index]
              for index in range(nodes)]
    # Frenet geometry is evaluated with respect to centerline arc length.  This
    # is smoother and more accurate than differentiating chords between sparse
    # displaced nodes, and it exactly reduces to the imported centerline when
    # e_y, de_y/ds and d2e_y/ds2 are zero.
    center_curvature_derivative = (
        np.roll(center_curvature, -1) - np.roll(center_curvature, 1)
    ) / (2.0 * ds)
    path_scale = []
    path_headings = []
    path_curvatures = []
    for index in range(nodes):
        tangent_component = 1.0 - center_curvature[index] * lateral_offset[index]
        normal_component = line_slopes[index]
        scale = ca.sqrt(tangent_component**2 + normal_component**2 + 1.0e-12)
        relative_heading_derivative = (
            tangent_component * line_seconds[index]
            + normal_component * (
                center_curvature_derivative[index] * lateral_offset[index]
                + center_curvature[index] * normal_component
            )
        ) / (tangent_component**2 + normal_component**2 + 1.0e-12)
        path_scale.append(scale)
        path_headings.append(center_heading[index] +
                             ca.atan2(normal_component, tangent_component))
        path_curvatures.append((center_curvature[index] + relative_heading_derivative) /
                               scale)
    segment_lengths = [
        0.5 * ds * (path_scale[index] + path_scale[(index + 1) % nodes])
        for index in range(nodes)
    ]

    normal_loads = []
    utilizations = []
    throttle_values = []
    brake_values = []
    lateral_residuals = []
    yaw_residuals = []
    for index in range(nodes):
        z = speed_squared[index]
        velocity = ca.sqrt(z)
        curvature = path_curvatures[index]
        yaw_rate = velocity * curvature
        lateral_acceleration = z * curvature
        smooth_absolute = ca.sqrt(command[index] ** 2 + 1.0e-8)
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
        loads = [0.5 * front_axle - front_transfer, 0.5 * front_axle + front_transfer,
                 0.5 * rear_axle - rear_transfer, 0.5 * rear_axle + rear_transfer]
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
            mu = mu_reference * (load / reference_load) ** load_exponent
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
        distance = segment_lengths[index]
        speed_defect = (speed_squared[following] - speed_squared[index] -
                        distance * (longitudinal_acceleration[index] +
                                    longitudinal_acceleration[following]))
        time_defect = (elapsed[index + 1] - elapsed[index] - 0.5 * distance *
                       (1.0 / ca.sqrt(speed_squared[index]) +
                        1.0 / ca.sqrt(speed_squared[following])))
        speed_defects.append(speed_defect)
        time_defects.append(time_defect)
        opti.subject_to(speed_defect / 100.0 == 0.0)
        opti.subject_to(time_defect == 0.0)

    control_change = ca.vertcat(*[command[(i + 1) % nodes] - command[i]
                                  for i in range(nodes)])
    line_second = ca.vertcat(*[
        lateral_offset[(i + 1) % nodes] - 2.0 * lateral_offset[i] +
        lateral_offset[(i - 1) % nodes] for i in range(nodes)
    ])
    opti.minimize(elapsed[-1] + 1.0e-7 * ca.sumsqr(control_change) +
                  1.0e-8 * ca.sumsqr(line_second))

    warm_rows = read_csv(args.warm_start_trajectory)
    speed_guess = periodic_interpolate(warm_rows, "speed_m_s", s_nodes, length)
    steering_guess = periodic_interpolate(warm_rows, "steering_angle_rad", s_nodes, length)
    throttle_guess = periodic_interpolate(warm_rows, "throttle", s_nodes, length)
    brake_guess = periodic_interpolate(warm_rows, "brake", s_nodes, length)
    sideslip_guess = periodic_interpolate(warm_rows, "sideslip_rad", s_nodes, length)
    ax_guess = periodic_interpolate(warm_rows, "longitudinal_accel_m_s2", s_nodes, length)
    if args.initialization == "warm":
        offset_guess = periodic_interpolate(warm_rows, "lateral_offset_m", s_nodes, length, 0.0)
    elif args.initialization == "center":
        offset_guess = np.zeros(nodes)
    elif args.initialization == "left":
        offset_guess = 0.45 * (left_width - args.margin)
    elif args.initialization == "right":
        offset_guess = -0.45 * (right_width - args.margin)
    else:
        available = np.minimum(left_width, right_width) - args.margin
        offset_guess = -0.35 * available * np.tanh(40.0 * center_curvature)
    z_guess = np.maximum(args.minimum_speed, speed_guess * args.warm_speed_scale) ** 2
    if args.lock_line:
        opti.subject_to(lateral_offset == offset_guess)
    time_guess = np.zeros(nodes + 1)
    for index in range(nodes):
        following = (index + 1) % nodes
        time_guess[index + 1] = time_guess[index] + 0.5 * ds * (
            1.0 / math.sqrt(z_guess[index]) + 1.0 / math.sqrt(z_guess[following]))
    opti.set_initial(speed_squared, z_guess)
    opti.set_initial(elapsed, time_guess)
    opti.set_initial(command, np.clip(throttle_guess - brake_guess, -1.0, 1.0))
    opti.set_initial(steering, np.clip(steering_guess, -args.maximum_steering,
                                      args.maximum_steering))
    opti.set_initial(sideslip, sideslip_guess)
    opti.set_initial(longitudinal_acceleration, np.clip(ax_guess, -10.0, 5.0))
    opti.set_initial(lateral_offset, offset_guess)

    opti.solver("ipopt", {"expand": True, "print_time": False}, {
        "max_iter": args.maximum_iterations, "tol": args.tolerance,
        "acceptable_tol": max(args.tolerance * 10.0, 1.0e-6),
        "print_level": args.print_level, "sb": "yes",
    })
    started = time.perf_counter()
    try:
        solution = opti.solve()
    except RuntimeError:
        constraint_values = np.asarray(opti.debug.value(opti.g), dtype=float).reshape(-1)
        lower_bounds = np.asarray(opti.debug.value(opti.lbg), dtype=float).reshape(-1)
        upper_bounds = np.asarray(opti.debug.value(opti.ubg), dtype=float).reshape(-1)
        violations = np.maximum.reduce((lower_bounds - constraint_values,
                                        constraint_values - upper_bounds,
                                        np.zeros_like(constraint_values)))
        worst = np.argsort(violations)[-12:][::-1]
        ranges = [
            (0, nodes, "speed bound"),
            (nodes, 2 * nodes, "command bound"),
            (2 * nodes, 3 * nodes, "steering bound"),
            (3 * nodes, 4 * nodes, "sideslip bound"),
            (4 * nodes, 5 * nodes, "longitudinal acceleration bound"),
            (5 * nodes, 5 * nodes + 1, "initial elapsed time"),
            (5 * nodes + 1, 6 * nodes + 2, "elapsed-time bound"),
            (6 * nodes + 2, 9 * nodes + 2, "line bound/smoothness"),
            (9 * nodes + 2, 20 * nodes + 2, "contact force/equilibrium"),
            (20 * nodes + 2, 22 * nodes + 2, "spatial dynamics"),
        ]
        if args.lock_line:
            ranges.append((22 * nodes + 2, 23 * nodes + 2, "locked line"))

        def category(index: int) -> str:
            return next((name for start, end, name in ranges if start <= index < end),
                        "unclassified")

        stats = opti.stats()
        failure = {
            "solver_failure": {
                "return_status": stats.get("return_status"),
                "iterations": stats.get("iter_count"),
                "success": stats.get("success"),
            },
            "maximum_constraint_violation": float(np.max(violations)),
            "worst_constraints": [
                {"index": int(index), "category": category(int(index)),
                 "value": float(constraint_values[index]),
                 "lower": float(lower_bounds[index]),
                 "upper": float(upper_bounds[index]),
                 "violation": float(violations[index])}
                for index in worst if violations[index] > 1.0e-8
            ],
        }
        print(json.dumps(failure, indent=2, default=str))
        raise
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
    offset_result = value(lateral_offset)
    throttle_result = value(ca.vertcat(*throttle_values))
    brake_result = value(ca.vertcat(*brake_values))
    utilization_result = value(ca.vertcat(*utilizations)).reshape(nodes, 4)
    load_result = value(ca.vertcat(*normal_loads)).reshape(nodes, 4)
    path_x_result = value(ca.vertcat(*path_x))
    path_y_result = value(ca.vertcat(*path_y))
    heading_result = value(ca.vertcat(*path_headings))
    curvature_result = value(ca.vertcat(*path_curvatures))
    segment_result = value(ca.vertcat(*segment_lengths))
    speed_defect_result = value(ca.vertcat(*speed_defects))
    time_defect_result = value(ca.vertcat(*time_defects))
    lateral_result = value(ca.vertcat(*lateral_residuals))
    yaw_result = value(ca.vertcat(*yaw_residuals))

    args.output.mkdir(parents=True, exist_ok=True)
    stem = f"racing-line-{args.initialization}-n{nodes}"
    trajectory_path = args.output / f"{stem}.csv"
    with trajectory_path.open("w", newline="", encoding="utf-8") as stream:
        fields = ["s_m", "time_s", "speed_m_s", "steering_angle_rad", "sideslip_rad",
                  "longitudinal_accel_m_s2", "lateral_accel_m_s2", "throttle", "brake",
                  "curvature_1_m", "grade", "lateral_offset_m", "path_heading_rad",
                  "path_curvature_1_m", "position_x_m", "position_y_m", "utilization_fl",
                  "utilization_fr", "utilization_rl", "utilization_rr", "fz_fl_n", "fz_fr_n",
                  "fz_rl_n", "fz_rr_n"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index in range(nodes):
            writer.writerow({
                "s_m": s_nodes[index], "time_s": time_result[index],
                "speed_m_s": math.sqrt(z_result[index]),
                "steering_angle_rad": steering_result[index], "sideslip_rad": sideslip_result[index],
                "longitudinal_accel_m_s2": ax_result[index],
                "lateral_accel_m_s2": z_result[index] * curvature_result[index],
                "throttle": float(np.clip(throttle_result[index], 0.0, 1.0)),
                "brake": float(np.clip(brake_result[index], 0.0, 1.0)),
                "curvature_1_m": center_curvature[index], "grade": grade[index],
                "lateral_offset_m": offset_result[index], "path_heading_rad": heading_result[index],
                "path_curvature_1_m": curvature_result[index], "position_x_m": path_x_result[index],
                "position_y_m": path_y_result[index],
                **{f"utilization_{wheel}": utilization_result[index, wheel_index]
                   for wheel_index, wheel in enumerate(("fl", "fr", "rl", "rr"))},
                **{f"fz_{wheel}_n": load_result[index, wheel_index]
                   for wheel_index, wheel in enumerate(("fl", "fr", "rl", "rr"))},
            })

    diagnostics = {
        "schema_version": 1, "problem": "free_racing_line_minimum_time",
        "claim": "locally optimized reduced-model solution; independent replay required",
        "track": manifest["track"]["name"], "vehicle": vehicle["name"],
        "initialization": args.initialization,
        "line_locked": args.lock_line,
        "solver": {"modeling": f"CasADi {ca.__version__}", "nlp": "IPOPT",
                   "status": stats.get("return_status"), "success": bool(stats.get("success")),
                   "iterations": int(stats.get("iter_count", -1)), "wall_time_s": wall_time},
        "mesh": {"nodes": nodes, "centerline_spacing_m": ds,
                 "optimized_path_length_m": float(np.sum(segment_result))},
        "size": {"decision_variables": int(opti.x.numel()), "constraints": int(opti.g.numel())},
        "objective": {"predicted_lap_time_s": float(time_result[-1]),
                      "fixed_line_warm_start_lap_time_s": args.fixed_line_time,
                      "predicted_gain_vs_fixed_s": args.fixed_line_time - float(time_result[-1])},
        "line": {"safety_margin_m": args.margin,
                 "minimum_offset_m": float(np.min(offset_result)),
                 "maximum_offset_m": float(np.max(offset_result)),
                 "maximum_absolute_offset_m": float(np.max(np.abs(offset_result))),
                 "closest_cg_boundary_clearance_m": float(np.min(np.minimum(
                     left_width - offset_result, right_width + offset_result)))},
        "residuals": {
            "maximum_speed_dynamic_defect_m2_s2": float(np.max(np.abs(speed_defect_result))),
            "maximum_time_dynamic_defect_s": float(np.max(np.abs(time_defect_result))),
            "maximum_lateral_equilibrium_m_s2": float(np.max(np.abs(lateral_result))),
            "maximum_normalized_yaw_equilibrium": float(np.max(np.abs(yaw_result))),
            "maximum_tire_force_violation": float(max(0.0, np.max(utilization_result) - 1.0)),
            "minimum_normal_load_n": float(np.min(load_result)),
            "maximum_control_violation": float(max(0.0, np.max(np.abs(command_result)) - 1.0)),
            "maximum_track_violation_m": float(max(0.0, np.max(
                np.maximum(offset_result - (left_width - args.margin),
                           (-right_width + args.margin) - offset_result)))),
            "periodicity_speed_m_s": float(abs(math.sqrt(z_result[0]) - math.sqrt(
                z_result[-1] + segment_result[-1] * (ax_result[-1] + ax_result[0])))),
            "periodicity_lateral_offset_m": 0.0,
        },
        "model_notes": [
            "Track boundaries constrain CG with an explicit safety margin; no body envelope is modeled.",
            "The line emerges from displaced centerline geometry; no apex or racing-line rules are imposed.",
            "Slope and second-derivative bounds suppress mesh-scale zig-zag artifacts.",
            "Lateral offset, slope and curvature use cyclic indices, enforcing closed-line periodicity.",
            "Independent production replay is mandatory before acceptance.",
        ],
        "warm_start": {"source": str(args.warm_start_trajectory.resolve()),
                       "initialization": args.initialization},
        "trajectory": trajectory_path.name,
    }
    diagnostics_path = args.output / f"{stem}.json"
    diagnostics_path.write_text(json.dumps(diagnostics, indent=2) + "\n", encoding="utf-8")
    return diagnostics


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", type=Path, required=True)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--warm-start-trajectory", type=Path, required=True)
    parser.add_argument("--fixed-line-time", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--nodes", type=int, default=200)
    parser.add_argument("--initialization", choices=("warm", "center", "left", "right", "heuristic"),
                        default="center")
    parser.add_argument("--margin", type=float, default=1.0)
    parser.add_argument("--lock-line", action="store_true",
                        help="hold the selected initialization line; useful for continuation")
    parser.add_argument("--warm-speed-scale", type=float, default=0.85)
    parser.add_argument("--minimum-speed", type=float, default=2.0)
    parser.add_argument("--maximum-speed", type=float, default=80.0)
    parser.add_argument("--maximum-steering", type=float, default=0.32)
    parser.add_argument("--maximum-lateral-slope", type=float, default=0.25)
    parser.add_argument("--maximum-lateral-second-derivative", type=float, default=0.025)
    parser.add_argument("--maximum-iterations", type=int, default=4000)
    parser.add_argument("--tolerance", type=float, default=1.0e-7)
    parser.add_argument("--print-level", type=int, default=3)
    args = parser.parse_args()
    if args.nodes < 40:
        parser.error("--nodes must be at least 40")
    diagnostics = solve(args)
    print(json.dumps(diagnostics, indent=2))
    return 0 if diagnostics["solver"]["success"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
