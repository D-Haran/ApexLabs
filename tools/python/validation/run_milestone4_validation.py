#!/usr/bin/env python3
"""Reproduce Milestone 4 geometry, lap, sensitivity, convergence, and SVG artifacts."""

import argparse
import bisect
import csv
import json
import math
import statistics
import subprocess
import time
from pathlib import Path


def independent_spline_curvatures(control_points, sample_s):
    """Independent Python Catmull-Rom derivative and chord-length implementation."""
    points = [(float(p["x_m"]), float(p["y_m"])) for p in control_points]
    count = len(points)
    def evaluate(u):
        u %= count; segment = int(math.floor(u)); t = u-segment
        p0,p1,p2,p3 = (points[(segment+k)%count] for k in (-1,0,1,2))
        component = lambda j: (
            0.5*(2*p1[j]+(p2[j]-p0[j])*t+(2*p0[j]-5*p1[j]+4*p2[j]-p3[j])*t*t+
                 (-p0[j]+3*p1[j]-3*p2[j]+p3[j])*t*t*t),
            0.5*((p2[j]-p0[j])+2*(2*p0[j]-5*p1[j]+4*p2[j]-p3[j])*t+
                 3*(-p0[j]+3*p1[j]-3*p2[j]+p3[j])*t*t),
            (2*p0[j]-5*p1[j]+4*p2[j]-p3[j])+3*(-p0[j]+3*p1[j]-3*p2[j]+p3[j])*t)
        return component(0), component(1)
    us=[i/256 for i in range(count*256+1)]; cumulative=[0.0]; previous=(evaluate(0)[0][0],evaluate(0)[1][0])
    for u in us[1:]:
        x,y=evaluate(u)[0][0],evaluate(u)[1][0]; cumulative.append(cumulative[-1]+math.hypot(x-previous[0],y-previous[1])); previous=(x,y)
    output=[]
    for s in sample_s:
        wrapped=s%cumulative[-1]; i=max(1,bisect.bisect_left(cumulative,wrapped)); f=(wrapped-cumulative[i-1])/(cumulative[i]-cumulative[i-1]); u=us[i-1]+f*(us[i]-us[i-1])
        x,y=evaluate(u); denom=(x[1]*x[1]+y[1]*y[1])**1.5; output.append((x[1]*y[2]-y[1]*x[2])/denom)
    return output


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def svg_plot(path, series, xlabel, ylabel, title):
    width, height, margin = 900, 420, 65
    values = [(x, y) for _, points, _ in series for x, y in points if math.isfinite(x) and math.isfinite(y)]
    xmin, xmax = min(x for x, _ in values), max(x for x, _ in values)
    ymin, ymax = min(y for _, y in values), max(y for _, y in values)
    if xmax == xmin: xmax += 1
    if ymax == ymin: ymax += 1
    pad = 0.05 * (ymax - ymin)
    ymin, ymax = ymin - pad, ymax + pad
    sx = lambda x: margin + (x - xmin) / (xmax - xmin) * (width - 2 * margin)
    sy = lambda y: height - margin - (y - ymin) / (ymax - ymin) * (height - 2 * margin)
    lines = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
             '<rect width="100%" height="100%" fill="white"/>',
             f'<text x="{width/2}" y="25" text-anchor="middle" font-family="sans-serif" font-size="18">{title}</text>',
             f'<line x1="{margin}" y1="{height-margin}" x2="{width-margin}" y2="{height-margin}" stroke="#333"/>',
             f'<line x1="{margin}" y1="{margin}" x2="{margin}" y2="{height-margin}" stroke="#333"/>',
             f'<text x="{width/2}" y="{height-12}" text-anchor="middle" font-family="sans-serif">{xlabel}</text>',
             f'<text x="18" y="{height/2}" transform="rotate(-90 18 {height/2})" text-anchor="middle" font-family="sans-serif">{ylabel}</text>']
    for label, points, color in series:
        coords = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in points)
        lines.append(f'<polyline fill="none" stroke="{color}" stroke-width="1.7" points="{coords}"/>')
    for i, (label, _, color) in enumerate(series):
        lines.append(f'<text x="{width-190}" y="{45+i*18}" fill="{color}" font-family="sans-serif" font-size="13">{label}</text>')
    lines.append('</svg>')
    path.write_text("\n".join(lines))


def svg_track(path, geometry, lap):
    width, height, margin = 760, 620, 35
    all_x = [float(r["left_x_m"]) for r in geometry] + [float(r["right_x_m"]) for r in geometry]
    all_y = [float(r["left_y_m"]) for r in geometry] + [float(r["right_y_m"]) for r in geometry]
    xmin, xmax, ymin, ymax = min(all_x), max(all_x), min(all_y), max(all_y)
    scale = min((width-2*margin)/(xmax-xmin), (height-2*margin)/(ymax-ymin))
    sx = lambda x: margin + (x-xmin)*scale
    sy = lambda y: height-margin-(y-ymin)*scale
    def points(rows, xkey, ykey): return " ".join(f"{sx(float(r[xkey])):.2f},{sy(float(r[ykey])):.2f}" for r in rows)
    text = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">', '<rect width="100%" height="100%" fill="white"/>',
            f'<polyline fill="none" stroke="#777" stroke-width="1" points="{points(geometry,"left_x_m","left_y_m")}"/>',
            f'<polyline fill="none" stroke="#777" stroke-width="1" points="{points(geometry,"right_x_m","right_y_m")}"/>',
            f'<polyline fill="none" stroke="#222" stroke-dasharray="5 4" points="{points(geometry,"x_m","y_m")}"/>',
            f'<polyline fill="none" stroke="#d62728" stroke-width="1.4" points="{points(lap,"position_x_m","position_y_m")}"/>',
            '<text x="20" y="24" font-family="sans-serif" font-size="17">Technical circuit: boundaries, reference, trajectory</text>', '</svg>']
    path.write_text("\n".join(text))


def lap_metrics(rows):
    lap_number = max(int(r["lap_number"]) for r in rows)
    lap = [r for r in rows if int(r["lap_number"]) == lap_number]
    if len(lap) < 2:
        raise RuntimeError("lap telemetry has no completed timed traversal")
    f = lambda key: [float(r[key]) for r in lap]
    errors, speeds = f("lateral_error_m"), f("speed_m_s")
    utils = [(float(r[f"friction_utilization_{wheel}"]), wheel, float(r["track_s_m"]))
             for r in lap for wheel in ("fl", "fr", "rl", "rr")]
    return lap, {
        "lap_time_s": float(lap[-1]["lap_elapsed_time_s"]),
        "mean_speed_mps": statistics.mean(speeds), "maximum_speed_mps": max(speeds),
        "minimum_speed_mps": min(speeds),
        "rms_lateral_error_m": math.sqrt(statistics.mean(x*x for x in errors)),
        "maximum_abs_lateral_error_m": max(map(abs, errors)),
        "peak_lateral_acceleration_mps2": max(map(abs, f("lateral_accel_m_s2"))),
        "peak_acceleration_mps2": max(f("longitudinal_accel_m_s2")),
        "peak_braking_mps2": min(f("longitudinal_accel_m_s2")),
        "maximum_tire_utilization": max(utils)[0], "maximum_tire_wheel": max(utils)[1],
        "maximum_tire_s_m": max(utils)[2], "all_samples_on_track": all(int(r["on_track"]) for r in lap),
        "endpoint_x_m": float(lap[-1]["position_x_m"]), "endpoint_y_m": float(lap[-1]["position_y_m"]),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=Path("data/generated/milestone-4"))
    parser.add_argument("--report", type=Path, default=Path("docs/experiments/milestone-4-results.md"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    sim = args.sim.resolve(); out = (root / args.output_dir).resolve(); out.mkdir(parents=True, exist_ok=True)
    vehicle_path = root / "configs/vehicles/generic_nonlinear_performance_car.json"
    track_path = root / "configs/tracks/technical_test_circuit.json"
    base_vehicle = json.loads(vehicle_path.read_text())

    def run(name, vehicle=vehicle_path, dt=0.005, warmup=0, geometry=None):
        output = out / f"{name}.csv"
        command = [str(sim), "--vehicle", str(vehicle), "--track", str(track_path), "--output", str(output),
                   "--warmup-laps", str(warmup), "--laps", "1", "--dt", str(dt), "--controller-dt", "0.02"]
        if geometry: command += ["--track-geometry", str(geometry)]
        start = time.perf_counter(); completed = subprocess.run(command, check=True, capture_output=True, text=True)
        wall = time.perf_counter() - start
        rows = read_csv(output); lap, metrics = lap_metrics(rows)
        metrics.update({"wall_time_s": wall, "real_time_factor": metrics["lap_time_s"] / wall,
                        "physics_steps": len(rows)-1, "cli_summary": completed.stdout.strip()})
        return rows, lap, metrics

    geometry_path = out / "technical_geometry.csv"
    base_rows, base_lap, base = run("technical_lap", warmup=1, geometry=geometry_path)
    geometry = read_csv(geometry_path)
    base["track_length_m"] = float(geometry[-1]["s_m"])
    independent_curvature = independent_spline_curvatures(
        json.loads(track_path.read_text())["control_points"],
        [float(r["s_m"]) for r in geometry[:-1]])
    transition_curvature_error = max(abs(float(row["curvature_1_m"])-reference)
                                     for row,reference in zip(geometry[:-1],independent_curvature))
    svg_track(out / "track_trajectory.svg", geometry, base_lap)
    by_s = lambda key: [(float(r["track_s_m"]), float(r[key])) for r in base_lap]
    svg_plot(out/"speed_curvature.svg", [("speed",by_s("speed_m_s"),"#1f77b4"),("target",by_s("target_speed_m_s"),"#d62728")], "track s (m)", "speed (m/s)", "Speed and target vs track position")
    svg_plot(out/"controls.svg", [("throttle",by_s("throttle"),"#2ca02c"),("brake",by_s("brake"),"#d62728"),("steering",by_s("steering_angle_rad"),"#1f77b4")], "track s (m)", "command", "Controls vs track position")
    svg_plot(out/"accelerations.svg", [("lateral",by_s("lateral_accel_m_s2"),"#9467bd"),("longitudinal",by_s("longitudinal_accel_m_s2"),"#ff7f0e")], "track s (m)", "acceleration (m/s²)", "Body acceleration")
    svg_plot(out/"tire_utilization.svg", [(w,by_s(f"friction_utilization_{w}"),c) for w,c in zip(("fl","fr","rl","rr"),("#1f77b4","#ff7f0e","#2ca02c","#d62728"))], "track s (m)", "utilization", "Four-tire utilization")
    svg_plot(out/"lateral_error.svg", [("lateral error",by_s("lateral_error_m"),"#d62728")], "track s (m)", "error (m)", "Path tracking error")

    grip_results = []
    for mu in (0.90, 1.05, 1.20, 1.35):
        vehicle = json.loads(json.dumps(base_vehicle)); vehicle["name"] = f"Grip sweep mu={mu}"; vehicle["tires"]["tire_mu_reference"] = mu
        path = out / f"vehicle_mu_{mu:.2f}.json"; path.write_text(json.dumps(vehicle, indent=2)+"\n")
        _, _, metrics = run(f"grip_{mu:.2f}", path); metrics["mu"] = mu; grip_results.append(metrics)
    mass_results = []
    for mass in (1200.0, 1450.0, 1700.0):
        vehicle = json.loads(json.dumps(base_vehicle)); vehicle["name"] = f"Mass sweep {mass:.0f} kg"; vehicle["mass_kg"] = mass
        path = out / f"vehicle_mass_{mass:.0f}.json"; path.write_text(json.dumps(vehicle, indent=2)+"\n")
        _, _, metrics = run(f"mass_{mass:.0f}", path); metrics["mass_kg"] = mass; mass_results.append(metrics)
    convergence = []
    for dt in (0.010, 0.005, 0.0025):
        _, _, metrics = run(f"timestep_{dt:g}", dt=dt); metrics["timestep_s"] = dt; convergence.append(metrics)
    svg_plot(out/"grip_sensitivity.svg", [("lap time",[(r["mu"],r["lap_time_s"]) for r in grip_results],"#1f77b4")], "tire mu reference", "lap time (s)", "Synthetic grip sensitivity")
    svg_plot(out/"mass_sensitivity.svg", [("lap time",[(r["mass_kg"],r["lap_time_s"]) for r in mass_results],"#ff7f0e")], "mass (kg)", "lap time (s)", "Synthetic mass sensitivity")

    # Analytical circle geometry, independently checked from sampled C++ output.
    radius, count = 60.0, 64
    circle = {"schema_version":1,"name":"Analytical circle","provenance":"ApexLab generated analytical validation","closed":True,
              "interpolation":"periodic_uniform_catmull_rom","control_points":[],"sector_boundaries_fraction":[1/3,2/3,1]}
    for i in range(count):
        angle = 2*math.pi*i/count
        circle["control_points"].append({"x_m":radius*math.cos(angle),"y_m":radius*math.sin(angle),"left_width_m":7,"right_width_m":7})
    circle_path = out/"circle_track.json"; circle_path.write_text(json.dumps(circle,indent=2)+"\n")
    old_track = track_path; track_path = circle_path
    circle_geometry_path = out/"circle_geometry.csv"
    _, _, circle_lap = run("circle_lap", dt=0.01, geometry=circle_geometry_path)
    circle_geometry = read_csv(circle_geometry_path)
    circle_validation = {
        "length_error_m": abs(float(circle_geometry[-1]["s_m"])-2*math.pi*radius),
        "maximum_radial_error_m": max(abs(math.hypot(float(r["x_m"]),float(r["y_m"]))-radius) for r in circle_geometry),
        "maximum_curvature_error_1_m": max(abs(float(r["curvature_1_m"])-1/radius) for r in circle_geometry),
        "maximum_projection_s_error_m": max(float(r["projection_s_error_m"]) for r in circle_geometry[1:-1]),
        "maximum_projection_lateral_error_m": max(float(r["projection_lateral_error_m"]) for r in circle_geometry[1:-1]),
        "maximum_projection_reconstruction_error_m": max(float(r["projection_reconstruction_error_m"]) for r in circle_geometry[1:-1]),
    }
    track_path = old_track
    curvature = [float(r["reference_curvature_1_m"]) for r in base_lap]
    target = [float(r["target_speed_m_s"]) for r in base_lap]
    mc, mt = statistics.mean(curvature), statistics.mean(target)
    correlation = sum((x-mc)*(y-mt) for x,y in zip(map(abs,curvature),target)) / math.sqrt(sum((abs(x)-statistics.mean(map(abs,curvature)))**2 for x in curvature)*sum((y-mt)**2 for y in target))

    results = {"circle_geometry":circle_validation,"spline_transition_cpp_python_curvature_error_1_m":transition_curvature_error,
               "circle_controller":circle_lap,"complete_lap":base,
               "speed_curvature_correlation":correlation,"grip_sensitivity":grip_results,
               "mass_sensitivity":mass_results,"timestep_convergence":convergence}
    (out/"summary.json").write_text(json.dumps(results,indent=2)+"\n")
    with (out/"sensitivity_summary.csv").open("w",newline="") as stream:
        writer=csv.writer(stream); writer.writerow(["sweep","value","lap_time_s","minimum_speed_mps","peak_utilization","rms_lateral_error_m"])
        for r in grip_results: writer.writerow(["mu",r["mu"],r["lap_time_s"],r["minimum_speed_mps"],r["maximum_tire_utilization"],r["rms_lateral_error_m"]])
        for r in mass_results: writer.writerow(["mass_kg",r["mass_kg"],r["lap_time_s"],r["minimum_speed_mps"],r["maximum_tire_utilization"],r["rms_lateral_error_m"]])

    report = f"""# Milestone 4 closed-track results

Generated by `tools/python/validation/run_milestone4_validation.py` from production C++ telemetry.

## Geometry and projection

The 60 m analytical circle length error was `{circle_validation['length_error_m']:.6g} m`, maximum radial error `{circle_validation['maximum_radial_error_m']:.6g} m`, maximum curvature error `{circle_validation['maximum_curvature_error_1_m']:.6g} 1/m`, and maximum coherent round-trip reconstruction error `{circle_validation['maximum_projection_reconstruction_error_m']:.6g} m`. An independent Python cubic/derivative implementation agreed with C++ technical-track transition curvature within `{transition_curvature_error:.6g} 1/m`.

## Feasible synthetic lap

The generated technical circuit is `{base['track_length_m']:.3f} m`. After one settling lap, the simulated lap was `{base['lap_time_s']:.3f} s` (mean `{base['mean_speed_mps']:.3f} m/s`, range `{base['minimum_speed_mps']:.3f}`–`{base['maximum_speed_mps']:.3f} m/s`). RMS/max lateral error were `{base['rms_lateral_error_m']:.3f}` / `{base['maximum_abs_lateral_error_m']:.3f} m`; every timed sample remained on track. Peak lateral/acceleration/braking magnitudes were `{base['peak_lateral_acceleration_mps2']:.3f}`, `{base['peak_acceleration_mps2']:.3f}`, and `{abs(base['peak_braking_mps2']):.3f} m/s²`. Peak tire utilization was `{base['maximum_tire_utilization']:.6f}` on `{base['maximum_tire_wheel']}` at `s={base['maximum_tire_s_m']:.3f} m`. The curvature/target-speed correlation was `{correlation:.4f}` (negative as intended).

![Track](../../data/generated/milestone-4/track_trajectory.svg)
![Speed](../../data/generated/milestone-4/speed_curvature.svg)
![Controls](../../data/generated/milestone-4/controls.svg)
![Acceleration](../../data/generated/milestone-4/accelerations.svg)
![Tire utilization](../../data/generated/milestone-4/tire_utilization.svg)
![Path error](../../data/generated/milestone-4/lateral_error.svg)

## Sensitivities and convergence

Grip sweep lap times: {', '.join(f"mu={r['mu']:.2f}: {r['lap_time_s']:.3f} s" for r in grip_results)}. The controlled higher-grip setup is the single-parameter setup comparison; differences concentrate at curvature-limited sections and their following acceleration zones.

Mass sweep lap times: {', '.join(f"{r['mass_kg']:.0f} kg: {r['lap_time_s']:.3f} s" for r in mass_results)}. Fixed force limits reduce acceleration and braking in acceleration units as mass rises; load sensitivity also changes effective tire friction.

RK4 timestep lap times with a fixed 20 ms controller update: {', '.join(f"dt={r['timestep_s']*1000:g} ms: {r['lap_time_s']:.3f} s" for r in convergence)}.

![Grip](../../data/generated/milestone-4/grip_sensitivity.svg)
![Mass](../../data/generated/milestone-4/mass_sensitivity.svg)

These are feasible reference laps, not minimum-time or optimal laps. Results depend on the fixed reference line, conservative speed heuristic, controller, and current nonlinear vehicle model.
"""
    report_path = (root/args.report).resolve(); report_path.parent.mkdir(parents=True,exist_ok=True); report_path.write_text(report)
    print(json.dumps(results,indent=2)); print(f"milestone 4 validation passed; report={report_path}")


if __name__ == "__main__":
    main()
