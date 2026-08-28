#!/usr/bin/env python3
"""Run the unchanged lap physics and package a self-contained version-1 replay session.
Each completed timed lap gets an explicit time interval in the manifest. No decimation.
"""
import argparse
import csv
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

def export(sim, vehicle, track, output, name, warmup=1, laps=1, maximum_duration=300,
           timestep=0.005, optimization_profile=None, replay_diagnostics=None):
    output.mkdir(parents=True, exist_ok=True)
    command = [str(sim.resolve()), '--vehicle', str(vehicle.resolve()), '--track', str(track.resolve()),
               '--output', str(output / 'telemetry.csv'), '--track-geometry', str(output / 'geometry.csv'),
               '--replay-data', str(output), '--warmup-laps', str(warmup), '--laps', str(laps),
               '--dt', str(timestep), '--controller-dt', '0.02',
               '--maximum-lap-duration', str(maximum_duration)]
    if optimization_profile is not None:
        command.extend(['--optimization-profile', str(optimization_profile.resolve())])
        if replay_diagnostics is not None:
            command.extend(['--replay-diagnostics', str(replay_diagnostics.resolve())])
    subprocess.run(command, check=True)
    v, t = json.loads(vehicle.read_text()), json.loads(track.read_text())
    results = json.loads((output / 'results.json').read_text())
    with (output / 'telemetry.csv').open() as f:
        rows = list(csv.DictReader(f))
    sectors = len(t['sector_boundaries_fraction'])
    lap_meta = []
    for index, duration in enumerate(results['lap_times_s']):
        number = warmup + index
        end = float([r for r in rows if int(r['lap_number']) == number][-1]['time_s'])
        lap_meta.append({'number': number, 'start_time_s': end-duration, 'end_time_s': end,
                         'lap_time_s': duration,
                         'sector_times_s': results['sector_times_s'][index*sectors:(index+1)*sectors]})
    # Include the recorded crossing sample at the beginning of the first timed lap.
    start = lap_meta[0]['start_time_s']
    for filename in ['telemetry.csv', 'wheels.csv'] + (['chassis.csv'] if v.get('normal_load_model') == 'sprung_body' else []):
        path = output / filename
        with path.open() as f:
            reader = csv.DictReader(f); fields = reader.fieldnames
            selected = [r for r in reader if float(r['time_s']) >= start-1e-9]
        with path.open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader(); writer.writerows(selected)
    (output / 'vehicle.json').write_text(json.dumps(v, indent=2)+'\n')
    (output / 'track.json').write_text(json.dumps(t, indent=2)+'\n')
    files = {'telemetry': 'telemetry.csv', 'wheels': 'wheels.csv',
             'geometry': 'geometry.csv', 'spatial': 'spatial.csv'}
    if v.get('normal_load_model') == 'sprung_body':
        files['chassis'] = 'chassis.csv'
    render_context = track.with_suffix('.render.json')
    if render_context.is_file():
        shutil.copyfile(render_context, output / 'render.json')
        files['render'] = 'render.json'
    manifest = {
        'schema_version': 1, 'telemetry_schema_version': 4, 'wheel_schema_version': 1,
        'spatial_schema_version': 1, 'session_name': name,
        'files': files,
        'vehicle': {'name': v['name'], 'mass_kg': v['mass_kg'], 'mu_reference': v['tires']['tire_mu_reference'],
                    'front_axle_m': v['planar']['cg_to_front_axle_m'], 'rear_axle_m': v['planar']['cg_to_rear_axle_m'],
                    'front_track_m': v['nonlinear_planar']['front_track_m'], 'rear_track_m': v['nonlinear_planar']['rear_track_m'],
                    'provenance': v['provenance'],
                    'aero': v.get('aero'), 'normal_load_model': v.get('normal_load_model', 'quasi_static'),
                    'cg_height_m': v['nonlinear_planar']['cg_height_m'], 'suspension': v.get('suspension')},
        'track': {'name': t['name'], 'length_m': results['track_length_m'], 'reference_offset_m': 0,
                  'geometry_sha256': hashlib.sha256((output/'geometry.csv').read_bytes()).hexdigest(),
                  'sector_boundaries_fraction': t['sector_boundaries_fraction'], 'provenance': t['provenance'],
                  'kind': t.get('kind', 'synthetic_validation'),
                  'coordinate_system': t.get('coordinate_system'),
                  'elevation_source': t.get('elevation', {}).get('source'),
                  'elevation_limitation': t.get('elevation', {}).get('limitation')},
        'simulation': {'timestep_s': timestep, 'controller_timestep_s': 0.02, 'integrator': 'rk4', 'warmup_laps': warmup},
        'laps': lap_meta,
    }
    if optimization_profile is not None:
        manifest['optimization'] = {
            'kind': 'locally_optimized_racing_line',
            'profile': str(optimization_profile),
            'independent_production_replay': True,
        }
    (output / 'session.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim', type=Path, default=ROOT/'build/apps/sim-cli/apexlab-sim')
    parser.add_argument('--vehicle', type=Path, default=ROOT/'configs/vehicles/generic_nonlinear_performance_car.json')
    parser.add_argument('--track', type=Path, default=ROOT/'configs/tracks/technical_test_circuit.json')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--name', default='Technical circuit · baseline')
    parser.add_argument('--warmup-laps', type=int, default=1)
    parser.add_argument('--laps', type=int, default=1)
    parser.add_argument('--maximum-duration', type=float, default=300)
    parser.add_argument('--dt', type=float, default=0.005)
    parser.add_argument('--optimization-profile', type=Path)
    parser.add_argument('--replay-diagnostics', type=Path)
    args = parser.parse_args()
    export(args.sim, args.vehicle, args.track, args.output.resolve(), args.name, args.warmup_laps,
           args.laps, args.maximum_duration, args.dt, args.optimization_profile,
           args.replay_diagnostics)
