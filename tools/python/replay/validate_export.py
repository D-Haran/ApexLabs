#!/usr/bin/env python3
"""Integration checks for actual C++ replay exports, including a two-timed-lap run."""
import csv
import json
import math
import tempfile
from pathlib import Path
from export_session import ROOT, export


def read(path):
    with path.open() as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def check(directory):
    m = json.loads((directory / 'session.json').read_text())
    rows, wheels, spatial = (read(directory / name) for name in ['telemetry.csv', 'wheels.csv', 'spatial.csv'])
    assert len(rows) == len(wheels)
    assert all(a['time_s'] == b['time_s'] and a['step'] == b['step'] for a, b in zip(rows, wheels))
    for a, b in zip(rows, wheels):
        assert a['schema_version'] == 4 and b['schema_version'] == 1
        assert math.isclose(sum(b[f'fz_{w}_n'] for w in ['fl', 'fr', 'rl', 'rr']), m['vehicle']['mass_kg']*9.80665, abs_tol=1e-7)
        for w in ['fl', 'fr', 'rl', 'rr']:
            use = math.hypot(b[f'fx_{w}_n'], b[f'fy_{w}_n'])/b[f'force_capacity_{w}_n']
            assert math.isclose(use, a[f'friction_utilization_{w}'], abs_tol=1e-12)
    for lap in m['laps']:
        assert math.isclose(sum(lap['sector_times_s']), lap['lap_time_s'], abs_tol=1e-8)
        grid = [r for r in spatial if r['lap_number'] == lap['number']]
        assert len(grid) > 1000
        assert grid[0]['time_s']-lap['start_time_s'] < .1
        assert lap['end_time_s']-grid[-1]['time_s'] < .1
        assert all(a['time_s'] < b['time_s'] and a['s_m'] < b['s_m'] for a, b in zip(grid, grid[1:]))
    return m


if __name__ == '__main__':
    for name in ['baseline', 'high-grip']:
        check(ROOT/'apps/dashboard/public/demo'/name)
    with tempfile.TemporaryDirectory(prefix='apexlab-replay-') as d:
        path=Path(d)
        export(ROOT/'build/apps/sim-cli/apexlab-sim', ROOT/'configs/vehicles/generic_nonlinear_performance_car.json',
               ROOT/'configs/tracks/technical_test_circuit.json', path, 'Multi-lap integrity test', 1, 2)
        m=check(path)
        assert len(m['laps']) == 2
        assert m['laps'][0]['end_time_s'] == m['laps'][1]['start_time_s']
    print('Replay export integrity passed: two demos, force/capacity/load identities, spatial seam coverage, two timed laps')
