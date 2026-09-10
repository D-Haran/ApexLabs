#!/usr/bin/env python3
"""Bounded calibration against McLaren anchors; every evaluation runs native contact physics."""
import argparse
import copy
import json
from pathlib import Path
import subprocess
import tempfile
import time
import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[2]
FIELDS = ['tire_mu', 'efficiency', 'brake_force_n', 'road_cda_m2']
TARGETS = dict(zero_to_100_s=2.8, zero_to_200_s=6.8, brake_100_m=30., brake_200_m=116.)


def bench(path, dt=.002):
    return json.loads(subprocess.check_output([str(ROOT/'build/apps/sim-cli/apexlab-dynamic-bench'), str(path), str(dt)], text=True))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--fit',action='store_true');args=parser.parse_args()
    path=ROOT/'configs/vehicles/mclaren-p1-dynamic.json';config=json.loads(path.read_text())
    output=ROOT/'data/generated/dynamics';output.mkdir(parents=True,exist_ok=True)
    log=[];started=time.perf_counter();solution=None
    if args.fit:
        with tempfile.NamedTemporaryFile(mode='w',suffix='.json',prefix='.p1-calibration-',dir=path.parent) as temporary:
            trial=Path(temporary.name)
            def residual(x):
                candidate=copy.deepcopy(config)
                for k,value in zip(FIELDS,x):candidate['values'][k]=float(value)
                trial.write_text(json.dumps(candidate))
                result=bench(trial)
                errors=[(result[k]-target)/target for k,target in TARGETS.items()]
                log.append(dict(parameters=dict(zip(FIELDS,map(float,x))),results=result,relative_errors=errors))
                print(f'evaluation {len(log)} max error {max(map(abs,errors)):.5f}',flush=True)
                return errors
            initial=[config['values'][k] for k in FIELDS]
            fit=least_squares(residual,initial,bounds=([1.1,.65,15000,.35],[1.9,.98,24000,1.2]),
                              diff_step=.002,x_scale='jac',max_nfev=45,ftol=1e-6,xtol=1e-6,gtol=1e-6)
            solution=dict(success=bool(fit.success),status=int(fit.status),message=str(fit.message),nfev=int(fit.nfev),optimality=float(fit.optimality))
            if not fit.success or max(abs(fit.fun))>.03:
                (output/'calibration-rejected.json').write_text(json.dumps(dict(solver=solution,evaluations=log),indent=2)+'\n')
                raise RuntimeError('Calibration failed the 3% anchor gate; configuration unchanged')
            for k,value in zip(FIELDS,fit.x):config['values'][k]=float(value);config['parameter_classes'][k]='calibrated'
            path.write_text(json.dumps(config,indent=2)+'\n')
    measured={str(dt):bench(path,dt) for dt in (.002,.001)}
    f1path=ROOT/'configs/vehicles/mcl36-dynamic.json';f1=bench(f1path)
    errors={key:{k:(row[k]-target)/target for k,target in TARGETS.items()} for key,row in measured.items()}
    if any(abs(x)>.03 for row in errors.values() for x in row.values()):raise RuntimeError('Independent calibration gate failed')
    aero=f1['aero_70mps']
    assert aero['drs_downforce_n']<aero['downforce_n'] and aero['drs_drag_n']<aero['drag_n']
    assert aero['raised_body_downforce_n']<aero['downforce_n']
    report=dict(status='passed',scope='Phase B fixed-mass mechanical calibration; resource states not yet included',
                source='https://cars.mclaren.com/us_en/legacy/mclaren-p1',targets=TARGETS,p1=measured,f1=f1,
                relative_errors=errors,solver=solution,wall_s=time.perf_counter()-started,evaluations=log,
                limitations=['P1 top speed uses an explicitly configured electronic governor; F1 has no fixed speed cap.',
                             'Estimated torque envelope, gearbox ratios and active-aero/floor maps; no OEM maps.',
                             'Braking starts after a documented rolling-bench trim; no position/velocity intervention during measurement.'])
    (output/'phase-b-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='evaluations'},indent=2))


if __name__=='__main__':main()
