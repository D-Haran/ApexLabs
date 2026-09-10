#!/usr/bin/env python3
"""Isolated mass/grip configurations; never edit calibrated source configurations."""
import argparse
import json
import subprocess
import sys
import uuid
from pathlib import Path
from native import ROOT
base_dir=ROOT/'configs/vehicles';output=ROOT/'data/generated/dynamics/sensitivity';output.mkdir(parents=True,exist_ok=True)
parser=argparse.ArgumentParser();parser.add_argument('--case');parser.add_argument('--dt',type=float,default=.002);args=parser.parse_args()
cases=[('grip',.85),('grip',1.15),('mass',.9),('mass',1.1)]
if args.case:
 key,factor=args.case.split(':');cases=[(key,float(factor))]
summary=json.loads((output/'summary.json').read_text()) if args.case and (output/'summary.json').exists() else []
for kind,factor in cases:
 token='sensitivity-'+uuid.uuid4().hex;config=base_dir/f'{token}-dynamic.json';chassis=base_dir/f'{token}-chassis.json'
 p=json.loads((base_dir/'mclaren-p1-dynamic.json').read_text());b=json.loads((base_dir/p['base_vehicle']).read_text())
 if kind=='grip':p['values']['tire_mu']*=factor;p['parameter_classes']['tire_mu']='estimated'
 else:b['mass_kg']*=factor
 p['base_vehicle']=chassis.name;config.write_text(json.dumps(p));chassis.write_text(json.dumps(b))
 suffix=f'-dt{args.dt}' if args.dt!=.002 else '';path=output/f'{kind}-{factor}{suffix}.json'
 try:
  with path.with_suffix('.log').open('w') as log:
   r=subprocess.run([sys.executable,str(ROOT/'tools/dynamics/optimize.py'),'--vehicle',token,'--tolerance','.04','--dt',str(args.dt),'--output',str(path)],stdout=log,stderr=subprocess.STDOUT)
  j=json.loads(path.read_text()) if path.exists() else {}
  summary.append({'kind':kind,'factor':factor,'dt_s':args.dt,'exit_code':r.returncode,**{k:v for k,v in j.items() if k not in ['samples','commands','evaluations','profile','initial','final']}})
  (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(kind,factor,r.returncode,j.get('lap_time_s'),flush=True)
 finally:config.unlink(missing_ok=True);chassis.unlink(missing_ok=True)
