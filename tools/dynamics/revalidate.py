#!/usr/bin/env python3
"""Revalidate saved policies against the current native model; no optimization rerun."""
import json
import math
import argparse
from pathlib import Path
import numpy as np
from native import Native,ROOT
p=argparse.ArgumentParser();p.add_argument('paths',nargs='*',type=Path);p.add_argument('--output',type=Path,default=ROOT/'data/generated/dynamics/final-replay-validation.json');args=p.parse_args()
paths=args.paths
if not paths:
 for track in ['technical_test_circuit','spa-francorchamps']:
  for vehicle in ['mclaren-p1','mcl36']:
   path=ROOT/f'data/generated/dynamics/{track}-{vehicle}-fastest.json'
   if not path.exists() and track=='technical_test_circuit':path=ROOT/f'data/generated/dynamics/study/{vehicle}-fastest.json'
   if path.exists():paths.append(path)
rows=[]
for path in paths:
  j=json.loads(path.read_text())
  if 'profile' not in j:continue
  vehicle=j['vehicle'];track=j['track']
  with Native(vehicle,track) as n:
   replay=n.rollout({**j['profile'],'record':True,'dt':j['profile'].get('dt',j.get('dt_s',.002))/2})
  a=j['samples'];b=replay['samples'];t=np.array([s['time'] for s in a]);bt=np.array([s['time'] for s in b]);pos=np.array([s['position'] for s in a]);q=np.array([s['quaternion'] for s in a]);posb=np.array([s['position'] for s in b]);pa=np.stack([np.interp(bt,t,pos[:,i]) for i in range(3)],axis=1)
  errors=np.linalg.norm(pa-posb,axis=1);speeds=np.array([np.linalg.norm(s['velocity']) for s in a]);speederror=np.abs(np.interp(bt,t,speeds)-[np.linalg.norm(s['velocity']) for s in b])
  def yaw(row):
   w,x,y,z=row['quaternion'];return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
  ya=np.unwrap([yaw(s) for s in a]);yb=np.unwrap([yaw(s) for s in b]);yawerror=np.abs((np.interp(bt,t,ya)-yb+np.pi)%(2*np.pi)-np.pi)
  max_control=max(max(abs(s['steering'])-.65,s['throttle']-1,s['brake']-1,-s['throttle'],-s['brake'],0) for s in b)
  peak_util=max(w['utilization'] for s in b for w in s['wheels']);bad_contact=sum(1 for s in b for w in s['wheels'] if (not w['contact']) and (w['fz'] or w['fx'] or w['fy']))
  row={'file':str(path.relative_to(ROOT)) if path.is_absolute() else str(path),'track':track,'vehicle':vehicle,'complete':replay['complete'],'position_max_m':float(errors.max()),'speed_max_mps':float(speederror.max()),'yaw_max_rad':float(yawerror.max()),'track_violation_m':max(0,-replay['minimum_clearance_m']),'control_violation':max_control,'tire_violation':max(0,peak_util-1),'invalid_airborne_forces':bad_contact,'periodicity':replay['periodicity'],'lap_time_s':replay['lap_time_s'],'all_airborne_s':replay['all_airborne_s'],'minimum_clearance_m':replay['minimum_clearance_m']}
  row['passed']=bool(row['complete'] and row['position_max_m']<.5 and row['speed_max_mps']<.5 and row['yaw_max_rad']<.03 and row['track_violation_m']==0 and max_control<1e-9 and bad_contact==0 and peak_util<=1+1e-9 and replay['periodicity']['normalized_max']<=1.0001)
  rows.append(row);print(row,flush=True)
args.output.write_text(json.dumps(rows,indent=2)+'\n')
if not rows or not all(r['passed'] for r in rows):raise SystemExit(2)
