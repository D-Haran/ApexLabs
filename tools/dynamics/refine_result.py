#!/usr/bin/env python3
"""Refine integration of an existing bounded policy, retaining original solver evidence."""
import json,sys
from pathlib import Path
import numpy as np
from native import Native
p=Path(sys.argv[1]);j=json.loads(p.read_text());vehicle=j['vehicle'];track=j['track'];profile=j['profile']
with Native(vehicle,track) as n:
 prediction=n.rollout({**profile,'dt':.001,'record':True});replay=n.rollout({**profile,'dt':.0005,'record':True})
a=prediction['samples'];b=replay['samples'];t=np.array([s['time'] for s in a]);bt=np.array([s['time'] for s in b]);pos=np.array([s['position'] for s in a]);posb=np.array([s['position'] for s in b]);pa=np.stack([np.interp(bt,t,pos[:,i]) for i in range(3)],axis=1)
error=np.linalg.norm(pa-posb,axis=1);ve=np.abs(np.interp(bt,t,[np.linalg.norm(s['velocity']) for s in a])-[np.linalg.norm(s['velocity']) for s in b])
def yaw(s):
 w,x,y,z=s['quaternion'];return np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
ye=np.abs((np.interp(bt,t,np.unwrap([yaw(s) for s in a]))-np.unwrap([yaw(s) for s in b])+np.pi)%(2*np.pi)-np.pi)
checks={'position_max_m':float(error.max()),'position_rms_m':float(np.sqrt(np.mean(error**2))),'speed_max_mps':float(ve.max()),'yaw_max_rad':float(ye.max()),'lap_time_difference_s':replay['lap_time_s']-prediction['lap_time_s'],'dt_s':.0005,'mode':'independent native trajectory feedback; post-solve timestep refinement','progress_m':replay['progress_m'],'replay_minimum_clearance_m':replay['minimum_clearance_m'],'original_open_loop_validation':j['replay_validation']}
accepted=bool(j['solver']['success'] and prediction['complete'] and replay['complete'] and error.max()<.5 and ve.max()<.5 and ye.max()<.03 and min(prediction['minimum_clearance_m'],replay['minimum_clearance_m'])>=0 and prediction['periodicity']['normalized_max']<=1.0001)
audit={k:v for k,v in j.items() if k not in ['samples','commands','evaluations','profile']};p.with_suffix('.coarse-audit.json').write_text(json.dumps(audit,indent=2)+'\n');j.update(prediction);j['replay_validation']=checks;j['accepted']=accepted;j['profile']['dt']=.001;j['post_solve_refinement']={'optimizer_dt_s':.002,'prediction_dt_s':.001,'replay_dt_s':.0005};tmp=p.with_suffix('.tmp');tmp.write_text(json.dumps(j,indent=2)+'\n');tmp.replace(p)
print(accepted,p,checks,flush=True)
if not accepted:raise SystemExit(2)
