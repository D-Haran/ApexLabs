#!/usr/bin/env python3
"""Reject discretization-sensitive policies, testing observed bounded candidates in rank order."""
import json,sys
from pathlib import Path
import numpy as np
from native import Native
path=Path(sys.argv[1]);j=json.loads(path.read_text());nodes=j['solver']['nodes'];profile=j['profile'];scale=profile['scales'][0]
candidates=sorted([r for r in j['evaluations'] if min(r['constraints'])>=-1e-5 and all(abs(v)<=3+1e-8 for v in r['x'][:nodes-1]) and all(.7*scale-1e-8<=v<=1.45*scale+1e-8 for v in r['x'][nodes-1:])],key=lambda r:r['objective'])
audit=[]
with Native(j['vehicle'],j['track']) as n:
 for candidate in candidates:
  dt=profile.get('dt',j.get('dt_s',.002))
  request={**profile,'dt':dt,'offsets':[0.,*candidate['x'][:nodes-1]],'scales':[scale,*candidate['x'][nodes-1:]],'record':True}
  prediction=n.rollout(request);replay=n.rollout({**request,'dt':dt/2});a=prediction['samples'];b=replay['samples'];t=np.array([s['time'] for s in a]);bt=np.array([s['time'] for s in b]);pos=np.array([s['position'] for s in a]);posb=np.array([s['position'] for s in b]);error=np.linalg.norm(np.stack([np.interp(bt,t,pos[:,i]) for i in range(3)],axis=1)-posb,axis=1);ve=np.abs(np.interp(bt,t,[np.linalg.norm(s['velocity']) for s in a])-[np.linalg.norm(s['velocity']) for s in b])
  def yaw(s):
   w,x,y,z=s['quaternion'];return np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
  ye=np.abs((np.interp(bt,t,np.unwrap([yaw(s) for s in a]))-np.unwrap([yaw(s) for s in b])+np.pi)%(2*np.pi)-np.pi)
  accepted=bool(j['solver']['success'] and prediction['complete'] and replay['complete'] and error.max()<.5 and ve.max()<.5 and ye.max()<.03 and min(prediction['minimum_clearance_m'],replay['minimum_clearance_m'])>=0 and prediction['periodicity']['normalized_max']<=1.0001 and replay['periodicity']['normalized_max']<=1.0001)
  checks={'lap_time_s':prediction['lap_time_s'],'position_max_m':float(error.max()),'position_rms_m':float(np.sqrt(np.mean(error**2))),'speed_max_mps':float(ve.max()),'yaw_max_rad':float(ye.max()),'lap_time_difference_s':replay['lap_time_s']-prediction['lap_time_s'],'dt_s':dt/2,'mode':'independent native trajectory feedback; candidate audit','replay_minimum_clearance_m':replay['minimum_clearance_m'],'accepted':accepted}
  audit.append(checks);print(checks,flush=True)
  if accepted:
   original={k:v for k,v in j.items() if k not in ['samples','commands','evaluations']};path.with_suffix('.selection-audit.json').write_text(json.dumps({'original':original,'candidate_checks':audit},indent=2)+'\n')
   j.update(prediction);request.pop('record');j.update(profile=request,replay_validation=checks,accepted=True,objective_value=candidate['objective'],replay_stable_selection_audit=audit);j.pop('post_solve_refinement',None)
   tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(j,indent=2)+'\n');tmp.replace(path);break
 else:raise SystemExit('No candidate passed independent replay')
