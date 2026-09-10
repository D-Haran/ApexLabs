#!/usr/bin/env python3
"""Audit persisted candidates against optimizer bounds and revalidate corrected selection.
Retains the original rejected selection as a compact audit record.
"""
import json,sys
from pathlib import Path
import numpy as np
from native import Native,ROOT
for arg in sys.argv[1:]:
 path=Path(arg);j=json.loads(path.read_text())
 if not isinstance(j,dict) or 'profile' not in j:continue
 n=j['solver']['nodes'];x=j['profile'];scale=x['scales'][0]
 if all(abs(v)<=3+1e-8 for v in x['offsets']) and all(.7*scale-1e-8<=v<=1.45*scale+1e-8 for v in x['scales']):continue
 candidates=[r for r in j['evaluations'] if min(r['constraints'])>=-1e-5 and all(abs(v)<=3+1e-8 for v in r['x'][:n-1]) and all(.7*scale-1e-8<=v<=1.45*scale+1e-8 for v in r['x'][n-1:])]
 if not candidates:raise SystemExit('No bounded feasible candidate: '+str(path))
 best=min(candidates,key=lambda r:r['objective']);profile={**x,'offsets':[0.,*best['x'][:n-1]],'scales':[scale,*best['x'][n-1:]]}
 vehicle=j['vehicle'];track=j['track']
 with Native(vehicle,track) as native:
  prediction=native.rollout({**profile,'record':True});replay=native.rollout({**profile,'record':True,'dt':.001});open_loop=native.rollout({**prediction,'replay_dt':.001},replay=True)
 t=np.array([s['time'] for s in prediction['samples']]);bt=np.array([s['time'] for s in replay['samples']]);a=np.array([s['position'] for s in prediction['samples']]);b=np.array([s['position'] for s in replay['samples']]);pa=np.stack([np.interp(bt,t,a[:,i]) for i in range(3)],axis=1);error=np.linalg.norm(pa-b,axis=1);vs=[np.linalg.norm(s['velocity']) for s in prediction['samples']];vr=np.array([np.linalg.norm(s['velocity']) for s in replay['samples']]);ve=np.abs(np.interp(bt,t,vs)-vr)
 audit={'reason':'Original best-observed selection included a bound-violating COBYLA exploratory evaluation. Replaced with best observed feasible candidate inside all declared bounds. No relaxation of bounds.','previous_lap_time_s':j['lap_time_s'],'previous_profile':x,'corrected':True}
 checks={'position_max_m':float(error.max()),'position_rms_m':float(np.sqrt(np.mean(error**2))),'speed_max_mps':float(ve.max()),'lap_time_difference_s':replay['lap_time_s']-prediction['lap_time_s'],'dt_s':.001,'progress_m':replay['progress_m'],'mode':'independent native trajectory feedback','open_loop_final_position_error_m':float(np.linalg.norm(np.array(open_loop['samples'][-1]['position'])-a[-1])),'open_loop_complete':bool(open_loop['progress_m']>=prediction['progress_m']-.5),'replay_minimum_clearance_m':replay['minimum_clearance_m']}
 accepted=bool(j['solver']['success'] and prediction['complete'] and replay['complete'] and min(prediction['minimum_clearance_m'],replay['minimum_clearance_m'])>=0 and error.max()<.5 and ve.max()<.5 and prediction['periodicity']['normalized_max']<=1.0001)
 j.update(prediction);j.update(profile=profile,accepted=accepted,bound_selection_audit=audit,replay_validation=checks,objective_value=best['objective'])
 tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(j,indent=2)+'\n');tmp.replace(path)
 print(path,accepted,j['lap_time_s'],checks['position_max_m'],flush=True)
