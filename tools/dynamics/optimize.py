#!/usr/bin/env python3
"""Constrained, parameterized single shooting through the native contact simulator.
The result is a locally improved policy within a periodic path/speed spline family.
It is not a full-state collocation solution or a global optimum.
"""
import argparse
import json
import time
import subprocess
import sys
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
from native import Native, ROOT, LIB
from evaluation_cache import EvaluationCache, configuration_identity
from publication import publish_accepted
PRESETS={'fastest':[1,0,0,0,0,0,0], 'shortest':[0,1,0,0,0,0,0], 'clearance':[0,0,1,0,0,0,0],
 'neutral':[0,0,0,1,0,0,0], 'platform':[0,0,0,0,1,0,0], 'fuel':[0,0,0,0,0,1,0], 'wear':[0,0,0,0,0,0,1]}
def save(path, value):
 tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n');tmp.replace(path)
def main():
 p=argparse.ArgumentParser();p.add_argument('--vehicle',default='mclaren-p1');p.add_argument('--track',default='technical_test_circuit');p.add_argument('--objective',choices=[*PRESETS,'custom'],default='fastest');p.add_argument('--nodes',type=int,default=4);p.add_argument('--maxiter',type=int,default=120);p.add_argument('--tolerance',type=float,default=.02);p.add_argument('--initial-bias',type=float,default=0);p.add_argument('--weights',type=float,nargs=7);p.add_argument('--time-budget',type=float,default=1.15);p.add_argument('--reference-scale',type=float,default=1.);p.add_argument('--dt',type=float,default=.002);p.add_argument('--output',type=Path);args=p.parse_args()
 if args.nodes<4 or args.time_budget<1:raise SystemExit('nodes >=4 and time budget >=1 required')
 weights=np.array(args.weights if args.weights else PRESETS.get(args.objective,[]),float)
 if len(weights)!=7 or np.any(weights<0) or not np.any(weights>0):raise SystemExit('seven nonnegative weights, at least one positive, required')
 weights/=weights.sum();out=args.output or ROOT/f'data/generated/dynamics/{args.track}-{args.vehicle}-{args.objective}.json';out.parent.mkdir(parents=True,exist_ok=True)
 with Native(args.vehicle,args.track) as n:
  baseline=n.rollout({'offsets':[0.]*args.nodes,'scales':[args.reference_scale]*args.nodes,'record':True,'warmup':1,'dt':args.dt})
  if not baseline['complete'] or baseline['minimum_clearance_m']<0:raise SystemExit('Reference failed feasibility; optimization not started')
  # Every candidate begins from exactly the same full physical/resource state.
  initial=baseline['initial'];initial['deployed']=initial['recovered']=0
  reference=n.rollout({'offsets':[0.]*args.nodes,'scales':[args.reference_scale]*args.nodes,'record':True,'warmup':0,'initial':initial,'dt':args.dt})
  scales=np.array([reference['lap_time_s'],reference['path_m'],max(reference['minimum_clearance_m'],.5),max(reference['balance_cost'],.002),max(reference['stability_cost'],.002),max(reference['fuel_kg'],.01),max(reference['wear'],1e-5)])
  evaluations=[];cache={};best=None;start=time.perf_counter();cache_hits=0
  persistent=EvaluationCache(ROOT/'.cache/dynamics-evaluations', {'configuration':configuration_identity(ROOT,LIB),'vehicle':args.vehicle,'track':args.track,'objective':weights.tolist(),'normalization':scales.tolist(),'time_budget':args.time_budget,'version':1})
  def status():
   return {'algorithm':'SciPy PRIMA COBYLA','decision_variables':2*(args.nodes-1),'gradients':False,'worker_count':1,'evaluations':len(evaluations),'native_evaluations':persistent.misses,'cache_hits':cache_hits+persistent.hits,'best_objective':None if best is None else best['objective'],'constraint_violation':None if best is None else max(0.,-min(best['constraints'])),'wall_s':time.perf_counter()-start,'mean_evaluation_s':sum(r.get('evaluation_wall_s',0) for r in evaluations)/max(1,len(evaluations)),'improvement':None if best is None or not evaluations else evaluations[0]['objective']-best['objective']}
  def decode(x):return {'offsets':[0.,*map(float,x[:args.nodes-1])],'scales':[args.reference_scale,*map(float,x[args.nodes-1:])],'initial':initial,'warmup':0,'dt':args.dt}
  def assess(x):
   nonlocal best,cache_hits
   key=tuple(x)
   if key in cache:
    cache_hits+=1
    return cache[key]
   evaluation_start=time.perf_counter();r=dict(persistent.evaluate(decode(x),lambda:n.rollout(decode(x))));r['evaluation_wall_s']=time.perf_counter()-evaluation_start;components=np.array([r['lap_time_s'],r['path_m'],-r['minimum_clearance_m'],r['balance_cost'],r['stability_cost'],r['fuel_kg'],r['wear']])/scales
   objective=float(weights@components)
   # Actual body-envelope clearance and maximum time. Tire law enforces its circle.
   constraints=np.array([r['minimum_clearance_m'],args.time_budget*reference['lap_time_s']-r['lap_time_s'],1. if r['complete'] else -100.,1.-r.get('periodicity',{}).get('normalized_max',1e6)])
   if not r['complete']:objective+=1000
   r['objective']=objective;r['normalized_components']=components.tolist();r['constraints']=constraints.tolist();r['x']=list(map(float,x))
   evaluations.append({k:v for k,v in r.items() if k not in ('initial','final')});cache[key]=r
   within_bounds=bool(np.all(np.abs(x[:args.nodes-1])<=3+1e-8) and np.all(x[args.nodes-1:]>=.7*args.reference_scale-1e-8) and np.all(x[args.nodes-1:]<=1.45*args.reference_scale+1e-8))
   if within_bounds and np.min(constraints)>=-1e-5 and (best is None or objective<best['objective']):best=r
   print(f'{len(evaluations):03d} J={objective:.6f} T={r["lap_time_s"]:.3f}s clearance={r["minimum_clearance_m"]:.3f}m complete={r["complete"]}',flush=True)
   save(out.with_suffix('.progress.json'),{'status':'running',**status()})
   return r
  x0=np.r_[np.full(args.nodes-1,args.initial_bias),np.full(args.nodes-1,args.reference_scale)]
  assess(x0)
  bounds=[(-3.,3.)]*(args.nodes-1)+[(.7*args.reference_scale,1.45*args.reference_scale)]*(args.nodes-1)
  solver=minimize(lambda x:assess(x)['objective'],x0,method='COBYLA',bounds=bounds,constraints={'type':'ineq','fun':lambda x:assess(x)['constraints']},options={'maxiter':args.maxiter,'rhobeg':.12,'tol':args.tolerance,'catol':1e-4})
  if best is None:raise SystemExit('No feasible candidate')
  predicted=n.rollout({**decode(best['x']),'record':True})
  open_loop=n.rollout({**predicted,'replay_dt':args.dt/2},replay=True)
  # Independent time-domain validation with the same recorded path/speed policy,
  # feedback recomputed from actual replay state. Also retain open-loop divergence.
  replay=n.rollout({**decode(best['x']),'dt':args.dt/2,'record':True})
  a=predicted['samples'];b=replay['samples'];times=np.array([s['time'] for s in a]);positions=np.array([s['position'] for s in a]);speeds=np.array([np.linalg.norm(s['velocity']) for s in a]);bt=np.array([s['time'] for s in b])
  pa=np.stack([np.interp(bt,times,positions[:,i]) for i in range(3)],axis=1);pb=np.array([s['position'] for s in b]);position_error=np.linalg.norm(pa-pb,axis=1);speed_error=np.abs(np.interp(bt,times,speeds)-[np.linalg.norm(s['velocity']) for s in b])
  def yaw(row):
   w,x,y,z=row['quaternion'];return np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
  yaw_error=np.abs((np.interp(bt,times,np.unwrap([yaw(s) for s in a]))-np.unwrap([yaw(s) for s in b])+np.pi)%(2*np.pi)-np.pi)
  reference_s=np.array([s['s'] for s in a]);lap_delta=float(np.interp(n.snapshot(True)['length'],[s['s'] for s in b],bt)-predicted['lap_time_s'])
  replay_checks={'position_max_m':float(position_error.max()),'position_rms_m':float(np.sqrt(np.mean(position_error**2))),'speed_max_mps':float(speed_error.max()),'yaw_max_rad':float(yaw_error.max()),'lap_time_difference_s':lap_delta,'dt_s':args.dt/2,'progress_m':replay['progress_m'],'mode':f'native time-domain trajectory tracking at {args.dt/2*1000:g} ms; feedback recomputed','open_loop_final_position_error_m':float(np.linalg.norm(np.array(open_loop['samples'][-1]['position'])-positions[-1])),'open_loop_complete':bool(open_loop['progress_m']>=reference_s[-1]-.5),'replay_minimum_clearance_m':replay['minimum_clearance_m']}
  accepted=bool(solver.success and predicted['complete'] and predicted['minimum_clearance_m']>=-1e-4 and position_error.max()<.5 and speed_error.max()<.5 and yaw_error.max()<.03 and replay['complete'] and replay['minimum_clearance_m']>=0 and predicted['periodicity']['normalized_max']<=1.0001 and replay['periodicity']['normalized_max']<=1.0001)
  result={**predicted,'vehicle':args.vehicle,'track':args.track,'accepted':accepted,'scope':'locally optimized periodic path-and-speed policy; native single shooting, not full-state collocation or global optimum',
    'objective_name':args.objective,'weights':weights.tolist(),'normalization_scales':scales.tolist(),'objective_value':best['objective'],'profile':decode(best['x']),
    'solver':{**status(),'name':'SciPy COBYLA (PRIMA)','success':bool(solver.success),'status':int(solver.status),'message':str(solver.message),'evaluations':int(solver.nfev),'wall_s':time.perf_counter()-start,'nodes':args.nodes,'decision_variables':len(x0),'inequality_constraints':4+2*len(x0)},
    'reference_lap_time_s':reference['lap_time_s'],'replay_validation':replay_checks,'evaluations':evaluations}
  candidate_path=out.with_name(out.stem+'-candidate.json');save(candidate_path,result)
  save(out.with_suffix('.progress.json'),{**status(),'status':'validating','solver':result['solver'],'replay':replay_checks})
  print(json.dumps({k:result[k] for k in ('accepted','lap_time_s','reference_lap_time_s','solver','replay_validation')},indent=2),flush=True)
  if not accepted and solver.success:
   audit=subprocess.run([sys.executable,str(ROOT/'tools/dynamics/select_replay_stable.py'),str(candidate_path)])
   if audit.returncode==0:
    result=json.loads(candidate_path.read_text());accepted=result['accepted']
  if accepted:result=publish_accepted(out,result,reference)
  save(out.with_suffix('.progress.json'),{**status(),'status':'accepted' if accepted else 'rejected','best_objective':result['objective_value'] if accepted else status()['best_objective'],'solver':result['solver'],'replay':result['replay_validation']})
  if not accepted:raise SystemExit(2)
if __name__=='__main__':main()
