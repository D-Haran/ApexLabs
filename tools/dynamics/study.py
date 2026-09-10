#!/usr/bin/env python3
"""Reproduce the mode/grid/initialization study; preserve every rejected solve."""
import argparse
import json
import subprocess
import sys
from pathlib import Path
from native import ROOT
MODES=['fastest','shortest','clearance','neutral','platform','fuel','wear']
def main():
 p=argparse.ArgumentParser();p.add_argument('--quick',action='store_true');a=p.parse_args()
 output=ROOT/'data/generated/dynamics/study';output.mkdir(parents=True,exist_ok=True);cases=[]
 for vehicle in ['mclaren-p1','mcl36']:
  for mode in MODES:cases.append((f'{vehicle}-{mode}', ['--vehicle',vehicle,'--objective',mode]))
 if not a.quick:
  for nodes in [6,8]:cases.append((f'p1-grid-{nodes}',['--nodes',str(nodes)]))
  for bias in [-1,1]:cases.append((f'p1-initial-{bias}',['--initial-bias',str(bias)]))
  for fraction in [.25,.5,.75]:cases.append((f'p1-pareto-fuel-{fraction}',['--objective','custom','--weights',str(1-fraction),'0','0','0','0',str(fraction),'0']))
 summary=[]
 for name,args in cases:
  path=output/f'{name}.json';log=output/f'{name}.log'
  with log.open('w') as stream:r=subprocess.run([sys.executable,str(ROOT/'tools/dynamics/optimize.py'),*args,'--tolerance','.04','--output',str(path)],stdout=stream,stderr=subprocess.STDOUT)
  result=json.loads(path.read_text()) if path.exists() else {}
  row={'name':name,'exit_code':r.returncode,**{k:v for k,v in result.items() if k not in ['samples','commands','evaluations','initial','final','profile']}}
  summary.append(row);(output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(name,r.returncode,result.get('lap_time_s'),flush=True)
 if any(r['exit_code'] for r in summary):raise SystemExit(2)
if __name__=='__main__':main()
