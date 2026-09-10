"""Compact reproducible summary of the stabilization audits; no solver or model tuning."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
folder=ROOT/'data/generated/dynamics'
def read(name):return json.loads((folder/name).read_text())
inputs=[]
for car in ['p1','mcl36']:
 for case in read(f'm61-{car}-input-audit.json')['cases']:
  inputs.append({'vehicle':car,**{k:v for k,v in case.items() if k!='samples'}})
summary={'input':inputs,'force':{car:read(f'm61-{car}-force-audit.json') for car in ['p1','mcl36']},'replay_unchanged':read('m61-baseline-replay.json')==read('m61-final-replay.json'),'cache':read('m61-cache-summary.json'),'candidate_profile':read('m61-evaluation-profile.json')}
(folder/'m61-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
