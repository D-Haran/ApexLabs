#!/usr/bin/env python3
"""Native manual-control/off-track smoke; separate from optimization and calibration."""
import json,math
from native import Native,ROOT
rows=[]
for track in ['spa-francorchamps','monza','silverstone','suzuka','technical_test_circuit']:
 for vehicle in ['mclaren-p1','mcl36']:
  with Native(vehicle,track) as n:
   n.command(throttle=.5,steering=.1)
   for i in range(100):n.advance(.02)
   s=n.snapshot();passed=all(math.isfinite(x) for x in [*s['position'],*s['velocity']]) and abs(s['time']-2)<1e-6
   rows.append({'vehicle':vehicle,'track':track,'duration_s':s['time'],'speed_mps':math.hypot(*s['velocity']),'passed':passed})
with Native('mclaren-p1','technical_test_circuit') as n:
 n.command(throttle=.6,steering=.4)
 materials=set();max_offset=0.;airborne=0
 for i in range(1000):
  n.advance(.02);s=n.snapshot();materials.update(w['material'] for w in s['wheels']);max_offset=max(max_offset,abs(s['lateral']));airborne+=sum(not w['contact'] for w in s['wheels'])
 n.command(reset=True,throttle=0,steering=0);reset=n.snapshot()
 rows.append({'scenario':'manual curved input leaves track without teleport; rough grass; reset','materials':sorted(materials),'max_abs_offset_m':max_offset,'airborne_wheel_samples':airborne,'reset_time_s':reset['time'],'passed':len(materials)>1 and reset['time']==0})
(ROOT/'data/generated/dynamics/driving-validation.json').write_text(json.dumps(rows,indent=2)+'\n')
print(json.dumps(rows,indent=2))
if not all(r['passed'] for r in rows):raise SystemExit(2)
