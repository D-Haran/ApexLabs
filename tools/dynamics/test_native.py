import math
import os
import time
import unittest
from native import Native
class NativeTests(unittest.TestCase):
 def test_deterministic_input_and_reset(self):
  outputs=[]
  with Native(track='technical_test_circuit') as n:
   for _ in range(2):
    n.command(reset=True,throttle=.4,steering=.15,steeringRate=1.)
    n.advance(.002);self.assertAlmostEqual(n.snapshot()['steering'],.002)
    for _ in range(49):n.advance(.02)
    outputs.append(n.snapshot())
   self.assertEqual(outputs[0],outputs[1]);self.assertGreater(outputs[0]['s'],0)
   n.command(assisted=True);n.advance(.01);self.assertTrue(n.snapshot()['assisted'])
 def test_friendly_input_is_separate_and_rate_limited(self):
  with Native(track='technical_test_circuit') as n:
   n.command(reset=True,speed=30,keyboardFriendly=True,steerKey=1,throttle=1,brake=0)
   n.advance(.002);a=n.snapshot()
   self.assertAlmostEqual(a['steerCommand'],.003)
   self.assertAlmostEqual(a['throttle'],.003)
   self.assertLess(a['steering'],.001)
   self.assertLessEqual(abs(a['steeringRateActual']),1.4+1e-10)
   for _ in range(20):n.advance(.02)
   a=n.snapshot();self.assertLess(a['steering'],.03)
   n.command(keyboardFriendly=False,steering=.65,throttle=1)
   n.advance(.02);b=n.snapshot();self.assertEqual(b['throttle'],1)
   self.assertGreater(b['steering'],a['steering'])
   n.command(reset=True);self.assertEqual(n.snapshot()['steering'],0)
 def test_friendly_reset_repeats(self):
  with Native(track='technical_test_circuit') as n:
   rows=[]
   for _ in range(2):
    n.command(reset=True,speed=20,keyboardFriendly=True,steerKey=1,throttle=.5)
    for _ in range(20):n.advance(.02)
    rows.append(n.snapshot())
   self.assertEqual(rows[0],rows[1])
 def test_limits(self):
  with Native() as n:
   for command in ({'throttle':2},{'steering':2},{'steeringRate':0}):
    with self.assertRaises(RuntimeError):n.command(**command)
 def test_real_track_pose_and_runtime(self):
  with Native() as n:
   m=n.snapshot(True);s=n.snapshot();p=m['geometry'][0]
   self.assertLess(abs(s['position'][0]-p['x_m']),1)
   self.assertGreater(s['position'][2],p['elevation_m'])
   start=time.perf_counter();n.command(throttle=.2)
   for _ in range(100):n.advance(.01)
   elapsed=time.perf_counter()-start
   if os.environ.get('APEXLAB_PERFORMANCE_ASSERT')=='1':self.assertLess(elapsed,1.,'release native core slower than real time')
   s=n.snapshot();self.assertTrue(all(math.isfinite(x) for x in s['position']))
if __name__=='__main__':unittest.main()
