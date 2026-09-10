import unittest
from native import Native
class ShootingTests(unittest.TestCase):
 def test_profile_bounds_and_independent_repeat(self):
  with Native(track='technical_test_circuit') as n:
   with self.assertRaises(RuntimeError):n.rollout({'offsets':[0]*3,'scales':[1]*3})
   with self.assertRaises(RuntimeError):n.rollout({'offsets':[0]*4,'scales':[3]*4})
   r=n.rollout({'offsets':[0]*4,'scales':[1]*4,'record':True,'warmup':0})
   self.assertTrue(r['complete']);self.assertGreater(r['minimum_clearance_m'],0)
   replay=n.rollout({**r,'replay_dt':.002},replay=True)
   self.assertEqual(len(r['samples']),len(replay['samples']))
   for a,b in zip(r['samples'],replay['samples']):
    self.assertEqual(a['position'],b['position'])
    self.assertEqual(a['wheels'],b['wheels'])
if __name__=='__main__':unittest.main()
