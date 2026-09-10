import tempfile
import unittest
from pathlib import Path
from evaluation_cache import EvaluationCache
class CacheTests(unittest.TestCase):
 def test_exact_repeat_and_configuration_invalidation(self):
  with tempfile.TemporaryDirectory() as d:
   calls=[]
   def compute():calls.append(1);return {'objective':2}
   c=EvaluationCache(Path(d),{'config':'a','objective':[1,0]})
   self.assertEqual(c.evaluate([.1],compute),c.evaluate([.1],compute));self.assertEqual(len(calls),1)
   c=EvaluationCache(Path(d),{'config':'a','objective':[1,0]});c.evaluate([.1],compute);self.assertEqual(c.hits,1)
   for context,candidate in [({'config':'b','objective':[1,0]},[.1]),({'config':'a','objective':[0,1]},[.1]),({'config':'a','objective':[1,0]},[.10000000000000002])]:
    EvaluationCache(Path(d),context).evaluate(candidate,compute)
   self.assertEqual(len(calls),4)
 def test_rejected_candidate_cannot_replace_accepted_pair(self):
  from publication import publish_accepted
  import json
  with tempfile.TemporaryDirectory() as d:
   path=Path(d)/'policy.json'
   first=publish_accepted(path,{'accepted':True,'objective':1},{'samples':[1],'wall_s':2})
   with self.assertRaises(ValueError):publish_accepted(path,{'accepted':False},{'samples':[2]})
   self.assertEqual(json.loads(path.read_text()),first)
   second=publish_accepted(path,{'accepted':True,'objective':.9},{'samples':[2],'wall_s':3})
   self.assertNotEqual(first['reference_artifact'],second['reference_artifact'])
   self.assertEqual(json.loads((Path(d)/first['reference_artifact']).read_text())['samples'],[1])
if __name__=='__main__':unittest.main()
