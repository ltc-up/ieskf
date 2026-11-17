"""Fast result/schema checks. Does not read or reconstruct sensor payloads."""
import json,unittest
from pathlib import Path
P=Path(__file__).resolve().parent
class ContractTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.d=json.load(open(P/'aggregate_results.json'))
 def test_native_math(self):self.assertTrue(self.d['tests']['all_pass'])
 def test_read_only_equivalence(self):self.assertTrue(self.d['diagnostic_equivalence']['pass_within_tolerance'])
 def test_matrix(self):
  self.assertEqual(len(self.d['results']),30)
  self.assertEqual({(r['series'],r['E'])for r in self.d['results']},{(s,e)for s in ['fault','normal8','right3','ordinary','large_proxy','pass713']for e in range(5)})
 def test_full_P(self):
  for q in self.d['P_status']:
   self.assertTrue(q['all_finite']);self.assertGreater(q['min_eigen'],0);self.assertLess(q['max_asym'],1e-8)
 def test_no_timing_certification(self):
  q=next(q for q in self.d['performance']if q['id']=='fault_E1');self.assertEqual(q['over50'],100);self.assertGreater(q['step']['p95'],50)
 def test_all_reference_offsets(self):
  for r in self.d['results']:self.assertEqual(set(r['by_lag']),{'0.0','0.075','0.15'})
 def test_native_R_not_calibrated(self):
  self.assertFalse(self.d['conditional_R']['calibrated']);self.assertEqual(self.d['conditional_R']['missing_actual_baseline_R'],7)
 def test_no_fake_reference_timing(self):
  for q in self.d['existing_15d_prediction_reference']:self.assertEqual(q['timing'],'NOT_MEASURED')
if __name__=='__main__':unittest.main()
