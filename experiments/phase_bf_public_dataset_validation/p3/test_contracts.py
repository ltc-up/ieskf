"""No-CUDA tests. Optional P3_RECEIPT selects completed measurements only."""
import importlib.util,json,os,unittest
from pathlib import Path
import numpy as np

def module(name,file):
 s=importlib.util.spec_from_file_location(name,Path(__file__).with_name(file));m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
run=module('p3run','run_pairs.py');analysis=module('p3analysis','analyze_pairs.py')
class ContractTests(unittest.TestCase):
 def test_bounded_scope(self):self.assertEqual(run.FRAMES,(60,65,76));self.assertEqual(run.REPEATS,3)
 def test_patch_fails_closed(self):
  with self.assertRaises(ValueError):run.patch('unexpected template',lambda s:s)
 def test_small_statistics(self):self.assertEqual(analysis.stats([2.,4.])['median'],3.)
 @unittest.skipUnless(os.environ.get('P3_RECEIPT'),'needs completed private receipt')
 def test_complete_receipt_and_no_fusion(self):
  r=json.loads(Path(os.environ['P3_RECEIPT']).read_text());self.assertEqual(r['calls'],18);self.assertTrue(r['no_state_P_commit'] and r['no_R_construction'] and r['no_replay'])
 @unittest.skipUnless(os.environ.get('P3_RECEIPT'),'needs completed private receipt')
 def test_off_missing_and_native_spd(self):
  r=json.loads(Path(os.environ['P3_RECEIPT']).read_text())
  for row in r['records']:
   z=row['result'];self.assertFalse(z['legacy_R_generated']);self.assertGreater(np.linalg.eigvalsh(np.asarray(z['native_internal'])).min(),0.)
   if row['mode']=='OFF':self.assertIsNone(z['fitness']);self.assertIsNone(z['fitness_complete']);self.assertIsNone(z['p2_pcl_fit_ms']);self.assertFalse(z['fitness_available'])
 @unittest.skipUnless(os.environ.get('P3_RECEIPT'),'needs completed private receipt')
 def test_existing_equivalence_and_additive_timing(self):
  rows,s=analysis.analyze(os.environ['P3_RECEIPT']);self.assertEqual(len(rows),18);self.assertEqual(s['maximum_historical_numeric_difference'],0.);self.assertTrue(s['all_equivalent']);self.assertTrue(all(x['ipc_serialization_residual_ms']>=0 for x in rows))
if __name__=='__main__':unittest.main()
