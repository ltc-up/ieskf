import sys, unittest
from pathlib import Path
import numpy as np
sys.path[:0]=[str(Path(__file__).resolve().parents[3]/'src'),str(Path(__file__).parent)]
from strategy import align_admission, ReplayStrategyFactory, RejectedAlignObserver
from score_map import FrozenMap
import test_score_map as base_tests

class StrategyTest(unittest.TestCase):
 def test_A1_invalid_flags_and_SE3_return_prediction_without_source_IO(self):
  t=base_tests.MathTest();t.setUp();factory=object.__new__(ReplayStrategyFactory);factory.strategy='A1';factory.lever=t.lever
  examples=[dict(converged=False,candidate_valid=True,final=np.eye(4).tolist()),dict(converged=True,candidate_valid=False,final=np.eye(4).tolist()),dict(converged=True,candidate_valid=True,final=np.full((4,4),np.nan).tolist()),dict(converged=True,candidate_valid=True,final=np.zeros((4,4)).tolist())]
  for result in examples:
   obs=factory.create_observer('/must_not_be_opened','source','target',[],None,result)
   self.assertIsInstance(obs,RejectedAlignObserver)
   y,Q,info=obs.correct(t.x,t.P,np.eye(4),None)
   self.assertEqual(info['reason'],'A1_MATCHER_INVALID');self.assertEqual(info['covariance_updates'],0)
   np.testing.assert_array_equal(Q,t.P);np.testing.assert_array_equal(y.p,t.x.p)
 def test_A1_finite_rigid_pose_accepted_for_geometry(self):
  self.assertEqual(align_admission(dict(converged=True,candidate_valid=True,final=np.eye(4).tolist())),(True,None))
 def test_A2_unavailable_nonfinite_align_does_not_destroy_prior_branches(self):
  t=base_tests.MathTest();t.setUp();f=FrozenMap(t.x,t.P,t.lever,t.evaluate)
  y,Q,info=f.solve(np.full((4,4),np.nan),align_valid=False,align_failure='ALIGN_CANDIDATE_INVALID')
  self.assertTrue(info['accepted']);self.assertIsNone(info['original_ndt_delta'])
  self.assertIn(info['selected'],['prior','prior_refined'])
  failed=[c for c in info['candidates'] if c.get('failure')]
  self.assertEqual(len(failed),2);self.assertTrue(all(c['failure']=='ALIGN_CANDIDATE_INVALID' for c in failed))

if __name__=='__main__':unittest.main()
