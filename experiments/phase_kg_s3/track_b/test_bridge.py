import unittest
import numpy as np
from bridge import Validator
from local_odometry import *
from glim_diagnostics import surface_mask,multires_support

class Tests(unittest.TestCase):
 def test_validator_support_counts_distinct_points(self):
  v=Validator();v.dt_s=.05
  q=dict(source=np.zeros((1000,3)),initial_guess=np.eye(4));r=dict(O_T_base=np.eye(4).tolist(),candidate_valid=True,converged=True,target_valid_voxels=30,neighbor_points=101)
  self.assertTrue(v(q,r));r['neighbor_points']=1001
  self.assertFalse(v(q,r));self.assertIn('invalid_distinct_source_count',r['validation']['reasons'])
 def test_true_dt_and_prediction_relative_gate(self):
  v=Validator();v.dt_s=.1
  pred=np.eye(4);pred[0,3]=100
  candidate=pred.copy();candidate[0,3]+=1.5
  q=dict(source=np.zeros((100,3)),initial_guess=pred);r=dict(O_T_base=candidate.tolist(),candidate_valid=True,converged=True,target_valid_voxels=20,neighbor_points=100)
  self.assertTrue(v(q,r));v.dt_s=.05;self.assertFalse(v(q,r))
 def test_signed_surface_not_abs_normal_test(self):
  p=np.array([[2.,0,0],[2,0,0]]);n=np.array([[1.,0,0],[-1,0,0]])
  self.assertEqual(surface_mask(p,n,np.eye(4)).tolist(),[False,True])
  d=multires_support(p,n,p,np.eye(4));self.assertEqual(d['levels'][0]['support_with_surface'],1)
 def test_pure_prediction_never_builds_target(self):
  o=LocalOdometry(Config(mode='B-P'));s=lambda i:Scan.create(i,i*10,i*10+1,[[0,0,0]],[[0,0,0]],'test')
  o.step(s(1),np.eye(4),None,None)
  o._target=lambda *a: (_ for _ in ()).throw(AssertionError('target called'))
  r=o.step(s(2),np.eye(4),None,None);self.assertEqual(r['target_points'],0);self.assertEqual(o.history,[])


class MultiresTests(unittest.TestCase):
 def test_same_arrays_and_fine_failure_not_coarse_fallback(self):
  from bridge import MultiresMatcher
  requests=[]
  def fake(q):
   requests.append(q)
   T=np.eye(4);T[0,3]=.2
   if len(requests)==1:return dict(candidate_valid=True,converged=True,O_T_base=T.tolist(),target_build_ms=1,align_wall_ms=2)
   return dict(candidate_valid=False,converged=False,error='fine failed',target_build_ms=3,align_wall_ms=4)
  a,b=np.zeros((2,3)),np.ones((3,3));q=dict(source=a,target=b,initial_guess=np.eye(4))
  r=MultiresMatcher(fake)(q)
  self.assertFalse(r['candidate_valid']);self.assertNotIn('O_T_base',r)
  self.assertEqual([x['resolution'] for x in requests],[3,1.5])
  self.assertTrue(all(x['source'] is a and x['target'] is b for x in requests))
  self.assertEqual(r['multiresolution']['total_target_build_ms'],4)
  np.testing.assert_equal(q['initial_guess'],np.eye(4))

 def test_mr_keyframe_policy_matches_BK(self):
  a,b=LocalOdometry(Config(mode='B-K')),LocalOdometry(Config(mode='B-K-MR'))
  for i in range(1,5):
   s=Scan.create(i,i*10,i*10+1,[[0,0,0]],[[0,0,0]],'test')
   for o in [a,b]:o.step(s,np.eye(4),lambda q:{'O_T_base':np.eye(4).tolist()},lambda q,r:True)
  self.assertEqual([x.scan.frame_id for x in a.history],[x.scan.frame_id for x in b.history])

if __name__=='__main__':unittest.main()
