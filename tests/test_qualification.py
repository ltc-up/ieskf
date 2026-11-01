import unittest
import numpy as np
from geometry_ieskf.state import State
from geometry_ieskf.qualification import eligible,pair_retention,atomic_select,local_optimality
class QualificationTests(unittest.TestCase):
 def setUp(self):
  self.tol=1e-4;self.good={'gn_white_norm':1e-7,'newton_decrement':1e-7,'association_witness':True,'support_good':True,'engineering_safe':True}
  self.x=State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3))
 def test_target_set_alone_is_insufficient(self):
  a={'source_index':[0,1],'target_ids':[4,5]};b={'source_index':[0,1],'target_ids':[5,4]}
  self.assertFalse(pair_retention(a,b)['identical'])
 def test_tiny_applied_step_is_not_stationarity(self):
  m=dict(self.good,gn_white_norm=.1);m['last_applied_step']=1e-12
  self.assertFalse(eligible(m,self.tol)['accepted'])
 def test_dual_gradient_required(self):
  self.assertFalse(eligible(dict(self.good,newton_decrement=.1),self.tol)['accepted'])
 def test_reject_preserves_both_objects(self):
  y=self.x.copy();y.p[:]=1;P=np.eye(15);Q=P*.1
  x,S,d=atomic_select(self.x,P,y,Q,eligible(dict(self.good,association_witness=False),self.tol))
  np.testing.assert_array_equal(x.p,self.x.p);np.testing.assert_array_equal(S,P);self.assertFalse(d['accepted'])
 def test_invalid_accepted_candidate_rolls_back(self):
  y=self.x.copy();y.v[2]=np.nan
  x,P,d=atomic_select(self.x,np.eye(15),y,np.eye(15),eligible(self.good,self.tol))
  self.assertFalse(d['accepted']);self.assertTrue(np.isfinite(x.v).all());np.testing.assert_array_equal(P,np.eye(15))
 def test_accept_keeps_matching_covariance(self):
  y=self.x.copy();y.p[2]=.1
  x,P,d=atomic_select(self.x,np.eye(15),y,np.eye(15)*.5,eligible(self.good,self.tol))
  self.assertTrue(d['accepted']);self.assertEqual(x.p[2],.1);self.assertEqual(P[2,2],.5)
 def test_untruncated_gradient_matches_numeric_objective(self):
  rng=np.random.default_rng(2);A=rng.normal(size=(15,15));P=A@A.T+np.eye(15);L=np.linalg.cholesky(P);J=rng.normal(size=(6,15));w=np.arange(1,7.);z=rng.normal(size=15)*.01;r0=rng.normal(size=6)
  def f(u):
   zz=L@u;rr=r0+J@zz;return .5*u@u+.5*np.sum(w*rr*rr)
  u=np.linalg.solve(L,z);h=1e-6;g=np.array([(f(u+np.eye(15)[i]*h)-f(u-np.eye(15)[i]*h))/(2*h) for i in range(15)])
  m=local_optimality(P,z,J,r0+J@z,w);self.assertAlmostEqual(np.linalg.norm(g),m['gradient_white_norm'],places=7)
 def test_nonfinite_metrics_rejected(self):
  self.assertFalse(eligible(dict(self.good,gn_white_norm=np.nan),self.tol)['accepted'])
if __name__=='__main__':unittest.main()
