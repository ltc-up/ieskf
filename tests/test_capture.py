import unittest
import numpy as np
from geometry_ieskf.capture import step_margins,step_jacobian,starting_seeds,AlternatingCapture
from geometry_ieskf.partition import margins
from geometry_ieskf.state import State
from geometry_ieskf.geometry import GeometryFactor
class ToyFactors:
 def __init__(self,good=True,shift=0.):
  self.p=np.array([[0.,0.,0.],[0.,1.,0.],[1.,0.,0.],[0.,0.,1.],[1.,1.,0.],[1.,0.,1.]])
  self.n=np.array([[1.,0,0],[0,1.,0],[0,0,1.],[1.,0,0],[0,1.,0],[0,0,1.]])
  self.q=self.p+np.array([shift,0,0]);self.good=good
 def get(self,x,source,factor):
  return {'p':self.p,'q':self.q,'n':self.n,'w':np.ones(6),'source_index':np.arange(6),'target_ids':np.arange(6),'good':self.good,'metrics':{'support_n':6}}
class CaptureTests(unittest.TestCase):
 def setUp(self):
  self.x=State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3));self.P=np.eye(15)*.001;self.factor=GeometryFactor(np.zeros(3))
 def test_trust_constraint_jacobian_with_nonzero_anchor(self):
  z=np.arange(15)*.001;a=z*.2;h=1e-7;num=np.column_stack([(step_margins(z+np.eye(15)[i]*h,a)-step_margins(z-np.eye(15)[i]*h,a))/(2*h) for i in range(15)])
  np.testing.assert_allclose(num,step_jacobian(z,a),atol=1e-8)
 def test_predetermined_seeds_feasible_and_white_bounded(self):
  P=np.eye(15);P[0,3]=P[3,0]=.2;ss=starting_seeds(P);self.assertEqual(len(ss),13)
  for s in ss:self.assertLessEqual(s['white_radius'],.250000000001);self.assertGreaterEqual(margins(s['z']).min(),.9375-1e-12)
 def test_seed_symmetry_and_zero_nominal(self):
  ss=starting_seeds(self.P);np.testing.assert_array_equal(ss[0]['z'],np.zeros(15))
  for i in range(1,13,2):np.testing.assert_allclose(ss[i]['z'],-ss[i+1]['z'])
 def test_no_budget_is_not_a_convergence_label(self):
  r=AlternatingCapture(self.factor,max_outer=0).run(self.x,self.P,np.zeros((1,3)),ToyFactors())
  self.assertEqual(r['status'],'OUTER_BUDGET_EXHAUSTED');self.assertFalse(r['interior_qualified'])
 def test_fixed_geometry_stationarity_and_identity_certified(self):
  r=AlternatingCapture(self.factor,max_outer=2).run(self.x,self.P,np.zeros((1,3)),ToyFactors())
  self.assertTrue(r['interior_qualified']);self.assertEqual(r['covariance_updates'],0)
 def test_low_support_not_promoted(self):
  r=AlternatingCapture(self.factor).run(self.x,self.P,np.zeros((1,3)),ToyFactors(good=False));self.assertEqual(r['status'],'LOW_SUPPORT');self.assertFalse(r['interior_qualified'])
 def test_prior_state_and_P_never_mutated(self):
  before=self.P.copy();p=self.x.p.copy();r=AlternatingCapture(self.factor,max_outer=4).run(self.x,self.P,np.zeros((1,3)),ToyFactors(shift=.01))
  np.testing.assert_array_equal(self.P,before);np.testing.assert_array_equal(self.x.p,p);self.assertEqual(r['covariance_updates'],0)
 def test_infeasible_seed_rejected_not_clipped(self):
  z=np.zeros(15);z[0]=3
  with self.assertRaises(ValueError):AlternatingCapture(self.factor).run(self.x,self.P,np.zeros((1,3)),ToyFactors(),z)
if __name__=='__main__':unittest.main()
