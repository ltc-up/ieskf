"""Focused tests for the new NDT XY/yaw observation interface."""
import unittest,numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.state import State,inject,chart_map,difference
from geometry_ieskf.ndt_pose_observation import NdtPoseObservation,engineering_R,pose_xyyaw,wrap
class NdtPoseTests(unittest.TestCase):
 def setUp(self):
  self.x=State(np.array([1.,2.,3.]),np.array([.2,.3,.1]),Rotation.from_euler('xyz',[.2,-.1,2.]).as_matrix(),np.zeros(3),np.zeros(3),42);self.obs=NdtPoseObservation([.4,-.2,.1]);g=np.random.default_rng(10).normal(size=(15,15));self.P=(g@g.T+np.eye(15))*.0001;self.R=np.diag([.03,.03,.001])
 def test_full_right_error_jacobian(self):
  h,H=self.obs.model(self.x);fd=np.zeros_like(H)
  for k in range(15):
   d=np.eye(15)[k]*1e-6;a=self.obs.model(inject(self.x,d))[0];b=self.obs.model(inject(self.x,-d))[0];v=a-b;v[2]=wrap(v[2]);fd[:,k]=v/2e-6
  np.testing.assert_allclose(H,fd,atol=2e-9)
 def test_base_reference_lever(self):
  h,_=self.obs.model(self.x);np.testing.assert_allclose(h[:2],(self.x.p-self.x.R@self.obs.lever)[:2])
 def test_yaw_wrap(self):
  self.x.R=Rotation.from_euler('z',np.pi-.001).as_matrix();z=self.obs.model(self.x)[0];z[2]=-np.pi+.001;_,_,i=self.obs.correct(self.x,self.P,z,self.R);self.assertAlmostEqual(i['innovation'][2],.002)
 def test_joseph_information_equivalence_and_reset(self):
  h,H=self.obs.model(self.x);x,P,i=self.obs.correct(self.x,self.P,h+[.01,-.01,.001],self.R);self.assertTrue(i['accepted']);expected=np.linalg.solve(np.linalg.solve(self.P,np.eye(15))+H.T@np.linalg.solve(self.R,H),np.eye(15));B=chart_map(np.array(i['delta']));np.testing.assert_allclose(P,B@expected@B.T,rtol=2e-12,atol=1e-15)
 def test_full_cross_can_update_vertical_and_bias(self):
  h,_=self.obs.model(self.x);x,P,i=self.obs.correct(self.x,self.P,h+[.02,-.01,.001],self.R);d=np.array(i['delta']);self.assertGreater(abs(d[2])+abs(d[5])+np.linalg.norm(d[9:]),0);self.assertEqual(i['covariance_updates'],1)
 def test_unsafe_proposal_atomic_rejection(self):
  h,_=self.obs.model(self.x);x,P,i=self.obs.correct(self.x,self.P,h+[100.,0.,0.],self.R);self.assertFalse(i['accepted']);np.testing.assert_array_equal(P,self.P);np.testing.assert_array_equal(difference(x,self.x),np.zeros(15))
 def test_invalid_R_atomic_rejection(self):
  h,_=self.obs.model(self.x);x,P,i=self.obs.correct(self.x,self.P,h,np.diag([1.,1.,-1.]));self.assertFalse(i['accepted']);np.testing.assert_array_equal(P,self.P)
 def test_legacy_piecewise_R(self):
  np.testing.assert_allclose(np.diag(engineering_R(.2)),[.05,.05,.00025]);np.testing.assert_allclose(np.diag(engineering_R(3.)),[22.5,22.5,9.])
 def test_invalid_fitness(self):
  for f in [-1.,np.nan,16.]:
   with self.assertRaises(ValueError):engineering_R(f)
 def test_existing_publisher_floor(self):
  np.testing.assert_allclose(np.diag(engineering_R(0.)),[2.5e-6,2.5e-6,1.25e-8])
 def test_zero_residual_no_state_change_valid_P(self):
  h,_=self.obs.model(self.x);x,P,i=self.obs.correct(self.x,self.P,h,self.R);self.assertTrue(i['accepted']);np.testing.assert_allclose(difference(x,self.x),0,atol=1e-15);self.assertGreater(np.linalg.eigvalsh(P).min(),0)
if __name__=='__main__':unittest.main()
