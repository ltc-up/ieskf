import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.common import skew
from geometry_ieskf.lidar_velocity import body_twist,sensor_twist_velocity,geometry_quality,estimate_pair
from ndt_global.lidar_speed import ScanVelocity
class LidarSpeedTests(unittest.TestCase):
 def test_log_exp_constant_twist(self):
  for dt in [.05,.1]:
   for w in [np.zeros(3),np.array([.1,-.2,.8])]:
    v=np.array([50.,1.,-.2]);W=skew(w*dt);t=np.linalg.norm(w*dt);J=np.eye(3)+(.5 if not t else (1-np.cos(t))/t**2)*W+(1/6 if not t else (t-np.sin(t))/t**3)*W@W;T=np.eye(4);T[:3,:3]=Rotation.from_rotvec(w*dt).as_matrix();T[:3,3]=J@v*dt;np.testing.assert_allclose(body_twist(T,dt),np.r_[v,w],atol=1e-11)
 def test_lever_rotation(self):
  T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',[.1,.2,.3]).as_matrix();T[:3,3]=[2,.3,.5];v=np.array([30,2,.1,.1,.2,.8]);np.testing.assert_allclose(sensor_twist_velocity(v,T),T[:3,:3].T@(v[:3]+np.cross(v[3:],T[:3,3])))
 def test_invalid_interval_transform(self):
  for dt in [0,-.05,.2,np.nan]:
   with self.assertRaises(ValueError):body_twist(np.eye(4),dt)
  T=np.eye(4);T[3,0]=1
  with self.assertRaises(ValueError):body_twist(T,.05)
 def test_inverse_pair_twist(self):
  T=np.eye(4);T[:3,:3]=Rotation.from_rotvec([.02,.03,.08]).as_matrix();T[:3,3]=[2.,.1,.03];np.testing.assert_allclose(body_twist(np.linalg.inv(T),.05),-body_twist(T,.05),atol=1e-10)
 def test_no_p_or_state_mutation(self):
  T=np.eye(4);copy=T.copy();U,info=ScanVelocity.auxiliary(T,200_000_000,dict(end_ns=150_000_000,delta_world=[1,2,3]));np.testing.assert_array_equal(T,copy);np.testing.assert_allclose(U[:3,3],[.05,.1,0]);self.assertTrue(info['applied'])
 def test_expired_future_and_domain_hint(self):
  for end,v in [(0,[1,0,0]),(250_000_000,[1,0,0]),(150_000_000,[6,0,0])]:
   T,i=ScanVelocity.auxiliary(np.eye(4),200_000_000,dict(end_ns=end,delta_world=v));self.assertFalse(i['applied']);np.testing.assert_array_equal(T,np.eye(4))
 def test_conditional_quality_reject(self):
  r=dict(converged=False,candidate_valid=True)
  with self.assertRaises(ValueError):estimate_pair(r,.05)
  r.update(converged=True,pairs=2,source_n=100)
  with self.assertRaises(ValueError):estimate_pair(r,.05)
 def test_schur_coupling_and_scale(self):
  A=np.eye(6);A[0,3]=.4;H=A.T@A;g=geometry_quality(H);g2=geometry_quality(H*100);np.testing.assert_allclose(g['translation_schur_vectors'],g2['translation_schur_vectors']);self.assertAlmostEqual(g['translation_ratio'],g2['translation_ratio'])
 def test_singular_geometry_no_information_claim(self):
  H=np.diag([1,1,0,0,0,1]);g=geometry_quality(H);self.assertEqual(g['translation_ratio'],0);self.assertFalse(g['statistical_covariance'])
 def test_midpoint_history_contract(self):
  from types import SimpleNamespace as S
  def x(t,v):return S(t=t,v=np.array(v,float),R=np.eye(3),bg=np.zeros(3))
  replay=S(base=(x(0,[0,0,0]),None,None),snapshots=[(x(100,[2,4,0]),None,None)])
  v,R,b=ScanVelocity.midpoint_state(replay,50);np.testing.assert_allclose(v,[1,2,0]);np.testing.assert_allclose(R,np.eye(3))
  with self.assertRaises(ValueError):ScanVelocity.midpoint_state(replay,101)
 def test_service_failure_no_stale_hint(self):
  import io
  from collections import deque
  from unittest.mock import Mock
  a=ScanVelocity.__new__(ScanVelocity);a.proc=Mock();a.proc.stdin.write.side_effect=BrokenPipeError();a.proc.poll.return_value=0;a.log=io.StringIO();a.count=1;a.origin=100_000_000;a.failed=False;a.stride=1;a.drop_interval=None;a.enabled=True;a.hint=dict(end_ns=100_000_000,delta_world=[1,0,0]);a.past=deque([dict(ref=100_000_000,seed=np.eye(4),path='private_previous',identity='id')],maxlen=2)
  r=a.measure('private_current',150_000_000,np.eye(4),None,np.zeros(3));self.assertTrue(a.failed);self.assertIsNone(a.hint);self.assertFalse(r['qualified']);self.assertIn('service_failure',r)
 def test_injected_unavailability_clears_previous_evidence(self):
  import io
  from collections import deque
  a=ScanVelocity.__new__(ScanVelocity);a.log=io.StringIO();a.count=400;a.origin=0;a.failed=False;a.stride=1;a.drop_interval=[20,22];a.enabled=True;a.hint=dict(end_ns=19_950_000_000,delta_world=[1,0,0]);a.past=deque(maxlen=2)
  r=a.measure('current',20_000_000_000,np.eye(4),None,np.zeros(3));self.assertIsNone(a.hint);self.assertEqual(r['usage'],'INJECTED_VELOCITY_UNAVAILABLE')
 def test_feedback_transport_subtracts_existing_velocity_correction(self):
  from types import SimpleNamespace as S
  a=ScanVelocity.__new__(ScanVelocity);a.enabled=True;a.feedback_aware=True;a.hint=dict(end_ns=100_000_000,delta_world=[1,0,0],predicted_end_v=[30,0,0]);replay=S(snapshots=[(S(t=100_000_000,v=np.array([30.8,0,0])),None,None)])
  T,info=a.seed(np.eye(4),150_000_000,replay);np.testing.assert_allclose(T[:2,3],[.01,0],atol=1e-12);self.assertTrue(info['applied']);self.assertEqual(a.hint['delta_world'],[1,0,0])
 def test_feedback_transport_requires_actual_historical_endpoint(self):
  from types import SimpleNamespace as S
  a=ScanVelocity.__new__(ScanVelocity);a.enabled=True;a.feedback_aware=True;a.hint=dict(end_ns=100_000_000,delta_world=[1,0,0],predicted_end_v=[30,0,0]);T,i=a.seed(np.eye(4),150_000_000,S(snapshots=[]));self.assertFalse(i['applied']);np.testing.assert_array_equal(T,np.eye(4))
