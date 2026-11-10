import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.optical_velocity import OpticalVelocity
from geometry_ieskf.state import State,inject
class OpticalTests(unittest.TestCase):
 def setUp(self):
  self.x=State(np.ones(3),np.array([20.,3.,.5]),Rotation.from_rotvec([.1,-.03,.5]).as_matrix(),np.array([.001,.002,.003]),np.zeros(3),1);T=np.eye(4);T[:3,:3]=Rotation.from_rotvec([.03,.01,-.02]).as_matrix();T[:3,3]=[3.,.2,.4];self.o=OpticalVelocity([1.,.1,.2],T,(0,1));self.w=np.array([.01,.02,.3])
 def test_full_H(self):
  h,H=self.o.model(self.x,self.w);F=np.zeros_like(H);eps=1e-6
  for j in range(15):
   d=np.eye(15)[j]*eps;F[:,j]=(self.o.model(inject(self.x,d),self.w)[0]-self.o.model(inject(self.x,-d),self.w)[0])/(2*eps)
  np.testing.assert_allclose(H,F,atol=5e-9)
 def test_joseph_full_cross_and_reset(self):
  rng=np.random.default_rng(3);L=rng.normal(size=(15,15))*.01;P=L@L.T+np.eye(15)*.01;h,H=self.o.model(self.x,self.w);R=np.diag([.26,.26]);z=h+[.1,-.2];y,Q,r=self.o.correct(self.x,P,self.w,z,R);self.assertTrue(r['accepted']);np.testing.assert_allclose(r['delta'],np.array(r['K'])@np.array(r['innovation']),atol=1e-14);self.assertGreater(np.linalg.eigvalsh(Q).min(),0);np.testing.assert_allclose(Q,Q.T,atol=1e-15)
 def test_nonobserved_z_is_not_zero_velocity(self):
  T=np.eye(4);o=OpticalVelocity([0,0,0],T,(1,));x=State(np.zeros(3),np.array([10.,2.,3.]),np.eye(3),np.zeros(3),np.zeros(3));y,Q,r=o.correct(x,np.eye(15),np.zeros(3),[10.,2.],np.eye(2));self.assertTrue(r['accepted']);self.assertAlmostEqual(y.v[2],3);self.assertAlmostEqual(Q[5,5],1)
 def test_status_rejection_atomic(self):
  P=np.eye(15);y,Q,r=self.o.correct(self.x,P,self.w,[0,0],np.eye(2),False);self.assertFalse(r['accepted']);np.testing.assert_array_equal(y.v,self.x.v);np.testing.assert_array_equal(Q,P)
 def test_scalar_axis_sign_and_lever(self):
  o=OpticalVelocity([0,0,0],np.array([[1,0,0,3],[0,1,0,0],[0,0,1,0],[0,0,0,1]]),(1,));x=State(np.zeros(3),np.array([10.,1.,0.]),np.eye(3),np.zeros(3),np.zeros(3));h,H=o.model(x,np.array([0,0,.2]));self.assertAlmostEqual(h[0],1.6);self.assertAlmostEqual(H[0,11],-3)

class OpticalReplayTests(unittest.TestCase):
 def replay(self,policy='optical_lateral'):
  from ndt_global.optical_replay import OpticalReplay
  x=State(np.zeros(3),np.array([10.,1.,0.]),np.eye(3),np.zeros(3),np.zeros(3),1);P=np.eye(15)*.01;held=dict(header_ns=1,omega=np.zeros(3),force=np.array([0,0,9.80665]));r=OpticalReplay(x,P,held,np.zeros(3));r.policy=policy;r.optical=OpticalVelocity([0,0,0],np.eye(4),(0,1) if policy=='optical_xy' else (1,));return r
 def test_late_optical_equivalence_and_duplicate(self):
  import copy
  a=self.replay();b=self.replay();e=dict(kind='optical',header_ns=5_000_001,message_id=99,velocity=[10.,.8],variance=[[.26,0],[0,.26]],status_good=True)
  for i in range(1,10):
   m=dict(kind='imu',header_ns=i*1_000_000+1,message_id=i,omega=np.zeros(3),force=np.array([0,0,9.80665]));a.ingest(copy.deepcopy(m));b.ingest(copy.deepcopy(m))
   if i==5:a.ingest(copy.copy(e))
  b.ingest(e);self.assertTrue(e['last_update']['accepted']);np.testing.assert_allclose(a.current()[0].v,b.current()[0].v,atol=1e-14);np.testing.assert_allclose(a.current()[1],b.current()[1],atol=1e-14);self.assertEqual(b.ingest(e),'DUPLICATE')
 def test_forward_source_substitution_and_timeout(self):
  r=self.replay('optical_xy');r.ingest(dict(kind='optical',header_ns=1_000_001,message_id=1,velocity=[10.,1.],variance=[[.26,0],[0,.26]],status_good=True));w=dict(kind='wheel',header_ns=2_000_001,message_id=2,speed=10.,variance=.01);r.ingest(w);self.assertEqual(w['last_update']['reason'],'ALTERNATIVE_OPTICAL_SOURCE_ACTIVE')
  for i in range(3,110):r.ingest(dict(kind='imu',header_ns=i*1_000_000+1,message_id=100+i,omega=np.zeros(3),force=np.array([0,0,9.80665])))
  w=dict(kind='wheel',header_ns=109_000_002,message_id=999,speed=10.,variance=.01);r.ingest(w);self.assertNotEqual(w['last_update']['reason'],'ALTERNATIVE_OPTICAL_SOURCE_ACTIVE')

class MountingCalibrationTests(unittest.TestCase):
 def test_augmented_jacobian_and_unobserved_gauge(self):
  from geometry_ieskf.optical_calibration import CalibratedOptical,lift,inject16
  x=lift(State(np.zeros(3),np.array([20.,1.,.3]),Rotation.from_rotvec([.02,.03,.1]).as_matrix(),np.zeros(3),np.zeros(3)),.01);T=np.eye(4);T[:3,3]=[3.,0,.2];o=CalibratedOptical([1,0,0],T,(0,1));w=np.array([0.,0.,.3]);h,H=o.model(x,w);D=np.zeros_like(H)
  for j in range(16):
   e=np.eye(16)[j]*1e-6;D[:,j]=(o.model(inject16(x,e),w)[0]-o.model(inject16(x,-e),w)[0])/2e-6
  np.testing.assert_allclose(H,D,atol=4e-9)
  x=lift(State(np.zeros(3),np.array([20.,1.,0]),np.eye(3),np.zeros(3),np.zeros(3)));o=CalibratedOptical([0,0,0],np.eye(4),(0,1));_,H=o.model(x,np.zeros(3));g=np.zeros(16);g[8]=1;g[15]=-1;np.testing.assert_allclose(H@g,0,atol=1e-12)
 def test_static_parameter_full_cross_covariance(self):
  from geometry_ieskf.optical_calibration import CalibratedPredictor,lift
  x=lift(State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3)),.01);P=np.eye(16)*.01;P[0,15]=P[15,0]=.002;y,Q=CalibratedPredictor().predict(x,P,np.zeros(3),[0,0,9.80665],.01);self.assertEqual(y.beta,.01);self.assertAlmostEqual(Q[15,15],P[15,15]);self.assertNotEqual(Q[0,15],0)

class OpticalInputTests(unittest.TestCase):
 def test_invalid_transform_and_duplicate_axes(self):
  for T in [np.eye(3),np.diag([-1,1,1,1]),np.full((4,4),np.nan)]:
   with self.assertRaises(ValueError):OpticalVelocity([0,0,0],T)
  with self.assertRaises(ValueError):OpticalVelocity([0,0,0],np.eye(4),(1,1))
 def test_augmented_joseph_independent_reference(self):
  from geometry_ieskf.optical_calibration import lift,correct16
  from geometry_ieskf.state import chart_map
  rng=np.random.default_rng(42);L=rng.normal(size=(16,16))*.01;P=L@L.T+np.eye(16)*.01;H=rng.normal(size=(2,16));R=np.eye(2)*.26;r=np.array([.001,-.002]);x=lift(State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3)))
  y,Q,info=correct16(x,P,H,r,R);K=P@H.T@np.linalg.inv(H@P@H.T+R);d=K@r;A=np.eye(16)-K@H;B=np.eye(16);B[:15,:15]=chart_map(d[:15]);reference=B@(A@P@A.T+K@R@K.T)@B.T
  np.testing.assert_allclose(Q,reference,atol=2e-17);np.testing.assert_allclose(info['delta'],d,atol=1e-17);self.assertGreater(np.linalg.eigvalsh(Q).min(),0)

 def test_augmented_invalid_covariance_rejected(self):
  from geometry_ieskf.optical_calibration import lift,correct16
  x=lift(State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3)))
  with self.assertRaises(ValueError):correct16(x,np.eye(16),np.zeros((2,16)),np.zeros(2),np.array([[1.,.2],[0.,1.]]))
