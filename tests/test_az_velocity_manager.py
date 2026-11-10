import unittest,copy
import numpy as np
from geometry_ieskf.state import State
from geometry_ieskf.optical_velocity import OpticalVelocity
from ndt_global.velocity_manager import ManagedVelocityReplay
class VelocityManagerTests(unittest.TestCase):
 def replay(self):
  x=State(np.zeros(3),np.array([10.,.4,0.]),np.eye(3),np.zeros(3),np.zeros(3),1);r=ManagedVelocityReplay(x,np.eye(15)*.01,dict(header_ns=1,omega=np.zeros(3),force=np.array([0.,0.,9.80665])),np.zeros(3));r.optical=OpticalVelocity(np.zeros(3),np.eye(4),(0,));return r
 def imu(self,t):return dict(kind='imu',header_ns=t,message_id=t,omega=np.zeros(3),force=np.array([0.,0.,9.80665]))
 def opt(self,t,v=10.,good=True):return dict(kind='optical',header_ns=t,message_id=t+1,velocity=[v,.4],variance=[[.26,0],[0,.26]],status_good=good)
 def warm(self,r):
  for i in range(1,13):
   t=i*10_000_000+1;r.ingest(self.imu(t));r.ingest(self.opt(t))
 def test_recovery_exclusive_and_immediate_fault(self):
  r=self.replay();self.warm(r);w=dict(kind='wheel',header_ns=121_000_001,message_id=991,speed=10.,variance=.01);r.ingest(w);self.assertEqual(w['last_update']['reason'],'OPTICAL_FORWARD_EXCLUSIVE');e=self.opt(122_000_001,good=False);r.ingest(e);w=dict(kind='wheel',header_ns=123_000_001,message_id=992,speed=10.,variance=.01);r.ingest(w);self.assertTrue(w['last_update']['accepted'])
 def test_dynamic_fault_valid_status(self):
  r=self.replay();self.warm(r);e=self.opt(121_000_001,25.);r.ingest(e);self.assertEqual(e['last_update']['reason'],'OPTICAL_DYNAMIC_JUMP')
 def test_no_zero_lateral_or_z_constraint(self):
  r=self.replay();self.warm(r);x,P,_=r.current();self.assertAlmostEqual(x.v[1],.4);self.assertEqual(r.optical.model(x,np.zeros(3))[1].shape,(1,15));self.assertGreater(np.linalg.eigvalsh(P).min(),0)
 def test_delayed_replay_metadata_equivalence(self):
  a=self.replay();b=self.replay();late=self.opt(111_000_001,good=False)
  for i in range(1,15):
   t=i*10_000_000+1;a.ingest(self.imu(t));b.ingest(self.imu(t));a.ingest(self.opt(t));b.ingest(self.opt(t))
   if i==11:a.ingest(copy.deepcopy(late))
  b.ingest(late);np.testing.assert_allclose(a.current()[0].v,b.current()[0].v,atol=1e-14);np.testing.assert_allclose(a.current()[1],b.current()[1],atol=1e-14);self.assertEqual(a.current()[2]['velocity_manager'],b.current()[2]['velocity_manager'])
 def test_stale_timeout_returns_wheel(self):
  r=self.replay();self.warm(r)
  for i in range(13,24):r.ingest(self.imu(i*10_000_000+1))
  w=dict(kind='wheel',header_ns=231_000_001,message_id=999,speed=10.,variance=.01);r.ingest(w);self.assertTrue(w['last_update']['accepted'])

 def test_one_fresh_packet_cannot_clear_fault_and_duplicate(self):
  r=self.replay();self.warm(r);r.ingest(self.opt(121_000_001,good=False));e=self.opt(122_000_001);r.ingest(e);self.assertFalse(e['last_update']['accepted']);self.assertEqual(e['last_update']['source_phase'],'RECOVERING');self.assertEqual(r.ingest(e),'DUPLICATE')

 def test_stale_wheel_cannot_claim_optical_stuck_zero(self):
  r=self.replay();r.ingest(dict(kind='wheel',header_ns=1_000_001,message_id=199,speed=10.,variance=.01))
  for i in range(1,14):r.ingest(self.imu(i*10_000_000+1))
  e=self.opt(131_000_001,0.);r.ingest(e);self.assertEqual(e['last_update']['reason'],'LOW_SPEED_WHEEL')

class PlanarSpeedTests(unittest.TestCase):
 def test_complete_jacobian(self):
  from geometry_ieskf.planar_speed import OpticalPlanarSpeed
  from geometry_ieskf.state import inject
  from geometry_ieskf.common import exp
  T=np.eye(4);T[:3,:3]=exp(np.array([.03,.01,.05]));T[:3,3]=[2,.1,.4];o=OpticalPlanarSpeed([1,0,.2],T,(0,));x=State(np.zeros(3),np.array([15.,2.,.3]),exp(np.array([.03,-.02,.2])),np.zeros(3),np.zeros(3));w=np.array([.01,.02,.3]);h,H=o.model(x,w);D=np.zeros_like(H)
  for j in range(15):
   d=np.eye(15)[j]*1e-6;D[:,j]=(o.model(inject(x,d),w)[0]-o.model(inject(x,-d),w)[0])/2e-6
  np.testing.assert_allclose(H,D,atol=5e-9)
 def test_angular_measurement_invariance_and_nonzero_sideslip(self):
  from geometry_ieskf.planar_speed import OpticalPlanarSpeed
  o=OpticalPlanarSpeed([0,0,0],np.eye(4),(0,));x=State(np.zeros(3),np.array([10.,2.,0]),np.eye(3),np.zeros(3),np.zeros(3));P=np.eye(15)*.01;z=np.sqrt(104);a=o.correct(x,P,np.zeros(3),[z,0],np.eye(2)*.26);b=o.correct(x,P,np.zeros(3),[0,z],np.eye(2)*.26);np.testing.assert_allclose(a[0].v,b[0].v,atol=1e-14);np.testing.assert_allclose(a[1],b[1],atol=1e-14);self.assertAlmostEqual(a[0].v[1],2.)
