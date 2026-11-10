import unittest,numpy as np
from geometry_ieskf.velocity_frontend import PolynomialVelocity
class FrontendTests(unittest.TestCase):
 def test_Q_psd_and_exact_white_jerk_integral(self):
  for dt in [.001,.004,.01,.1]:
   F,Q=PolynomialVelocity.matrices(dt);self.assertGreater(np.linalg.eigvalsh(Q).min(),0);self.assertAlmostEqual(Q[0,2],dt**3/6)
   t=np.linspace(0,dt,10001);G=np.array([t*t/2,t,np.ones_like(t)]);reference=np.trapezoid(G[:,None,:]*G[None,:,:],t,axis=2) if hasattr(np,'trapezoid') else np.trapz(G[:,None,:]*G[None,:,:],t,axis=2);np.testing.assert_allclose(Q,reference,atol=1e-10)
 def test_legacy_actual_integer_division(self):
  _,Q=PolynomialVelocity.matrices(.1,legacy=True);np.testing.assert_array_equal(Q,np.diag([0,0,.1]))
 def test_ramp_and_query_timestamp(self):
  f=PolynomialVelocity()
  for i in range(1000):t=i*.004;f.push(t,20-2*t)
  x,P=f.query(t+.01);self.assertAlmostEqual(x[0],20-2*(t+.01),places=4);self.assertGreater(np.linalg.eigvalsh(P).min(),0)
 def test_time_and_gap(self):
  f=PolynomialVelocity();f.push(0,1)
  with self.assertRaises(ValueError):f.push(0,2)
  with self.assertRaises(ValueError):f.query(2)
  f.push(2,5);self.assertEqual(f.x[0],5.)

class SharedErrorTests(unittest.TestCase):
 def state(self):
  from geometry_ieskf.velocity_bias import lift
  from geometry_ieskf.state import State
  from geometry_ieskf.common import exp
  return lift(State(np.zeros(3),np.array([20.,1.,.2]),exp(np.array([.02,.03,.3])),np.zeros(3),np.zeros(3)))
 def test_full17_optical_jacobian(self):
  from geometry_ieskf.velocity_bias import BiasOptical,lift
  from geometry_ieskf.state import inject
  x=self.state();T=np.eye(4);T[:3,3]=[2.,.1,.3];o=BiasOptical([1,0,0],T,(0,1));w=np.array([0,0,.3]);h,H=o.model(x,w);D=np.zeros_like(H)
  for j in range(17):
   d=np.eye(17)[j]*1e-6;a=lift(inject(x,d[:15]));b=lift(inject(x,-d[:15]));a.bo=x.bo+d[15:];b.bo=x.bo-d[15:];D[:,j]=(o.model(a,w)[0]-o.model(b,w)[0])/2e-6
  np.testing.assert_allclose(H,D,atol=5e-9)
 def test_full17_joseph_and_reset(self):
  from geometry_ieskf.velocity_bias import update
  from geometry_ieskf.state import chart_map
  rng=np.random.default_rng(8);L=rng.normal(size=(17,17))*.01;P=L@L.T+np.eye(17)*.01;H=rng.normal(size=(2,17));R=np.array([[.2,.03],[.03,.3]]);r=np.array([.002,-.001]);y,Q,info=update(self.state(),P,H,r,R);K=P@H.T@np.linalg.inv(H@P@H.T+R);d=K@r;A=np.eye(17)-K@H;B=np.eye(17);B[:15,:15]=chart_map(d[:15]);ref=B@(A@P@A.T+K@R@K.T)@B.T;np.testing.assert_allclose(Q,ref,atol=2e-17);np.testing.assert_allclose(info['delta'],d,atol=1e-17)
 def test_bias_persistence_and_covariance_cross(self):
  from geometry_ieskf.velocity_bias import BiasPredictor
  x=self.state();x.bo=np.array([.2,-.1]);P=np.eye(17)*.01;P[0,15]=P[15,0]=.002;y,Q=BiasPredictor().predict(x,P,[0,0,0],[0,0,9.80665],.01);np.testing.assert_array_equal(y.bo,x.bo);self.assertEqual(Q[15,15],P[15,15]);self.assertNotEqual(Q[0,15],0)
 def test_two_source_covariance_upper_bound(self):
  # For any pair joint PSD errors, 2 diag(marginals)-Rjoint is PSD.
  rng=np.random.default_rng(23)
  for i in range(20):
   A=rng.normal(size=(3,3));V=A@A.T;U=np.zeros((3,3));U[0,0]=2*V[0,0];U[1:,1:]=2*V[1:,1:];self.assertGreater(np.linalg.eigvalsh(U-V).min(),-1e-12)
 def test_late_joint_source_equivalence(self):
  import copy
  from geometry_ieskf.velocity_bias import BiasOptical
  from ndt_global.joint_velocity_replay import JointVelocityReplay
  from geometry_ieskf.state import State
  def make():
   x=State(np.zeros(3),np.array([10.,0.,0.]),np.eye(3),np.zeros(3),np.zeros(3),1);r=JointVelocityReplay(x,np.eye(15)*.01,dict(header_ns=1,omega=np.zeros(3),force=np.array([0,0,9.80665])),np.zeros(3));r.optical=BiasOptical([0,0,0],np.eye(4),(0,1));return r
  a=make();b=make();late=dict(kind='optical',header_ns=5_000_001,message_id=999,velocity=[10.,.1],variance=[[.2,.01],[.01,.3]],status_good=True)
  for i in range(1,10):
   e=dict(kind='imu',header_ns=i*1_000_000+1,message_id=i,omega=np.zeros(3),force=np.array([0.,0.,9.80665]));a.ingest(copy.deepcopy(e));b.ingest(copy.deepcopy(e))
   if i==5:a.ingest(copy.deepcopy(late))
  b.ingest(late);np.testing.assert_allclose(a.current()[0].v,b.current()[0].v,atol=1e-14);np.testing.assert_allclose(a.current()[1],b.current()[1],atol=1e-14);self.assertEqual(b.ingest(late),'DUPLICATE')

class BlockTests(unittest.TestCase):
 def test_exact_quadratic_forecast_and_noise_floor(self):
  from geometry_ieskf.velocity_frontend import BlockVelocity
  ns=np.arange(0,48_000_000,4_000_000,dtype=np.int64);t=ns*1e-9;Y=np.c_[10-2*t+3*t*t,.5+4*t-t*t];z,R,info=BlockVelocity.estimate(ns,Y,50_000_000,np.array([[.2,.01],[.01,.3]]));np.testing.assert_allclose(z,[10-2*.05+3*.05**2,.5+4*.05-.05**2],atol=1e-13);self.assertGreater(np.linalg.eigvalsh(R).min(),.25)
 def test_future_or_duplicate_input_rejected(self):
  from geometry_ieskf.velocity_frontend import BlockVelocity
  with self.assertRaises(ValueError):BlockVelocity.estimate(np.arange(6),np.zeros((6,2)),4,np.eye(2))
  with self.assertRaises(ValueError):BlockVelocity.estimate(np.zeros(6),np.zeros((6,2)),10,np.eye(2))


class BrakeInterventionTests(unittest.TestCase):
 def test_unchanged_outside_deceleration_and_actual_omission(self):
  import copy
  from geometry_ieskf.state import State
  from ndt_global.ar_replay import AuditedPoseReplay
  from ndt_global.brake_counterfactual import BrakeWheelCounterfactual
  for ax in [0.,-3.]:
   x=State(np.zeros(3),np.array([10.,0.,0.]),np.eye(3),np.zeros(3),np.zeros(3),1);P=np.eye(15)*.01;imu=dict(header_ns=1,omega=np.zeros(3),force=np.array([ax,0.,9.80665]));a=AuditedPoseReplay(x,P,imu,np.zeros(3));b=BrakeWheelCounterfactual(x,P,imu,np.zeros(3));e=dict(kind='wheel',header_ns=1_000_001,message_id=3,speed=9.5,variance=.01);a.ingest(copy.deepcopy(e));b.ingest(e)
   if ax==0:np.testing.assert_array_equal(a.current()[0].v,b.current()[0].v);np.testing.assert_array_equal(a.current()[1],b.current()[1])
   else:self.assertEqual(e['last_update']['reason'],'BRAKE_WHEEL_COUNTERFACTUAL');self.assertAlmostEqual(b.current()[0].v[0],9.997);self.assertGreater(np.linalg.norm(b.current()[0].v-a.current()[0].v),0)
 def test_block_stale_samples_rejected(self):
  from geometry_ieskf.velocity_frontend import BlockVelocity
  with self.assertRaises(ValueError):BlockVelocity.estimate(np.arange(0,24_000_000,4_000_000),np.zeros((6,2)),100_000_000,np.eye(2))


class FrontendFaultTests(unittest.TestCase):
 def test_smoothed_mean_cannot_hide_raw_fault(self):
  from geometry_ieskf.state import State
  from geometry_ieskf.optical_velocity import OpticalVelocity
  from ndt_global.frontend_replay import FrontendReplay
  x=State(np.zeros(3),np.array([10.,0,0]),np.eye(3),np.zeros(3),np.zeros(3),1);r=FrontendReplay(x,np.eye(15)*.01,dict(header_ns=1,omega=np.zeros(3),force=np.array([0,0,9.80665])),np.zeros(3));r.policy='optical_xy';r.optical=OpticalVelocity([0,0,0],np.eye(4),(0,1));e=dict(kind='optical',header_ns=1_000_001,message_id=99,velocity=[10.,0],raw_velocity=[25.,0],variance=[[.26,0],[0,.26]],status_good=True);r.ingest(e);self.assertFalse(e['last_update']['accepted']);self.assertEqual(e['last_update']['reason'],'RAW_FRONTEND_DISAGREEMENT');self.assertEqual(r.current()[0].v[0],10.)
