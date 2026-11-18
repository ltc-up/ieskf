import sys
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src'))
from geometry_ieskf.state import State
from ndt_global.managed_replay import ManagedReplay
from score_map import FrozenMap, Contract, ScoreMapObserver, DirectionalObserver, POSE, prior_identity, sandwich_information, certify_snap_path, score_domain

class MathTest(unittest.TestCase):
 def setUp(self):
  self.x=State(np.zeros(3),np.array([2.,0,0]),Rotation.from_euler('XYZ',[.12,.08,-.2]).as_matrix(),np.zeros(3),np.zeros(3),0)
  self.P=np.eye(15)*.04;self.P[0,3]=self.P[3,0]=.015
  self.P[0,9]=self.P[9,0]=.00001;self.lever=np.array([.1,-.05,.08])
  self.center=np.r_[.1,.03,-.02,.12,.08,-.2]
 def evaluate(self,q):
  A=np.diag([500,100,50,10,20,70]);d=q-self.center
  return dict(score=1000-.5*d@A@d,gradient=-A@d,hessian=-A,valid_pairs=20)
 def test_snap_points_reject_before_geometry_and_keep_prediction(self):
  x=self.x.copy();x.R=np.eye(3);calls=[]
  def forbidden(q):calls.append(q);raise AssertionError('snap geometry must not evaluate')
  obs=ScoreMapObserver(self.lever,forbidden,'s','t')
  T=np.eye(4);T[:3,3]=x.p-self.lever
  y,Q,info=obs.correct(x,self.P,T,None)
  self.assertFalse(info['accepted']);self.assertEqual(info['reason'],'NUMERICAL_DOMAIN_INVALID')
  np.testing.assert_array_equal(y.p,x.p);np.testing.assert_array_equal(Q,self.P)
  self.assertEqual(calls,[]);self.assertEqual(info['covariance_updates'],0)
 def test_snap_cross_sign_and_threshold_boundary(self):
  x=self.x.copy();x.R=Rotation.from_euler('XYZ',[.001,.3,.4]).as_matrix()
  end=Rotation.from_euler('XYZ',[-.001,.3,.4]).as_matrix();z0=np.zeros(15);z1=np.zeros(15)
  z1[6:9]=Rotation.from_matrix(x.R.T@end).as_rotvec()
  path=certify_snap_path(x,z0,z1,self.lever)
  self.assertFalse(path['valid']);self.assertIn('roll',path['axes'])
  self.assertIn('roll',score_domain(np.r_[0,0,0,1e-4,.3,.4]))
  self.assertNotIn('roll',score_domain(np.r_[0,0,0,np.nextafter(1e-4,1.),.3,.4]))
 def test_snap_same_sign_endpoints_curve_into_invalid_region(self):
  x=self.x.copy();x.R=Rotation.from_euler('XYZ',[.0003,.3,.4]).as_matrix()
  end=Rotation.from_euler('XYZ',[.0003,.1,.6]).as_matrix();a=np.zeros(15);b=np.zeros(15)
  b[6:9]=Rotation.from_matrix(x.R.T@end).as_rotvec()
  path=certify_snap_path(x,a,b,self.lever)
  self.assertFalse(path['valid']);self.assertIn('roll',path['axes'])
  self.assertEqual(path['reason'],'SNAP_ENDPOINT_OR_INTERIOR')
 def test_snap_uncertified_budget_rejects_conservatively(self):
  x=self.x.copy();x.R=Rotation.from_euler('XYZ',[.0003,.3,.4]).as_matrix()
  end=Rotation.from_euler('XYZ',[.0003,.1,.2]).as_matrix();a=np.zeros(15);b=np.zeros(15)
  b[6:9]=Rotation.from_matrix(x.R.T@end).as_rotvec()
  path=certify_snap_path(x,a,b,self.lever,max_segments=16)
  self.assertFalse(path['valid']);self.assertEqual(path['reason'],'SNAP_PATH_NOT_CERTIFIED_WITHIN_BUDGET')
 def test_snap_legal_translation_and_rotation_are_certified(self):
  a=np.zeros(15);b=np.zeros(15);b[:3]=[.1,.2,0]
  self.assertTrue(certify_snap_path(self.x,a,b,self.lever)['valid'])
  b[6:9]=[.001,.002,-.001]
  self.assertTrue(certify_snap_path(self.x,a,b,self.lever)['valid'])
 def test_prior_identity_preserves_nanoseconds(self):
  x=self.x.copy();x.t=1_800_000_000_000_000_000
  a=prior_identity(x,self.P);x.t+=1
  self.assertNotEqual(a,prior_identity(x,self.P))
 def test_chain_full15_gradient_hessian(self):
  f=FrozenMap(self.x,self.P,self.lever,self.evaluate);z=np.linspace(-.01,.01,15);v=f.value(z);eps=2e-4
  g=[];H=[]
  for i in range(15):
   d=np.eye(15)[i]*eps;p=f.value(z+d);m=f.value(z-d)
   g.append((p['F']-m['F'])/(2*eps));H.append((p['g']-m['g'])/(2*eps))
  self.assertLess(abs(v['g']-g).max(),1e-6)
  self.assertLess(abs(v['H']-np.array(H).T).max(),1e-5)
 def test_frozen_beta_one_preserves_large_align_correction(self):
  x=self.x.copy();x.p=np.zeros(3);P=np.eye(15)*.04
  center=np.r_[2.,0,0,Rotation.from_matrix(x.R).as_euler('XYZ')]
  A=np.diag([4000.,4000,4000,100,100,100])
  def geometry(q):
   d=q-center
   return dict(score=10000-.5*d@A@d,gradient=-A@d,hessian=-A,valid_pairs=100)
  T=np.eye(4);T[:3,:3]=x.R;T[:3,3]=[2,0,0]
  y,Q,info=FrozenMap(x,P,np.zeros(3),geometry).solve(T)
  self.assertEqual(info['beta'],1.0);self.assertAlmostEqual(y.p[0],2*160/161,places=6)
  self.assertEqual(info['selected'],'align_refined')
  self.assertEqual(info['selected_metrics']['valid_pairs'],100)
  self.assertEqual(len(info['selected_metrics']['prior_gradient']),15)
 def test_frozen_prior_cross_terms_once(self):
  f=FrozenMap(self.x,self.P,self.lever,self.evaluate);T=np.eye(4);T[:3,:3]=Rotation.from_euler('XYZ',self.center[3:]).as_matrix();T[:3,3]=self.center[:3]
  y,Q,info=f.solve(T)
  self.assertEqual(len(info['candidates']),4);self.assertEqual(info['covariance_updates'],1)
  self.assertNotEqual(y.v[0],self.x.v[0]);self.assertNotEqual(y.bg[0],0)
  self.assertEqual(info['prior_hash'],prior_identity(self.x,self.P));self.assertFalse(info['covariance_calibrated'])
  self.assertTrue(np.linalg.eigvalsh(Q).min()>0)
 def test_failed_align_branch_keeps_prior_branch_and_logs_failure(self):
  def limited(q):
   value=self.evaluate(q)
   if q[0]>.3:value['valid_pairs']=0
   return value
  f=FrozenMap(self.x,self.P,self.lever,limited);T=np.eye(4);T[:3,:3]=self.x.R;T[:3,3]=[.6,0,0]
  y,Q,info=f.solve(T)
  self.assertTrue(info['accepted']);self.assertIn(info['selected'],['prior','prior_refined'])
  failures=[v for v in info['candidates'] if v.get('failure')]
  self.assertEqual(len(failures),2);self.assertEqual(failures[0]['failure'],'INVALID_SCORE')
 def test_total_prior_rejects_score_only_trial(self):
  f=FrozenMap(self.x,self.P,self.lever,self.evaluate)
  a=f.value(np.zeros(15));z=np.zeros(15);z[0]=5;b=f.value(z)
  self.assertGreater(b['F'],a['F'])
 def test_replay_duplicate_information_and_historical_prior(self):
  imu=dict(header_ns=0,omega=np.zeros(3),force=np.array([0,0,9.81]))
  r=ManagedReplay(self.x,self.P,imu,self.lever)
  r.ingest(dict(kind='imu',header_ns=10_000_000,message_id=1,omega=np.zeros(3),force=np.array([0,0,9.81])))
  obs=ScoreMapObserver(self.lever,self.evaluate,'source-1','target-1','wrong-worker-hash')
  T=np.eye(4);T[:3,:3]=self.x.R;T[:3,3]=self.center[:3]
  e=dict(kind='ndt',header_ns=10_000_000,message_id=3,observation_id='scan1',correlation_group='scan1',observer=obs,pose=T,R=None)
  r.ingest(e);self.assertTrue(e['last_update']['worker_prior_changed']);state=r.current()[0].p
  self.assertEqual(r.ingest(e),'DUPLICATE_TERMINAL');np.testing.assert_array_equal(state,r.current()[0].p)
  # A new ID with the same information group also cannot fuse this scan.
  e2=e|dict(message_id=4,observation_id='scan1alias')
  self.assertEqual(r.ingest(e2),'DUPLICATE_INFORMATION_GROUP')
 def test_a1_full_chart_and_score_scale_cancellation(self):
  G=np.random.default_rng(7).normal(size=(15,6))*10;H=-np.diag([100,80,60,20,15,30])
  I,_=sandwich_information(H,G);I2,_=sandwich_information(7*H,7*G)
  np.testing.assert_allclose(I,I2,rtol=1e-12,atol=1e-12)
  def geom(q):return dict(hessian=H,block_gradients=G)
  obs=DirectionalObserver(self.lever,geom,'source','target')
  T=np.eye(4);T[:3,:3]=self.x.R;T[:3,3]=[.15,-.03,.04]
  y,Q,info=obs.correct(self.x,self.P,T,None)
  self.assertTrue(info['accepted']);self.assertFalse(info['covariance_calibrated'])
  self.assertNotEqual(y.v[0],self.x.v[0]);self.assertEqual(info['covariance_updates'],1)
 def test_sandwich_zero_direction_receives_no_information(self):
  H=-np.diag([100,80,0,20,10,0]);G=np.random.default_rng(4).normal(size=(12,6));I,info=sandwich_information(H,G)
  self.assertAlmostEqual(I[2,2],0);self.assertAlmostEqual(I[5,5],0)
  self.assertFalse(info['covariance_calibrated'])

if __name__=='__main__':unittest.main()
