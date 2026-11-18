import unittest
import numpy as np
from kinematic_predictor import *


def sample(kind,h,a=None,ident=None,value=None):
 r=dict(kind=kind,header_ns=int(h),available_ns=int(h if a is None else a),message_id=int(h if ident is None else ident))
 if kind=='imu':r['omega_base']=[0.,0.,0.] if value is None else value
 else:r.update(speed_base=1. if value is None else value,variance=.1)
 return r


class Tests(unittest.TestCase):
 def test_straight_reverse_and_zero(self):
  for speed in [0.,2.,-2.]:
   T=twist_step(np.eye(4),[0,0,0],speed,.1);np.testing.assert_allclose(T[:3,3],[speed*.1,0,0]);np.testing.assert_allclose(T[:3,:3],np.eye(3))
 def test_exact_constant_turn(self):
  w,v,t=.7,12.,.13;T=twist_step(np.eye(4),[0,0,w],v,t)
  np.testing.assert_allclose(T[:3,3],[v/w*np.sin(w*t),v/w*(1-np.cos(w*t)),0],atol=1e-13)
  np.testing.assert_allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-14)
 def test_full3d_twist_matches_fine_midpoint(self):
  w=np.array([.4,-.7,.2]);v=8.;dt=.1;T=twist_step(np.eye(4),w,v,dt);fine=np.eye(4)
  for _ in range(1000):fine=twist_step(fine,w,v,dt/1000)
  np.testing.assert_allclose(T,fine,atol=2e-13)
  self.assertNotEqual(T[2,3],0)
 def test_tilt_does_not_force_world_z_constant(self):
  H=twist_step(np.eye(4),[0,1,0],0,.4);T=H@twist_step(np.eye(4),[0,0,0],5,.1)
  self.assertLess(T[2,3],-.1)
 def test_gyro_25ms_wheel_100ms_boundaries(self):
  p=IntervalKinematicPredictor(0);p.ingest(sample('imu',0));p.ingest(sample('wheel',0))
  self.assertEqual(p.query(0,25_000_000,25_000_000,25_000_000)['status'],'VALID')
  self.assertEqual(p.query(0,25_000_001,25_000_001,25_000_001)['status'],'DELAY')
  p=IntervalKinematicPredictor(0)
  for h in range(0,100_000_001,20_000_000):p.ingest(sample('imu',h))
  p.ingest(sample('wheel',0,a=100_000_000));self.assertEqual(p.query(0,100_000_000,100_000_000,100_000_000)['status'],'VALID')
  p.ingest(sample('imu',100_000_001));self.assertEqual(p.query(0,100_000_001,100_000_001,100_000_001)['status'],'DELAY')
 def test_future_value_excluded_and_earliest_actual_retry(self):
  audit=HeaderCoverageAudit([0,20_000_000,40_000_000],[0,100_000_000],0,100_000_000,'synthetic')
  p=IntervalKinematicPredictor(0,audit);p.ingest(sample('imu',0));p.ingest(sample('wheel',0))
  before=p.query(0,40_000_000,50_000_000,50_000_000);self.assertEqual(before['status'],'DELAY');self.assertIsNone(before['delta_T'])
  result,attempts=earliest_retry(p,0,40_000_000,50_000_000,[sample('imu',20_000_000,a=60_000_000,value=[0,0,1])])
  self.assertEqual(result['status'],'VALID');self.assertEqual(result['query_delay_ns'],10_000_000)
  self.assertEqual(result['original_cloud_available_ns'],50_000_000)
  for s in result['segments']:self.assertLessEqual(max(s['imu_available_ns'],s['wheel_available_ns']),result['query_available_ns'])
 def test_prefetched_future_input_still_not_used(self):
  p=IntervalKinematicPredictor(0);p.ingest(sample('imu',0));p.ingest(sample('wheel',0));p.ingest(sample('imu',20_000_000,a=60_000_000))
  self.assertEqual(p.query(0,40_000_000,50_000_000,50_000_000)['status'],'DELAY')
 def test_audit_gap_distinct_from_delay(self):
  audit=HeaderCoverageAudit([0,40_000_000],[0,100_000_000],0,100_000_000,'synthetic')
  p=IntervalKinematicPredictor(0,audit);p.ingest(sample('imu',0));p.ingest(sample('wheel',0))
  r=p.query(0,40_000_000,50_000_000,50_000_000);self.assertEqual(r['status'],'IRRECOVERABLE_INPUT_GAP')
  self.assertEqual(r['audited_gaps'][0]['kind'],'imu')
 def test_no_unaudited_gap_claim_at_end_of_available_stream(self):
  p=IntervalKinematicPredictor(0);p.ingest(sample('imu',0));p.ingest(sample('wheel',0))
  r,attempts=earliest_retry(p,0,40_000_000,50_000_000,[]);self.assertEqual(r['status'],'DELAY');self.assertIn('NOT_PROVEN_GAP',r['reason'])
 def test_bias_fixed_zero_and_duplicates(self):
  p=IntervalKinematicPredictor(0);e=sample('imu',0);p.ingest(e);self.assertEqual(p.ingest(e),'DUPLICATE')
  with self.assertRaises(ValueError):p.bias[0]=1
  e=dict(e,omega_base=[1,0,0])
  with self.assertRaises(ValueError):p.ingest(e)
 def test_invalid_input_and_resource_failure(self):
  p=IntervalKinematicPredictor(0,maximum_samples=1)
  with self.assertRaises(ValueError):p.ingest(sample('wheel',0,value=float('nan')))
  p.ingest(sample('imu',0))
  with self.assertRaises(MemoryError):p.ingest(sample('wheel',0))
 def test_exact_header_changes_zoh_not_interpolation(self):
  p=IntervalKinematicPredictor(0)
  for e in [sample('imu',0),sample('wheel',0,value=1),sample('wheel',10_000_000,value=3),sample('imu',20_000_000)]:p.ingest(e)
  r=p.query(0,20_000_000,20_000_000,20_000_000)
  self.assertAlmostEqual(r['delta_T'][0][3],.04)
 def test_prune_keeps_held_prefix(self):
  p=IntervalKinematicPredictor(0)
  for h in range(0,61_000_000,20_000_000):
   p.ingest(sample('imu',h));p.ingest(sample('wheel',h))
  p.prune_before(40_000_000)
  self.assertTrue(all(s.header_ns>=40_000_000 for rows in p.samples.values() for s in rows))
  self.assertEqual(p.query(40_000_000,60_000_000,60_000_000,60_000_000)['status'],'VALID')


if __name__=='__main__':unittest.main()
