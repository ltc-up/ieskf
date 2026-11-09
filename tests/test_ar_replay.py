import unittest
import numpy as np
from geometry_ieskf.state import State
from ndt_global.pose_delay import PoseDelayReplay
from ndt_global.ar_replay import AuditedPoseReplay,observation_health
class ARTests(unittest.TestCase):
 def test_health_age_not_a_new_measurement(self):
  self.assertEqual(observation_health(600,None,0,500,2000),'COASTING')
  self.assertEqual(observation_health(2100,0,0,500,2000),'LOST')
  self.assertEqual(observation_health(2101,2100,0,500,2000),'TRACKING')
 def test_time_reversal(self):
  with self.assertRaisesRegex(ValueError,'TIME_REVERSAL'):observation_health(1,2,0)
 def test_wheel_instrumentation_preserves_full_update(self):
  x=State(np.zeros(3),np.array([10.,.2,0]),np.eye(3),np.zeros(3),np.zeros(3),0);A=np.eye(15);A[2,3]=.5;P=A@A.T;imu=dict(header_ns=0,omega=np.zeros(3),force=np.array([0.,0.,9.80665]));args=(x,P,imu,np.array([1.,.1,0]))
  a=PoseDelayReplay(*args);b=AuditedPoseReplay(*args);e=dict(kind='wheel',header_ns=10_000_000,message_id=1,speed=10.1,variance=.01);f=dict(e);a.ingest(e);b.ingest(f)
  np.testing.assert_array_equal(a.current()[0].p,b.current()[0].p);np.testing.assert_array_equal(a.current()[1],b.current()[1]);u=f['last_update'];self.assertAlmostEqual(u['delta'][2],u['Kz']*u['innovation'])
