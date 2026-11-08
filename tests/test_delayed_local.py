import unittest
import numpy as np
from geometry_ieskf.state import State,difference
from ndt_global.delayed_local import DelayedLocalReplay

class DelayedTests(unittest.TestCase):
 def make(self):
  x=State(np.zeros(3),np.array([5.,0.,0.]),np.eye(3),np.zeros(3),np.zeros(3),1_000_000_000)
  P=np.eye(15);P[0,3]=P[3,0]=.2
  return DelayedLocalReplay(x,P,dict(header_ns=x.t,omega=np.zeros(3),force=np.array([0,0,9.80665])),[0,0,0])
 def events(self):
  return [dict(kind='imu',header_ns=1_005_000_000,message_id=1,omega=[0,0,.1],force=[.2,0,9.80665]),dict(kind='wheel',header_ns=1_006_000_000,message_id=2,speed=5.1,variance=.1),dict(kind='imu',header_ns=1_010_000_000,message_id=3,omega=[0,0,.2],force=[.3,0,9.80665])]
 def test_replay_equals_header_state_and_full_P(self):
  a,b=self.make(),self.make();e=self.events()
  for v in e:a.ingest(v)
  for v in [e[1],e[2],e[0]]:b.ingest(v)
  x,P,_=a.current();y,Q,_=b.current();np.testing.assert_allclose(difference(x,y),0,atol=1e-14);np.testing.assert_array_equal(P,Q);self.assertGreater(np.linalg.eigvalsh(Q).min(),0);self.assertEqual(Q[0,3],Q[3,0])
 def test_duplicate_is_not_refused(self):
  a=self.make();v=self.events()[0];a.ingest(v);x,P,_=a.current();self.assertEqual(a.ingest(v),'DUPLICATE');np.testing.assert_array_equal(P,a.current()[1])
 def test_failed_future_gap_is_atomic(self):
  a=self.make();v=self.events()[0];a.ingest(v);x,P,_=a.current();bad=dict(v,header_ns=2_000_000_000,message_id=5)
  with self.assertRaisesRegex(ValueError,'OUTSIDE_FORWARD_HORIZON'):a.ingest(bad)
  np.testing.assert_array_equal(P,a.current()[1]);self.assertEqual(a.current()[0].t,x.t)
 def test_pending_wheel_unblocked_by_late_imu(self):
  a=self.make();v=dict(self.events()[1],header_ns=1_030_000_000);self.assertEqual(a.ingest(v),'DEFERRED_IMU_GAP');self.assertEqual(a.current()[0].t,1_000_000_000)
  a.ingest(dict(self.events()[0],header_ns=1_015_000_000));self.assertEqual(a.current()[0].t,1_030_000_000)
 def test_before_checkpoint_rejected(self):
  a=self.make();v=dict(self.events()[0],header_ns=999_999_999);self.assertEqual(a.ingest(v),'OUTSIDE_CHECKPOINT_HISTORY')
if __name__=='__main__':unittest.main()
