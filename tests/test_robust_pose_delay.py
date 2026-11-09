import unittest
import numpy as np
from geometry_ieskf.state import State,difference
from geometry_ieskf.robust_packet_pose import ScaleMixturePose
from ndt_global.pose_delay import PoseDelayReplay
class RobustDelayTests(unittest.TestCase):
 def make(self,horizon):
  x=State(np.zeros(3),np.array([2.,0.,0.]),np.eye(3),np.zeros(3),np.zeros(3),1_000_000_000);imu=dict(header_ns=x.t,omega=np.zeros(3),force=np.array([0.,0.,9.80665]));return PoseDelayReplay(x,np.eye(15)*.01,imu,np.zeros(3),horizon_ns=horizon),imu
 def test_late_mixture_matches_in_order_and_deduplicates(self):
  a,imu=self.make(2_100_000_000);b,_=self.make(2_100_000_000);short,_=self.make(500_000_000);T=np.eye(4);T[:3,3]=[.03,.02,0.];nd=dict(kind='ndt',header_ns=1_010_000_000,message_id=1,correlation_group='single',observer=ScaleMixturePose(np.zeros(3),('x','y','z','rx','ry','rz')),pose=T,R=np.eye(6)*.02)
  a.ingest(dict(nd))
  for i in range(1,64):
   event=dict(kind='imu',header_ns=1_000_000_000+i*10_000_000,message_id=100+i,omega=imu['omega'],force=imu['force'])
   for replay in [a,b,short]:replay.ingest(dict(event))
  b.ingest(dict(nd));self.assertEqual(short.ingest(dict(nd)),'OUTSIDE_CHECKPOINT_HISTORY');np.testing.assert_allclose(difference(a.current()[0],b.current()[0]),0,atol=1e-13);np.testing.assert_allclose(a.current()[1],b.current()[1],atol=1e-13);self.assertEqual(b.ingest(dict(nd)),'DUPLICATE');self.assertEqual(b.ingest(dict(nd,message_id=999)),'DUPLICATE_INFORMATION_GROUP')
 def test_retention_alone_does_not_change_forward_equations(self):
  a,imu=self.make(500_000_000);b,_=self.make(2_100_000_000)
  for i in range(1,100):
   e=dict(kind='imu',header_ns=1_000_000_000+i*10_000_000,message_id=i,omega=imu['omega'],force=imu['force'])
   a.ingest(dict(e));b.ingest(dict(e))
  np.testing.assert_array_equal(a.current()[0].p,b.current()[0].p);np.testing.assert_array_equal(a.current()[1],b.current()[1]);self.assertLessEqual(len(b.events),10000)
