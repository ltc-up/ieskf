import unittest
import numpy as np
from ndt_global.validation import static_transform,closed_traverse,timestamp_qualified
from ndt_global.shadow import Contract,PoseHistory,ShadowCorrector,Candidate

class ValidationTests(unittest.TestCase):
 def test_zero_header_is_not_recorder_time(self):
  self.assertFalse(timestamp_qualified([dict(header_ns=0,recorder_ns=123)]))
 def test_missing_transform_fails(self):
  with self.assertRaisesRegex(ValueError,'UNAVAILABLE'):static_transform([],'base','imu')
 def test_graph_uses_direction_and_composition(self):
  tf=[dict(parent='base',child='axle',p=[1,0,0],q=[0,0,0,1]),dict(parent='axle',child='imu',p=[0,2,0],q=[0,0,0,1])]
  np.testing.assert_allclose(static_transform(tf,'base','imu')@static_transform(tf,'imu','base'),np.eye(4))
  np.testing.assert_allclose(static_transform(tf,'base','imu')[:3,3],[1,2,0])
 def test_short_input_not_certified_lap(self):
  self.assertIsNone(closed_traverse([dict(header_ns=t,position=[0,0,0],quaternion_xyzw=[0,0,0,1]) for t in [0,100_000_000_000]]))
 def test_delay_matches_header_not_latest(self):
  c=Contract('fixture','fixture','fixture');h=PoseHistory(c);a=np.eye(4);b=np.eye(4);b[0,3]=10;h.add(0,a,c);h.add(100_000_000,b,c);s=ShadowCorrector(h);m=np.eye(4);m[0,3]=20;before=[p.copy() for p in h.poses]
  self.assertTrue(s.consume(Candidate(50_000_000,'a','a',m,c,True,0.,'OK'),100_000_000)['accepted']);self.assertAlmostEqual(s.correction[0,3],15.)
  for x,y in zip(before,h.poses):np.testing.assert_array_equal(x,y)
 def test_historical_gap_rejected(self):
  c=Contract('fixture','fixture','fixture');h=PoseHistory(c);h.add(0,np.eye(4),c);h.add(200_000_000,np.eye(4),c)
  with self.assertRaisesRegex(ValueError,'HISTORY_GAP'):h.at(100_000_000)
if __name__=='__main__':unittest.main()
