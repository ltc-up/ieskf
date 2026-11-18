import unittest
from pathlib import Path
import numpy as np
from causal_runner import at_reference, state_record
from geometry_ieskf.state import State
from ndt_global.managed_replay import LegacyComparisonReplay
from provenance import require_run_output
class TestHarness(unittest.TestCase):
 def test_history_lookup_never_uses_future_wheel(self):
  t=10**18;x=State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3),t)
  P=np.eye(15)*.01
  r=LegacyComparisonReplay(x,P,dict(header_ns=t,omega=np.zeros(3),force=np.array([0,0,9.80665])),np.zeros(3))
  r.ingest(dict(kind='imu',header_ns=t+10_000_000,message_id=1,omega=np.zeros(3),force=np.array([0,0,9.80665])))
  before=at_reference(r,t+5_000_000,1)[0].copy()
  r.ingest(dict(kind='wheel',header_ns=t+10_000_000,message_id=2,speed=5.,variance=.01))
  after=at_reference(r,t+5_000_000,2)[0]
  np.testing.assert_array_equal(before.v,after.v)
  self.assertGreater(r.current()[0].v[0],after.v[0])
 def test_full_P_and_base_reference_preserved(self):
  x=State(np.array([3.,1,2]),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3),10**18)
  P=np.eye(15);P[0,4]=P[4,0]=.2
  r=LegacyComparisonReplay(x,P,dict(header_ns=x.t,omega=np.zeros(3),force=np.array([0,0,9.8])),np.array([1.,0,0]))
  z=state_record(r,np.array([1.,0,0]));self.assertEqual(z['P'][0,4],.2);np.testing.assert_equal(z['T_map_base'][:3,3],[2,1,2])
 def test_output_path_never_falls_back_to_system_disk(self):
  with self.assertRaisesRegex(ValueError,'OUTPUT_OUTSIDE_APPROVED_VOLUME'):require_run_output('/tmp/not_an_experiment')
if __name__=='__main__':unittest.main()
