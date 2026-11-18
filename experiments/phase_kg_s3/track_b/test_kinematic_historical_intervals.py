import unittest
import numpy as np
from kinematic_historical_intervals import diagnose

class HistoryTests(unittest.TestCase):
 def test_lag2_composes_committed_intervals_and_keeps_availability(self):
  d=np.eye(4);d[0,3]=1
  rows=[dict(ref_ns=i*50_000_000,prev_ref_ns=(i-1)*50_000_000,delta_T=d.tolist(),legacy_delta_T=(d@d).tolist(),original_available_ns=i*50_000_000+10,query_available_ns=i*50_000_000+20) for i in (1,2)]
  r=diagnose(rows,[dict(case_id='historical',lag=2,requested_ref_ns=100_000_001)])[0]
  self.assertEqual(r['actual_dt_s'],.1);self.assertEqual(r['kinematic_delta_T'][0][3],2);self.assertFalse(r['B0_strict_previous_registration']);self.assertFalse(r['uses_future_interval_rewrite']);self.assertEqual(r['offset_from_requested_ns'],-1)
 def test_no_silent_lag_shortening(self):
  r=diagnose([dict(ref_ns=50)], [dict(lag=2,requested_ref_ns=50)])[0]
  self.assertEqual(r['status'],'INSUFFICIENT_COMMITTED_PREHISTORY')

if __name__=='__main__':unittest.main()
