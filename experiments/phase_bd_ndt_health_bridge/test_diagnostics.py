import unittest
import numpy as np
from diagnostics import period_two,scaled_xy_yaw
class DiagnosticsTest(unittest.TestCase):
 def test_many_monotone_iterations_not_oscillation(self):self.assertFalse(period_two(np.linspace(0,10,100)))
 def test_alternation_requires_recurrence(self):
  self.assertTrue(period_two([6107,6142]*6))
  self.assertFalse(period_two([0,10,2,20,5,40,9,80]))
 def test_short_trace_and_nonfinite_not_proof(self):
  self.assertFalse(period_two([1,2,1,2]));self.assertFalse(period_two([1,2,np.nan,2,1,2]))
 def test_coupling_retained(self):
  H=-np.eye(6);H[0,5]=H[5,0]=-.4
  info,e,u=scaled_xy_yaw(H,np.eye(3),10)
  self.assertAlmostEqual(info[0,2],.04);self.assertGreater(abs(u[0,0]),.001)
 def test_wrong_sign_is_not_repaired(self):
  _,e,_=scaled_xy_yaw(np.eye(6),np.eye(3),10);self.assertTrue(np.all(e<0))
 def test_invalid_scale(self):
  with self.assertRaises(ValueError):scaled_xy_yaw(-np.eye(6),np.eye(3),0)
if __name__=='__main__':unittest.main()
