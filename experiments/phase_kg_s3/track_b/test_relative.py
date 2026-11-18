import unittest
import numpy as np
from evaluate_relative import relative_components

class Tests(unittest.TestCase):
 def test_constant_gauge_cancels(self):
  a=np.eye(4);b=np.eye(4);b[0,3]=1
  H=np.eye(4);H[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];H[:3,3]=[5,6,7]
  r=relative_components(H@a,H@b,a,b,.05)
  self.assertEqual(r['xy_m'],0)
 def test_actual_dt_scales_rate(self):
  a=np.eye(4);b=np.eye(4);b[0,3]=.1
  self.assertAlmostEqual(relative_components(a,b,a,a,.1)['translation_rate_error_mps'],1.)
  self.assertAlmostEqual(relative_components(a,b,a,a,.05)['translation_rate_error_mps'],2.)

if __name__=='__main__':unittest.main()
