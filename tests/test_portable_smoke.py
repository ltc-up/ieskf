"""Small migration smoke checks; no historical experiment is rerun."""
import unittest
import numpy as np
from geometry_ieskf.state import State,inject,difference
from geometry_ieskf.inertial import InertialPredictor
from geometry_ieskf.geometry_factor import update
class PortableSmoke(unittest.TestCase):
 def test_static_predictor(self):
  x=State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3));y,P=InertialPredictor().predict(x,np.eye(15),np.zeros(3),np.array([0.,0.,9.80665]),.005)
  self.assertLess(np.linalg.norm(y.v),1e-12);self.assertGreater(np.linalg.eigvalsh(P).min(),0)
 def test_full_cross_state_factor(self):
  P=np.eye(15);P[0,3]=P[3,0]=.4;H=np.zeros((1,15));H[0,0]=1
  d,_,_=update(P,H,np.array([-1.]),np.array([1.]));self.assertAlmostEqual(d[0],.5);self.assertAlmostEqual(d[3],.2)
 def test_injection(self):
  x=State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3));d=np.arange(15)*.0001
  np.testing.assert_allclose(difference(inject(x,d),x),d,atol=1e-14)
if __name__=='__main__':unittest.main()
