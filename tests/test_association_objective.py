import unittest
import numpy as np
from geometry_ieskf.association_objective import CompactHuberMixture
from geometry_ieskf.valid_correspondence import NearestValidCorrespondence
from geometry_ieskf.geometry import GeometryFactor
from geometry_ieskf.state import State

class AssociationObjectiveTests(unittest.TestCase):
 def make(self,target=None,source=None,valid=None):
  target=np.array([[0,0,0],[.4,.2,.1],[-.2,.1,0.]]) if target is None else np.array(target,dtype=float)
  source=np.array([[.2,.1,.3],[.6,.2,.1]]) if source is None else np.array(source,dtype=float)
  q=NearestValidCorrespondence(target);q.cached[:]=True;q.valid[:]=True if valid is None else valid;q.normals[:]=[0.,0.,1.]
  x=State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3))
  A=np.eye(15);A[3,2]=.3;A[9,6]=.2;P=A@A.T
  return CompactHuberMixture(x,P,source,GeometryFactor([.05,-.02,.03]),q)
 def finite(self,o,z):
  g=o.gradient(z);fd=np.array([(o.cost(z+np.eye(15)[j]*1e-6)-o.cost(z-np.eye(15)[j]*1e-6))/2e-6 for j in range(15)])
  np.testing.assert_allclose(g,fd,atol=2e-7,rtol=1e-6)
 def test_complete_gradient_nonzero_chart_and_cross_prior(self):
  o=self.make();self.finite(o,np.linspace(-.03,.04,15))
 def test_unsupported_source_constant_and_prior(self):
  o=self.make([[5,5,5]],[[0,0,0]]);z=np.zeros(15);z[4]=.1
  self.finite(o,z);self.assertEqual(o.evaluate(z)[2]['candidate_pairs'],0)
 def test_radius_value_and_first_derivative_continuity(self):
  o=self.make([[0,0,0]],[[1.05,-.02,.03]])
  a=[]
  for dx in [-1e-4,-1e-6,0,1e-6,1e-4]:
   z=np.zeros(15);z[0]=dx;a.append((o.cost(z),o.gradient(z)[0]))
  self.assertLess(abs(a[1][0]-a[3][0]),1e-8);self.assertLess(abs(a[1][1]-a[3][1]),1e-4)
 def test_plane_order_tie_has_no_switch(self):
  o=self.make([[-.2,0,0],[.2,0,0]],[[.05,-.02,.25]])
  self.finite(o,np.zeros(15));z=np.zeros(15);z[0]=1e-7
  self.assertLess(np.linalg.norm(o.gradient(z)-o.gradient(-z)),1e-3)
 def test_invalid_plane_contributes_identically_zero(self):
  a=self.make([[0,0,0],[.2,0,0]],valid=[True,False]);b=self.make([[0,0,0]])
  np.testing.assert_allclose(a.gradient(np.zeros(15)),b.gradient(np.zeros(15)));self.assertAlmostEqual(a.cost(np.zeros(15)),b.cost(np.zeros(15)))
 def test_duplicate_map_is_not_claimed_density_invariant(self):
  a=self.make([[0,0,0]]);b=self.make([[0,0,0],[0,0,0]])
  self.assertGreater(abs(a.cost(np.zeros(15))-b.cost(np.zeros(15))),1e-6)
 def test_repeatability_and_permutation(self):
  a=self.make();b=self.make(a.query.target[::-1],a.source[::-1]);z=np.zeros(15)
  self.assertAlmostEqual(a.cost(z),b.cost(z));np.testing.assert_allclose(a.gradient(z),b.gradient(z),atol=1e-12)
 def test_mixture_bounds_nonnegative_and_capped(self):
  o=self.make();z=np.zeros(15);loss=o.evaluate(z)[3]['loss_per_source'];R=o.c['correspondence_distance_m'];h=o.c['huber_delta_m'];cap=h*(R-.5*h)
  self.assertTrue(np.all(loss>=0));self.assertTrue(np.all(loss<=cap+1e-12))
 def test_huber_kink_gradient_continuous(self):
  o=self.make([[0,0,0]],[[.05,-.02,.33]]);a=np.zeros(15);a[2]=-1e-8;b=-a
  self.assertLess(np.linalg.norm(o.gradient(a)-o.gradient(b)),1e-4)
if __name__=='__main__':unittest.main()
