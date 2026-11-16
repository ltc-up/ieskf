import unittest
import numpy as np
from sampling import space_order,random_order,quality
class SamplingTests(unittest.TestCase):
 def setUp(self):self.p=np.random.default_rng(42).normal(size=(25000,3))*[30,15,2]
 def test_exact_permutation(self):
  o=space_order(self.p,123);self.assertEqual(len(o),len(self.p));np.testing.assert_array_equal(np.sort(o),np.arange(len(o)))
 def test_reproducibility_and_nestedness(self):
  a=space_order(self.p,123);b=space_order(self.p,123);np.testing.assert_array_equal(a,b)
  for x,y in [(3000,5000),(5000,10000),(10000,20000)]:self.assertTrue(set(a[:x])<=set(b[:y]))
 def test_extrema_preserved(self):
  p=self.p[space_order(self.p,123)[:3000]];np.testing.assert_array_equal(p.min(0),self.p.min(0));np.testing.assert_array_equal(p.max(0),self.p.max(0))
 def test_different_seed_and_random(self):
  a=space_order(self.p,123);self.assertFalse(np.array_equal(a,space_order(self.p,456)));r=random_order(self.p,789);np.testing.assert_array_equal(r,random_order(self.p,789));self.assertEqual(len(np.unique(r[:10000])),10000)
 def test_input_unmodified(self):
  p=self.p.copy();space_order(p,123);quality(p[:3000]);np.testing.assert_array_equal(p,self.p)
 def test_finite_contract(self):
  self.p[0,0]=np.nan
  with self.assertRaises(AssertionError):space_order(self.p,123)
if __name__=='__main__':unittest.main()
