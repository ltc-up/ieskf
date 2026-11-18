import hashlib
import unittest
import numpy as np
from local_odometry import voxel_centroids, voxel_centroids_reference

class Tests(unittest.TestCase):
 def compare(self,p,leaf=.8):
  p=np.asarray(p)
  a,b=voxel_centroids_reference(p,leaf),voxel_centroids(p,leaf)
  np.testing.assert_array_equal(a,b)
  self.assertEqual(hashlib.sha256(a.tobytes()).hexdigest(),hashlib.sha256(b.tobytes()).hexdigest())
 def test_empty(self):self.compare(np.empty((0,3)))
 def test_duplicate_negative(self):self.compare(np.array([[-1.,-.1,-4],[-1.,-.1,-4],[0,0,0],[.4,.5,.6],[-2,-5,-1]]))
 def test_signed_pack_boundaries(self):
  v=(1<<20)
  self.compare(np.array([[-v,-v,-v],[v-1,v-1,v-1],[-v,v-1,-v],[v-1,-v,v-1]],dtype=float),1.)
 def test_overflow_fallback(self):
  self.compare(np.array([[-(1<<20)-1,3,2],[(1<<20),-2,1],[2,3,4]],dtype=float),1.)
 def test_random_and_input_permutations(self):
  rng=np.random.default_rng(91)
  for dtype in [np.float32,np.float64]:
   p=rng.uniform(-120,120,(10000,3)).astype(dtype)
   self.compare(p);self.compare(p[rng.permutation(len(p))])
 def test_dense_coincident_voxel_sums(self):
  rng=np.random.default_rng(92)
  for dtype in [np.float32,np.float64]:self.compare(rng.uniform(-.1,.1,(10000,3)).astype(dtype))
 def test_invalid_leaf(self):
  for leaf in [0,-1,float('nan')]:
   with self.assertRaises(ValueError):voxel_centroids(np.zeros((1,3)),leaf)

if __name__=='__main__':unittest.main()
