import unittest
import numpy as np
from local_odometry import *

class Tests(unittest.TestCase):
 def compare_support(self,p,q,res=1.5):
  self.assertEqual(support(q,occupied(p,res),res),packed_support(q,packed_occupancy(p,res),res))
 def test_random_duplicates_negative_and_empty(self):
  rng=np.random.default_rng(2)
  for dtype in [np.float32,np.float64]:
   p=rng.uniform(-20,20,(2000,3)).astype(dtype);q=np.vstack([p[:400],p[:400],rng.uniform(-20,20,(500,3))]).astype(dtype)
   self.compare_support(p,q);self.compare_support(np.empty((0,3)),q);self.compare_support(p,np.empty((0,3)))
 def test_boundary_overflow_and_mixed_union(self):
  n=1<<20;p=np.array([[-n,0,0],[n-1,0,0],[n,0,0],[-n-1,0,0]],dtype=float)
  self.compare_support(p,p,1.)
  items=[packed_occupancy(p[:2],1.),packed_occupancy(p[2:],1.)]
  self.assertEqual(packed_support(p,packed_union(items),1.),1.)
 def test_union_has_distinct_occupancy_source_denominator_has_duplicates(self):
  a=np.array([[0,0,0],[0,0,0]]);b=np.array([[0,0,0],[2,0,0]])
  keys=packed_union([packed_occupancy(a,1.),packed_occupancy(a,1.)])
  self.assertEqual(packed_support(b,keys,1.),.5)
  self.assertEqual(packed_support(a,keys,1.),1.)
 def test_full_policy_history_and_pose_exact(self):
  rng=np.random.default_rng(5)
  for mode in ['B-K','B-K-MR']:
   a=LocalOdometry(Config(mode=mode,fast_overlap=False,max_frames=3,insert_overlap=.7,remove_overlap=.01))
   b=LocalOdometry(Config(mode=mode,fast_overlap=True,max_frames=3,insert_overlap=.7,remove_overlap=.01))
   for i in range(1,61):
    # Repeated points exercise support ratios, absent clusters exercise eviction.
    center=(i//8)*4
    p=np.vstack([rng.uniform(-2,2,(100,3))+[center,0,2],np.repeat([[center,0,2]],50,axis=0)])
    if i%17==0:p[0,0]=(1<<21)*1.5 # explicit fallback occupancy
    s=Scan.create(str(i),i*50_000_000,i*50_000_000+1,p,p[:30],'synthetic')
    U=np.eye(4);U[0,3]=i*.2
    ra=a.step(s,U,lambda q:{'O_T_base':q['initial_guess'].tolist()},lambda q,r:i%7!=0)
    rb=b.step(s,U,lambda q:{'O_T_base':q['initial_guess'].tolist()},lambda q,r:i%7!=0)
    self.assertEqual([x.scan.frame_id for x in a.history],[x.scan.frame_id for x in b.history])
    np.testing.assert_array_equal(a.O,b.O)
    self.assertEqual(ra.get('target_identity'),rb.get('target_identity'))
 def test_exact_insert_threshold_tie(self):
  # Exactly .7 must insert because only >.7 skips insertion.
  p=np.array([[i*3,0,0] for i in range(10)],float)
  q=np.vstack([p[:7],[[100,0,0],[103,0,0],[106,0,0]]])
  self.assertEqual(packed_support(q,packed_occupancy(p,1.5),1.5),.7)

if __name__=='__main__':unittest.main()
