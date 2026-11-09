import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from ndt_global.footprint_audit import polygon_audit,antenna_to_base,segments_intersect
class FootprintAuditTests(unittest.TestCase):
 def setUp(self):
  self.o=np.array([[-10,-10],[10,-10],[10,10],[-10,10.]])
  self.i=np.array([[-1,-1],[1,-1],[1,1],[-1,1.]])
 def test_contained(self):self.assertTrue(polygon_audit(np.array([[3,3],[4,3],[4,4],[3,4]]),self.o,self.i)['contained'])
 def test_sparse_edge_crossing(self):self.assertTrue(polygon_audit(np.array([[9,0],[11,0],[11,2],[9,2]]),self.o,self.i)['boundary_intersection'])
 def test_hole_enclosed(self):self.assertTrue(polygon_audit(np.array([[-2,-2],[2,-2],[2,2],[-2,2]]),self.o,self.i)['encloses_hole'])
 def test_touch(self):self.assertFalse(polygon_audit(np.array([[9,0],[10,0],[10,2],[9,2]]),self.o,self.i)['contained'])
 def test_outside(self):self.assertFalse(polygon_audit(np.array([[11,0],[12,0],[12,2],[11,2]]),self.o,self.i)['contained'])
 def test_lever_full_rotation(self):
  R=Rotation.from_euler('xyz',[.2,.3,.8]).as_matrix();p=np.array([4,5,6]);l=np.array([2,1,3]);np.testing.assert_allclose(antenna_to_base(p+R@l,R,l),p)
 def test_parallel(self):self.assertFalse(segments_intersect(*map(np.array,[[0,0],[1,0],[0,1],[1,1]])))
 def test_overlap(self):self.assertTrue(segments_intersect(*map(np.array,[[0,0],[2,0],[1,0],[3,0]])))
 def test_height_changes_pitched_silhouette(self):
  from ndt_global.footprint_audit import project_footprint
  p=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]])
  w=project_footprint(p,Rotation.from_euler('y',np.pi/4).as_matrix(),[0,0,0]);self.assertAlmostEqual(w[:,0].max(),np.sqrt(2))
 def test_full_chain_equals_direct_lever(self):
  R=Rotation.from_euler('xyz',[.4,-.3,.9]).as_matrix();a=np.array([1.,2.,3.]);b=np.array([-.2,.7,.5]);ant=np.array([40.,20.,10.])
  # Ant->intermediate->base translations expressed in common aligned axes.
  chain=ant+R@(b-a)+R@(-b);np.testing.assert_allclose(chain,antenna_to_base(ant,R,a))
 def test_delay_curve_shift_not_raw_rebuild(self):
  l=np.array([2.,0.,0.]);ant=np.array([10.,10.,0.]);R0=np.eye(3);R1=Rotation.from_euler('z',.1).as_matrix()
  # Same antenna sample, different physical attitude assignment.
  self.assertGreater(np.linalg.norm(antenna_to_base(ant,R0,l)-antenna_to_base(ant,R1,l)),.1)
