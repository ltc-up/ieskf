import unittest
import numpy as np
from geometry_ieskf.valid_correspondence import NearestValidCorrespondence
from geometry_ieskf.qualification import TrackedCorrespondence
from geometry_ieskf.geometry import GeometryFactor
from geometry_ieskf.state import State
class ValidCorrespondenceTests(unittest.TestCase):
 def setUp(self):
  self.x=State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3));self.factor=GeometryFactor(np.zeros(3))
 def preset(self,target,valid):
  q=NearestValidCorrespondence(np.array(target,dtype=float));q.cached[:]=True;q.valid[:]=valid;q.normals[:]=[0,0,1.];return q
 def test_invalid_nearest_does_not_hide_valid_second(self):
  q=self.preset([[0,0,0],[.1,0,0]],[False,True]);c=q.get(self.x,np.array([[0.,0,0]]),self.factor)
  np.testing.assert_array_equal(c['target_ids'],[1]);self.assertEqual(c['metrics']['nearest_valid_rescued_count'],1)
 def test_original_radius_never_expanded(self):
  q=self.preset([[0,0,0],[2.,0,0]],[False,True]);c=q.get(self.x,np.array([[0.,0,0]]),self.factor);self.assertEqual(len(c['p']),0)
 def test_all_invalid_stays_unsupported(self):
  q=self.preset([[0,0,0],[.1,0,0]],[False,False]);self.assertEqual(len(q.get(self.x,np.array([[0.,0,0]]),self.factor)['p']),0)
 def test_target_normalization_unchanged(self):
  q=self.preset([[0,0,0],[.1,0,0]],[False,True]);c=q.get(self.x,np.array([[0.,0,0],[.01,0,0]]),self.factor);self.assertAlmostEqual(c['w'].sum(),1/q.c['lidar_sigma_m']**2)
 def test_normal_fit_matches_original_on_shared_targets(self):
  target=np.array([[x,y,0.] for x in np.linspace(-2,2,9) for y in np.linspace(-2,2,9)]);src=np.array([[.01,.02,.05],[.51,.02,.05]])
  a=TrackedCorrespondence(target);b=NearestValidCorrespondence(target);ca=a.get(self.x,src,self.factor);cb=b.get(self.x,src,self.factor)
  np.testing.assert_array_equal(ca['target_ids'],cb['target_ids']);np.testing.assert_allclose(abs(np.einsum('ij,ij->i',ca['n'],cb['n'])),1);np.testing.assert_array_equal(ca['w'],cb['w'])
 def test_nearest_valid_choice_matches_brute_force(self):
  q=self.preset([[0,0,0],[.2,0,0],[.1,.1,0],[.7,0,0]],[False,True,True,True]);src=np.array([[.01,0,0],[.66,0,0]])
  c=q.get(self.x,src,self.factor);expected=[int(np.flatnonzero(q.valid)[np.argmin(np.linalg.norm(q.target[q.valid]-p,axis=1))]) for p in src];np.testing.assert_array_equal(c['target_ids'],expected)
if __name__=='__main__':unittest.main()
