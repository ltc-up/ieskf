import unittest
import numpy as np
from geometry_ieskf.density_audit import FrozenNormalsView,density_arms,block_partition
from geometry_ieskf.information_audit import decompose,curvature_summary
from geometry_ieskf.association_objective import CompactHuberMixture
from geometry_ieskf.valid_correspondence import NearestValidCorrespondence
from geometry_ieskf.geometry import GeometryFactor
from geometry_ieskf.state import State
class InformationAuditTests(unittest.TestCase):
 def setUp(self):
  self.target=np.array([[x,y,0.] for x in np.arange(-1.,1.01,.25) for y in np.arange(-1.,1.01,.25)])
  self.q=NearestValidCorrespondence(self.target);self.q.cached[:]=True;self.q.valid[:]=True;self.q.normals[:]=[0,0,1.]
  self.x=State(np.array([0.,0.,.15]),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3));self.factor=GeometryFactor([.05,.02,.01]);self.src=np.array([[.1,.2,0],[.4,-.2,.1],[-.3,.25,.2]])
  A=np.eye(15);A[3,0]=.3;A[9,6]=.2;self.P=A@A.T;self.z=np.linspace(-.01,.01,15)
 def objective(self,q=None):return CompactHuberMixture(self.x,self.P,self.src,self.factor,q or self.q)
 def test_complete_signed_reconstruction(self):
  d=decompose(self.objective(),self.z);self.assertLess(d['reconstruction_relative'],2e-6)
 def test_force_reconstruction(self):
  d=decompose(self.objective(),self.z);self.assertLess(d['force_sum_error'],1e-10)
 def test_prior_is_identity_in_full_white_chart(self):
  d=decompose(self.objective(),self.z);np.testing.assert_array_equal(d['parts_white']['prior'],np.eye(15))
 def test_negative_responsibility_covariance(self):
  d=decompose(self.objective(),self.z);self.assertLess(np.linalg.eigvalsh(d['parts_white']['residual_responsibility_covariance']).max(),1e-8)
 def test_signed_kernel_first_terms(self):
  d=decompose(self.objective(),self.z);self.assertGreater(np.linalg.eigvalsh(d['parts_white']['numerator_kernel_first']).min(),-1e-8);self.assertLess(np.linalg.eigvalsh(d['parts_white']['normalization_density_first']).max(),1e-8)
 def test_pure_density_preserves_normal_and_identity_aliases(self):
  ids=np.repeat(np.arange(len(self.target)),2);q=FrozenNormalsView(self.q,ids);q.warm_normals(np.arange(len(ids)));np.testing.assert_array_equal(q.normals,self.q.normals[ids]);np.testing.assert_array_equal(q.valid,self.q.valid[ids])
 def test_duplicated_view_matches_materialized_same_planes(self):
  ids=np.repeat(np.arange(len(self.target)),2);a=FrozenNormalsView(self.q,ids);b=NearestValidCorrespondence(self.target[ids]);b.cached[:]=True;b.valid[:]=True;b.normals[:]=[0,0,1.]
  self.assertAlmostEqual(self.objective(a).cost(self.z),self.objective(b).cost(self.z),places=11)
 def test_sampling_is_deterministic_without_coordinate_jitter(self):
  a,meta=density_arms(self.target,np.arange(len(self.target)));b,_=density_arms(self.target,np.arange(len(self.target)))
  for k in a:np.testing.assert_array_equal(a[k],b[k]);self.assertTrue(np.all(a[k]<len(self.target)))
  self.assertEqual(len(a['B']),2*len(self.target))
 def test_nonpositive_curvature_does_not_invent_sigma(self):
  H=np.eye(15);H[0,0]=-1;r=curvature_summary(self.objective(),self.z,H);self.assertFalse(r['SPD']);self.assertNotIn('sigma_xyz',r)
 def test_block_cores_have_disjoint_large_gap(self):
  p=np.array([[5,5,0],[25,5,0],[45,5,0],[1,1,0]])
  tr,va=block_partition(p);self.assertFalse(set(tr)&set(va));self.assertNotIn(3,set(tr)|set(va));self.assertGreater(np.min(np.linalg.norm(p[tr,None,:]-p[va,:],axis=2)),5.)
if __name__=='__main__':unittest.main()
