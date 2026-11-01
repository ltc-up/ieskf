import unittest,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments/phase_af_surface_observation'))
import numpy as np
from scipy.linalg import cho_factor,cho_solve
from geometry_ieskf.common import C
from geometry_ieskf.state import inject,chart_map
from geometry_ieskf.surface_observation import *
from geometry_ieskf.partition import POSE
from synthetic_cases import *
class SurfaceObservationTests(unittest.TestCase):
 def setUp(self):self.x,self.P=prior(correlated=True)
 def test_horizontal_nulls_and_rank(self):
  r,H,J,R=observation().linearize(self.x);A=H[:,POSE];self.assertEqual(np.linalg.matrix_rank(A),3);np.testing.assert_allclose(A[:,[0,1,5]],0,atol=1e-14)
 def test_two_plane_rank5(self):
  r,H,J,R=observation('two').linearize(self.x);self.assertEqual(np.linalg.matrix_rank(H[:,POSE]),5);np.testing.assert_allclose(H[:,1],0,atol=1e-14)
 def test_three_and_local_plane_rank6(self):
  for kind in ['three','finite_local']:self.assertEqual(np.linalg.matrix_rank(observation(kind).linearize(self.x)[1][:,POSE]),6)
 def test_finite_coplanar_has_no_edge_information(self):
  H=observation('finite_coplanar').linearize(self.x)[1];np.testing.assert_allclose(H[:,[0,1,8]],0,atol=1e-14)
 def test_full_state_and_plane_jacobians(self):
  o=observation('three',3);x=inject(self.x,np.r_[.1,-.1,.05,np.zeros(3),.03,-.02,.01,np.zeros(6)]);r,H,J,R=o.linearize(x)
  for j in range(15):
   e=np.eye(15)[j]*1e-6;fd=(o.linearize(inject(x,e))[0]-o.linearize(inject(x,-e))[0])/2e-6;np.testing.assert_allclose(fd,H[:,j],atol=2e-8)
  T=o.factor.transform(x);y=o.points@T[:3,:3].T+T[:3,3]
  for i,key in enumerate(o.plane_ids):
   p=o.planes[str(key)]
   for j in range(3):
    e=np.eye(3)[j]*1e-6
    def value(a):n=p.normal+p.tangent@a[:2];n/=np.linalg.norm(n);return n@(y[i]-p.anchor)-a[2]
    self.assertAlmostEqual((value(e)-value(-e))/2e-6,J[i,j],places=7)
 def test_low_rank_effective_R_matches_dense(self):
  r,H,J,R=observation('three',3,True).linearize(self.x);D=R.dense();np.linalg.cholesky(D);np.testing.assert_allclose(D,D.T,atol=1e-14);np.testing.assert_allclose(R.solve(H),np.linalg.solve(D,H),atol=1e-11)
 def test_duplicate_source_ids_do_not_change_information(self):
  o=observation('three',3);r,H,J,R=o.linearize(self.x);base=linear_surface_update(self.P,H,r,R)
  for copies in [2,8,32]:
   repeat=np.repeat(np.arange(len(o.points)),copies);copy=SurfaceObservation(o.points[repeat],o.sample_ids[repeat],o.plane_ids[repeat],o.planes,o.factor);rr,HH,JJ,RR=copy.linearize(self.x);out=linear_surface_update(self.P,HH,rr,RR);np.testing.assert_allclose(out['information'],base['information'],atol=1e-10);np.testing.assert_allclose(out['P_chart'],base['P_chart'],atol=1e-12)
 def test_singular_alias_covariance_has_same_information(self):
  r,H,J,R=observation('horizontal',3).linearize(self.x);base=H.T@R.solve(H);A=np.repeat(np.eye(len(r)),2,axis=0);Dc=A@R.dense()@A.T;np.testing.assert_allclose((A@H).T@np.linalg.pinv(Dc)@(A@H),base,atol=1e-10)
 def test_inconsistent_duplicate_identity_rejected(self):
  o=observation('horizontal',3);p=np.r_[o.points,o.points[:1]+.01];ids=np.r_[o.sample_ids,o.sample_ids[:1]];g=np.r_[o.plane_ids,o.plane_ids[:1]]
  with self.assertRaises(ValueError):SurfaceObservation(p,ids,g,o.planes,o.factor)
 def test_duplicate_plane_registry_not_independent(self):
  p=geometry('horizontal')[3]['plane0'];self.assertEqual(len(plane_registry([p,p,p])),1)
 def test_anchor_covariance_transport_preserves_R(self):
  p=geometry('horizontal',3,True)[3]['plane0'];shift=p.tangent@np.array([2.,-3.]);S,B=transported_plane_covariance(p.covariance,p.normal,p.tangent,shift);pts=geometry('horizontal',3)[0];J=np.c_[(pts-p.anchor)@p.tangent,-np.ones(len(pts))];Jn=np.c_[(pts-p.anchor-shift)@p.tangent,-np.ones(len(pts))];np.testing.assert_allclose(Jn@B,J,atol=1e-13);np.testing.assert_allclose(Jn@S@Jn.T,J@p.covariance@J.T,atol=1e-13)
 def test_dense_independent_samples_have_shared_error_floor(self):
  x,P=prior();vals=[]
  for side in [3,5,9,17,33]:
   r,H,J,R=observation('horizontal',side).linearize(x);out=linear_surface_update(P,H,r,R);N=side**2;expected=1/(1+1/(.1**2+C['lidar_sigma_m']**2/N));self.assertAlmostEqual(out['P_chart'][2,2],expected,places=11);vals.append(out['P_chart'][2,2])
  self.assertTrue(all(v>=1/(1+1/.1**2)-1e-12 for v in vals))
 def test_shared_parent_patch_partition_and_independent_counterexample(self):
  x,P=prior();o=observation('horizontal',9);r,H,J,R=o.linearize(x);base=linear_surface_update(P,H,r,R);last=base['P_chart'][2,2]
  for count in [1,2,4,8]:
   patch=np.arange(len(r))%count;parent=np.array(['plane0' for k in patch]);correct=SharedPlaneNoise(parent,J,o.planes,C['lidar_sigma_m']**2);v=linear_surface_update(P,H,r,correct);np.testing.assert_allclose(v['information'],base['information'],atol=1e-12)
   ids=np.array(['patch'+str(k) for k in patch]);p=o.planes['plane0'];reg={k:Plane(k,p.normal,p.anchor,p.tangent,p.covariance) for k in np.unique(ids)};bad=linear_surface_update(P,H,r,SharedPlaneNoise(ids,J,reg,C['lidar_sigma_m']**2));self.assertLessEqual(bad['P_chart'][2,2],last+1e-12);last=bad['P_chart'][2,2]
 def test_low_support_atomic_reject(self):
  o=observation();r,H,J,R=o.linearize(self.x);p=linear_surface_update(self.P,H,r,R);tx=atomic_surface_update(self.x,self.P,p,False);self.assertFalse(tx['accepted']);self.assertEqual(tx['covariance_updates'],0);np.testing.assert_array_equal(tx['P'],self.P)
 def test_joseph_dense_gain_full_cross_state(self):
  x=inject(self.x,np.r_[.02,-.03,.04,np.zeros(12)]);r,H,J,R=observation('three',3,True).linearize(x);o=linear_surface_update(self.P,H,r,R);S=H@self.P@H.T+R.dense();K=np.linalg.solve(S,H@self.P).T;np.testing.assert_allclose(o['delta'],-K@r,atol=1e-11);np.testing.assert_allclose(o['P_chart'],self.P-K@H@self.P,atol=1e-11);self.assertLess(o['Joseph_relative_error'],1e-12);self.assertGreater(abs(o['delta'][5]),0)
 def test_unobserved_independent_prior_variances_stay(self):
  x,P=prior();r,H,J,R=observation().linearize(x);out=linear_surface_update(P,H,r,R);np.testing.assert_allclose(np.diag(out['P_chart'])[[0,1,8]],np.diag(P)[[0,1,8]],atol=1e-13)
 def test_reset_and_atomic_reject(self):
  x=inject(self.x,np.r_[.01,-.01,.03,np.zeros(12)]);r,H,J,R=observation('three',3).linearize(x);out=linear_surface_update(self.P,H,r,R);tx=atomic_surface_update(x,self.P,out);self.assertTrue(tx['accepted']);G=chart_map(out['delta']);np.testing.assert_allclose(tx['P'],G@out['P_chart']@G.T,atol=1e-12)
  bad=dict(out);bad['delta']=np.r_[.6,np.zeros(14)];no=atomic_surface_update(x,self.P,bad);self.assertFalse(no['accepted']);np.testing.assert_array_equal(no['P'],self.P);np.testing.assert_array_equal(no['state'].p,x.p)
 def test_invalid_covariance_rejected_without_update(self):
  r,H,J,R=observation().linearize(self.x);out=linear_surface_update(self.P,H,r,R);out['P_chart'][0,0]=-1;no=atomic_surface_update(self.x,self.P,out);self.assertFalse(no['accepted']);np.testing.assert_array_equal(no['P'],self.P)
 def test_shared_plane_R_monte_carlo_known_law(self):
  o=observation('horizontal',3,True);r,H,J,R=o.linearize(self.x);rng=np.random.default_rng(20261101);S=o.planes['plane0'].covariance;plane=rng.standard_normal((5000,3))@np.linalg.cholesky(S).T;eps=rng.standard_normal((5000,len(r)))*C['lidar_sigma_m'];draw=plane@J.T+eps;emp=np.cov(draw,rowvar=False);self.assertLess(np.linalg.norm(emp-R.dense())/np.linalg.norm(R.dense()),.06)
class SurfaceExtractionTests(unittest.TestCase):
 def setUp(self):
  u,v=np.meshgrid(np.arange(-5,5.01,.25),np.arange(-5,5.01,.25));self.target=np.c_[u.ravel(),v.ravel(),np.zeros(u.size)];self.query=np.array([[0,0,.1],[2,2,.1],[-2,-2,.1]])
 def test_target_record_copies_do_not_change_surfaces(self):
  from geometry_ieskf.surface_map import SurfaceMap
  a=SurfaceMap(self.target,self.query,5,error_covariance());b=SurfaceMap(np.repeat(self.target,8,axis=0),self.query,5,error_covariance());self.assertGreater(len(a.groups),0);self.assertEqual(len(a.groups),len(b.groups))
  for p,q in zip(a.groups,b.groups):
   for k in ['normal','anchor','target_ids']:np.testing.assert_array_equal(p[k],q[k])
 def test_sparse_support_cannot_be_faked_by_duplicates(self):
  from geometry_ieskf.surface_map import SurfaceMap
  a=SurfaceMap(np.repeat(self.target[:3],20,axis=0),self.query,5,error_covariance());self.assertEqual(len(a.groups),0)
 def test_patch_scale_does_not_invent_tangential_information(self):
  from geometry_ieskf.surface_map import SurfaceMap
  x,P=prior();o=observation('horizontal',3)
  for side in [2.5,5,10]:
   a=SurfaceMap(self.target,self.query,side,error_covariance());self.assertEqual(len(a.groups),1);assoc=a.associate(x,o.points,o.factor);idx=assoc['source_indices'];obs=SurfaceObservation(o.points[idx],idx,assoc['plane_ids'],a.planes,o.factor);H=obs.linearize(x)[1];np.testing.assert_allclose(H[:,[0,1,8]],0,atol=1e-13)
if __name__=='__main__':unittest.main()

