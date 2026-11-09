import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from ndt_global.matcher_geometry import ndt_native_information,gicp_information,describe,compare
from geometry_ieskf.pose_uncertainty import euler_xyz_to_body
class MatcherGeometryTests(unittest.TestCase):
 def test_native_transport_full_covariance_inverse(self):
  angles=np.array([.13,-.21,.7]);R=Rotation.from_euler('XYZ',angles).as_matrix();rng=np.random.default_rng(9);Q=rng.normal(size=(6,6));I=Q@Q.T+np.eye(6);kappa=2.;C=kappa*np.linalg.inv(I);row=dict(final=np.block([[R,np.zeros((3,1))],[np.zeros((1,3)),np.ones((1,1))]]),parameters_internal=np.r_[np.zeros(3),angles],native_internal=C,kappa_at_float_pose=kappa);A=np.zeros((6,6));A[:3,:3]=R;A[3:,3:]=np.linalg.inv(euler_xyz_to_body(angles))/10;np.testing.assert_allclose(ndt_native_information(row),A.T@I@A,atol=1e-13);self.assertGreater(np.linalg.norm(ndt_native_information(row)-np.diag(np.diag(ndt_native_information(row)))),.1)
 def test_gicp_axis_order_and_units(self):
  H=np.diag([100,200,300,4,5,6.]);out=gicp_information(dict(H_right_rotation_translation=H,source_n=10));np.testing.assert_allclose(np.diag(out),[.4,.5,.6,.1,.2,.3])
 def test_shape_not_raw_scale(self):
  I=np.diag([1,2,3,4,5,6.]);r=compare(I,1000*I);self.assertAlmostEqual(r['trace_shape_cosine'],1);self.assertAlmostEqual(r['weakest_axis_abs_cosine'],1)
 def test_nuisance_coupling_is_not_diagonal_information(self):
  I=np.eye(6);I[1,5]=I[5,1]=.8;d=describe(I);self.assertAlmostEqual(d['marginal_translation_information'][1,1],.36);self.assertAlmostEqual(d['normalized_coupling'][1,5],.8);self.assertNotEqual(d['conditional_axis_energy'][1],d['marginal_translation_energy'][1])
 def test_singular_geometry_fails_without_manufactured_covariance(self):
  with self.assertRaises(ValueError):describe(np.diag([0.,1,1,1,1,1]))
 def test_gicp_factor_right_twist_jacobian(self):
  R=Rotation.from_euler('xyz',[.1,-.3,.5]).as_matrix();p=np.array([2.,1.,.7]);eps=1e-6;skew=np.array([[0,-p[2],p[1]],[p[2],0,-p[0]],[-p[1],p[0],0]]);J=np.c_[R@skew,-R]
  def residual(d):return -R@(Rotation.from_rotvec(d[:3]).apply(p)+d[3:])
  fd=np.column_stack([(residual(eps*np.eye(6)[j])-residual(-eps*np.eye(6)[j]))/(2*eps) for j in range(6)]);np.testing.assert_allclose(fd,J,atol=1e-9)
