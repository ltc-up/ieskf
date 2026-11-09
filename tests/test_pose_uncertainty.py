import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.pose_uncertainty import euler_xyz_to_body,native_pose_R,conditional_laplace_xy,source_floor_R
from geometry_ieskf.state import State
class UncertaintyTests(unittest.TestCase):
 def test_euler_derivative(self):
  a=np.array([.2,-.3,.6]);R=Rotation.from_euler('XYZ',a).as_matrix();J=euler_xyz_to_body(a)
  for j in range(3):
   d=np.eye(3)[j]*1e-6;fd=(Rotation.from_matrix(R.T@Rotation.from_euler('XYZ',a+d).as_matrix()).as_rotvec()-Rotation.from_matrix(R.T@Rotation.from_euler('XYZ',a-d).as_matrix()).as_rotvec())/2e-6;np.testing.assert_allclose(fd,J[:,j],atol=1e-9)
 def test_native_transport(self):
  a=[.1,.2,.3];R=Rotation.from_euler('XYZ',a).as_matrix();T=np.eye(4);T[:3,:3]=R;x=State(np.zeros(3),np.zeros(3),R,np.zeros(3),np.zeros(3));n=dict(fitness=.2,parameters_internal=[0,0,0,*a],final=T,native_internal=np.diag([1,2,3,.1,.2,.3]));C=native_pose_R(n,('x','y','z','yaw'),x);np.testing.assert_allclose(C[:3,:3],np.diag([1,2,3]));self.assertGreater(np.linalg.eigvalsh(C).min(),0)
 def test_chart_mismatch(self):
  x=State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3));n=dict(fitness=.2,parameters_internal=[0,0,0,.5,0,0],final=np.eye(4),native_internal=np.eye(6))
  with self.assertRaisesRegex(ValueError,'CHART'):native_pose_R(n,('x','y','z','yaw'),x)
 def test_conditional_not_marginal(self):
  I=np.eye(6);I[0,2]=I[2,0]=.5;C=conditional_laplace_xy(-I);self.assertAlmostEqual(C[0,0],1);self.assertGreater(np.linalg.inv(I)[0,0],C[0,0])
 def test_invalid_curvature(self):
  with self.assertRaises(np.linalg.LinAlgError):conditional_laplace_xy(np.eye(6))
 def test_source_floor(self):np.testing.assert_array_equal(source_floor_R([[1,.5],[.5,1]],[2,.1]),np.diag([2,1]))
 def test_duplicate_information(self):
  I=np.diag([1.,2.,3.]);n=100;np.testing.assert_allclose(np.linalg.inv(2*n*I/(2*n)),np.linalg.inv(n*I/n));np.testing.assert_allclose(np.linalg.inv(2*n*I),.5*np.linalg.inv(n*I))
 def test_pitch_second_derivative(self):
  x,y,z,b,c=1.3,.7,2.,.2,.3;f=lambda q:np.cos(q)*np.cos(c)*x-np.cos(q)*np.sin(c)*y+np.sin(q)*z;h=1e-4;fd=(f(b+h)-2*f(b)+f(b-h))/h**2;correct=-np.cos(b)*np.cos(c)*x+np.cos(b)*np.sin(c)*y-np.sin(b)*z
  self.assertAlmostEqual(fd,correct,places=6);self.assertGreater(abs(fd-(correct+2*np.sin(b)*z)),.79)
