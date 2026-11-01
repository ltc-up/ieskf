import unittest,numpy as np
from scipy.spatial.transform import Rotation as Q
from geometry_ieskf.deskew_ablation import compensate
class DeskewAblationTests(unittest.TestCase):
 def setUp(self):self.r=np.array([1.,.2,.1]);self.ref=Q.from_euler('xyz',[.1,-.2,.4]).as_matrix();self.pref=np.array([3.,2.,1.]);self.points=np.array([[2.,3.,4.],[-1.,.2,2.]])
 def test_reference_all_identity(self):
  for m in ['A','B','C']:np.testing.assert_allclose(compensate(self.points,np.repeat(self.ref[None],2,axis=0),np.tile(self.pref,(2,1)),self.ref,self.pref,self.r,m),self.points,atol=2e-15)
 def test_B_keeps_pure_body_rotation_and_LiDAR_lever_height(self):
  R=Q.from_euler('xyz',[[.15,-.3,.6],[.05,-.1,.2]]).as_matrix();p=np.tile(self.pref,(2,1));np.testing.assert_allclose(compensate(self.points,R,p,self.ref,self.pref,self.r,'A'),compensate(self.points,R,p,self.ref,self.pref,self.r,'B'),atol=1e-14)
 def test_B_removes_only_kernel_world_base_z(self):
  R=np.repeat(self.ref[None],2,axis=0);p=np.tile(self.pref,(2,1))+[[.2,.3,.4],[-.1,.4,-.2]];a=compensate(self.points,R,p,self.ref,self.pref,self.r,'A');b=compensate(self.points,R,p,self.ref,self.pref,self.r,'B');np.testing.assert_allclose((a-b)@self.ref.T,np.c_[np.zeros((2,2)),p[:,2]-self.pref[2]],atol=1e-14)
 def test_C_keeps_yaw_and_reference_RP(self):
  yaw=np.array([.6,.9]);R=Q.from_euler('xyz',np.c_[[.3,-.2],[.4,-.1],yaw]).as_matrix();expected=Q.from_euler('xyz',np.c_[[.1,.1],[-.2,-.2],yaw]).as_matrix();p=np.tile(self.pref,(2,1))+[[.2,.1,.3],[.1,-.1,-.2]];np.testing.assert_allclose(compensate(self.points,R,p,self.ref,self.pref,self.r,'C'),compensate(self.points,expected,p,self.ref,self.pref,self.r,'A'),atol=1e-14)
 def test_C_no_change_when_RP_constant(self):
  R=Q.from_euler('xyz',[[.1,-.2,.6],[.1,-.2,-.5]]).as_matrix();p=np.tile(self.pref,(2,1));np.testing.assert_allclose(compensate(self.points,R,p,self.ref,self.pref,self.r,'A'),compensate(self.points,R,p,self.ref,self.pref,self.r,'C'),atol=1e-14)
if __name__=='__main__':unittest.main()
