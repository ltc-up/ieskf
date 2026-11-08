import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.state import State,inject,difference
from geometry_ieskf.external_pose import ExternalPose
from ndt_global.pose_delay import PoseDelayReplay
class PoseTests(unittest.TestCase):
 def state(self):return State(np.array([1.,2.,3.]),np.array([5.,0.,0.]),Rotation.from_euler('xyz',[.1,-.2,.3]).as_matrix(),np.zeros(3),np.zeros(3),1_000_000_000)
 def test_FD_xyz_yaw_and_SO3(self):
  x=self.state()
  for components in [('x','y','z','yaw'),('x','y','z','rx','ry','rz')]:
   obs=ExternalPose([1.,.2,.1],components);anchor=Rotation.from_euler('xyz',[.03,.05,.06]).as_matrix();h,H=obs.model(x,anchor)
   for j in range(15):
    d=np.eye(15)[j]*1e-6;fd=(obs.model(inject(x,d),anchor)[0]-obs.model(inject(x,-d),anchor)[0])/2e-6;np.testing.assert_allclose(fd,H[:,j],atol=1e-8)
 def test_cross_coupling_retained(self):
  x=self.state();P=np.eye(15);P[2,5]=P[5,2]=.5;T=np.eye(4);T[:3,:3]=x.R;T[:3,3]=x.p+[0,0,.1];o=ExternalPose([0,0,0],('z',));y,Q,i=o.correct(x,P,T,np.eye(1));self.assertTrue(i['accepted']);self.assertGreater(y.v[2],0);self.assertLess(np.linalg.eigvalsh(Q).min(),1)
 def test_rejected_update_is_atomic(self):
  x=self.state();P=np.eye(15);y,Q,i=ExternalPose([0,0,0]).correct(x,P,np.eye(4),np.zeros((4,4)));self.assertFalse(i['accepted']);np.testing.assert_array_equal(P,Q);np.testing.assert_array_equal(x.p,y.p)
 def test_projection_preserves_full_covariance(self):
  x=self.state();T=np.eye(4);T[:3,:3]=x.R;T[:3,3]=x.p+[.1,0,0];o=ExternalPose([0,0,0]);y,Q,i=o.correct(x,np.eye(15),T,np.eye(4),np.eye(4)[:2]);self.assertTrue(i['accepted']);np.testing.assert_allclose(Q,Q.T);self.assertGreater(np.linalg.eigvalsh(Q).min(),0)
 def test_delayed_pose_repropagates_once(self):
  x=self.state();imu=dict(header_ns=x.t,omega=np.zeros(3),force=x.R.T@np.array([0,0,9.80665]));a=PoseDelayReplay(x,np.eye(15),imu,[0,0,0]);b=PoseDelayReplay(x,np.eye(15),imu,[0,0,0]);T=np.eye(4);T[:3,:3]=x.R;T[:3,3]=x.p+[.1,0,0];nd=dict(kind='ndt',header_ns=x.t+5_000_000,message_id=1,correlation_group='g',observer=ExternalPose([0,0,0]),pose=T,R=np.eye(4));im=dict(kind='imu',header_ns=x.t+10_000_000,message_id=2,omega=imu['omega'],force=imu['force']);a.ingest(dict(nd));a.ingest(im);b.ingest(im);b.ingest(dict(nd));np.testing.assert_allclose(difference(a.current()[0],b.current()[0]),0,atol=1e-13);np.testing.assert_array_equal(a.current()[1],b.current()[1]);self.assertEqual(b.ingest(dict(nd)),'DUPLICATE');self.assertEqual(b.ingest(dict(nd,message_id=99)),'DUPLICATE_INFORMATION_GROUP')
if __name__=='__main__':unittest.main()
