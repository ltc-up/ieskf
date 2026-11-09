import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.state import State
from geometry_ieskf.external_pose import ExternalPose
from ndt_global.feedback_diagnostics import audit_pose_update,right_jacobian,difference_balance
class FeedbackDiagnosticsTests(unittest.TestCase):
 def setUp(self):
  g=np.random.default_rng(73);A=g.normal(size=(15,15));self.P=(A@A.T+np.eye(15))*.001;self.R=np.diag([.2,.3,.1,.01,.02,.03]);self.lever=np.array([1.2,.2,.1]);self.x=State(np.array([3.,4,5]),np.array([10.,2,0]),Rotation.from_euler('xyz',[.03,-.02,.4]).as_matrix(),np.zeros(3),np.zeros(3));self.T=np.eye(4);self.T[:3,:3]=self.x.R@Rotation.from_rotvec([.002,-.001,.003]).as_matrix();self.T[:3,3]=self.x.p-self.x.R@self.lever+[.02,-.03,.01]
 def test_independent_full_update_matches_original(self):
  o=ExternalPose(self.lever,('x','y','z','rx','ry','rz'));_,Q,info=o.correct(self.x,self.P,self.T,self.R);a=audit_pose_update(self.x,self.P,self.T,self.R,self.lever);self.assertTrue(info['accepted']);np.testing.assert_allclose(a['K'],info['K'],atol=1e-14);np.testing.assert_allclose(a['delta'],info['delta'],atol=1e-14);np.testing.assert_allclose(a['P_post'],Q,atol=1e-14)
 def test_reset_derivative_at_nonzero_rotation(self):
  d=np.array([.07,-.04,.02]);eps=1e-7;J=np.column_stack([(Rotation.from_rotvec(d).inv()*Rotation.from_rotvec(d+eps*np.eye(3)[i])).as_rotvec()/eps for i in range(3)]);np.testing.assert_allclose(J,right_jacobian(d),atol=2e-9)
 def test_whitened_modal_gains_bounded(self):
  a=audit_pose_update(self.x,self.P,self.T,self.R,self.lever);self.assertTrue(np.all(a['modal_gains']>0));self.assertTrue(np.all(a['modal_gains']<1))
 def test_psd_sensitivity_not_calibration(self):
  for scale in [.5,1.,2.]:
   a=audit_pose_update(self.x,self.P,self.T,scale*self.R,self.lever);self.assertGreater(np.linalg.eigvalsh(a['P_post'])[0],0)
 def test_changing_axes_balance_closes(self):
  prior=np.array([[1.,2,0],[2,4,0],[4,5,0]]);post=prior+[.1,-.2,0];ref=np.array([[0.,0,0],[1,1,0],[2,3,0]]);axes=np.array([[1.,0,0],[0,1,0],[.6,.8,0]]);b=difference_balance(prior,post,ref,axes);np.testing.assert_allclose(b['closure'][1:],0,atol=1e-14);np.testing.assert_allclose(b['cumulative_closure'][1:],0,atol=1e-14)
 def test_true_common_motion_is_not_new_error(self):
  ref=np.array([[0.,0,0],[10,4,0],[20,10,0]]);prior=ref+[1,2,0];post=prior.copy();axes=np.tile([0.,1,0],(3,1));b=difference_balance(prior,post,ref,axes);np.testing.assert_allclose(b['new_difference'][1:],0)
