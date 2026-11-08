import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.motion_audit import derivative_audit,gain_decomposition
from geometry_ieskf.state import State
from geometry_ieskf.inertial import InertialPredictor
class MotionAuditTests(unittest.TestCase):
 def test_right_error_F_G_H(self):
  rng=np.random.default_rng(421)
  for _ in range(12):
   x=State(rng.normal(size=3),rng.normal(size=3)*15,Rotation.from_rotvec(rng.normal(size=3)).as_matrix(),rng.normal(size=3)*.02,rng.normal(size=3)*.2);r=derivative_audit(x,rng.normal(size=3),rng.normal(size=3)*10,.007,rng.normal(size=3))
   for v in r.values():self.assertLess(v,2e-7)
 def test_gain_identity_with_cross_correlations(self):
  rng=np.random.default_rng(91);A=rng.normal(size=(15,15));P=A@A.T+np.eye(15);x=State(np.zeros(3),np.array([15.,2.,1.]),Rotation.from_euler('xyz',[.05,.1,.3]).as_matrix(),np.zeros(3),np.zeros(3));r=gain_decomposition(x,P,np.array([.1,.2,.3]),16.,.01,[1.,.1,.2]);self.assertAlmostEqual(sum(r['Kz_by_block'].values()),r['K'][2]);self.assertAlmostEqual(r['K'][2]*r['innovation'],r['delta'][2])
 def test_static_specific_force_and_linear_acceleration_differ(self):
  x=State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3));m=InertialPredictor();a=m.model(x,np.zeros(3),np.array([0,0,9.80665]),.01)[0];b=m.model(x,np.zeros(3),np.zeros(3),.01)[0];self.assertLess(np.linalg.norm(a.v),1e-12);self.assertLess(b.v[2],-.09)
 def test_ZOH_Q_is_not_exact_continuous_white_noise(self):
  x=State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3));m=InertialPredictor();dt=.01;y,F,G,Q=m.model(x,np.zeros(3),np.zeros(3),dt);disc=G@Q@G.T;exact=m.c['accel_noise_density_m_s2_sqrt_hz']**2*dt**3/3;self.assertAlmostEqual(disc[0,0]/exact,.75)
if __name__=='__main__':unittest.main()

class VelocityFrameTests(unittest.TestCase):
 def test_sensor_axis_lever_H(self):
  from geometry_ieskf.motion_audit import sensor_velocity_model
  from geometry_ieskf.state import inject
  x=State(np.ones(3),np.array([12.,2.,1.]),Rotation.from_euler('xyz',[.2,-.1,.4]).as_matrix(),np.ones(3)*.01,np.zeros(3));omega=np.array([.2,.3,.4]);args=(np.array([1.,.1,.2]),np.array([2.,.3,.4]),np.array([.8,.6,0.]));pred,H=sensor_velocity_model(x,omega,*args)
  for j in range(15):
   eps=1e-6;d=np.eye(15)[j]*eps;fd=(sensor_velocity_model(inject(x,d),omega,*args)[0]-sensor_velocity_model(inject(x,-d),omega,*args)[0])/(2*eps);self.assertAlmostEqual(fd,H[0,j],places=7)
 def test_absolute_height_unobservable_to_velocity(self):
  from geometry_ieskf.wheel import WheelObservation
  x=State(np.zeros(3),np.array([12.,0.,1.]),np.eye(3),np.zeros(3),np.zeros(3));y=x.copy();y.p[2]=1000;w=WheelObservation([1.,0.,0.]);self.assertEqual(w.model(x,np.zeros(3))[0],w.model(y,np.zeros(3))[0]);np.testing.assert_array_equal(w.model(x,np.zeros(3))[1][:,:3],0)
