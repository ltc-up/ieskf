import unittest
import numpy as np
from scipy.integrate import quad
from geometry_ieskf.robust_packet_pose import scale_mixture_moments,ScaleMixturePose
from geometry_ieskf.state import State,chart_map
class RobustPacketPoseTests(unittest.TestCase):
 def setUp(self):self.P=np.eye(15)*.01;self.H=np.eye(15)[:2];self.R=np.eye(2)*.02
 def test_mean_gain_and_total_covariance_identity(self):
  a=scale_mixture_moments(self.P,self.H,self.R,np.array([.1,.2]));np.testing.assert_allclose(a['mean'],a['K']@np.array([.1,.2]),atol=1e-15);np.testing.assert_allclose(a['covariance'],a['within']+a['between'],atol=1e-15);self.assertGreaterEqual(np.linalg.eigvalsh(a['between'])[0],-1e-14);self.assertGreater(np.linalg.eigvalsh(a['covariance'])[0],0)
 def test_scalar_posterior_matches_independent_integration(self):
  p=.1;r=.2;z=.7;H=np.eye(15)[:1];P=np.eye(15)*p;R=np.array([[r]]);a=scale_mixture_moments(P,H,R,np.array([z]),128)
  def likelihood(l):
   s=p+r/l;return 2*l*np.exp(-z*z/(2*s))/np.sqrt(2*np.pi*s)
  den=quad(likelihood,0,1,epsabs=1e-13)[0];mu=quad(lambda l:likelihood(l)*(p/(p+r/l)*z),0,1,epsabs=1e-13)[0]/den;second=quad(lambda l:likelihood(l)*(p*r/l/(p+r/l)+(p/(p+r/l)*z)**2),0,1,epsabs=1e-13)[0]/den;self.assertAlmostEqual(a['mean'][0],mu,places=11);self.assertAlmostEqual(a['covariance'][0,0],second-mu*mu,places=11)
 def test_no_component_more_precise_than_packet(self):
  a=scale_mixture_moments(self.P,self.H,self.R,np.array([0.,0.]));self.assertTrue(np.all(a['precision']<1));self.assertTrue(np.all(a['precision']>0));self.assertGreater(a['covariance'][0,0],1/(1/.01+1/.02))
 def test_large_residual_downweights_without_hard_starvation(self):
  small=scale_mixture_moments(self.P,self.H,self.R,np.array([.1,0.]));large=scale_mixture_moments(self.P,self.H,self.R,np.array([20.,0.]));self.assertLess(large['expected_precision'],small['expected_precision']);self.assertGreater(abs(large['mean'][0]),0);self.assertLess(abs(large['mean'][0]),.02)
 def test_quadrature_converges(self):
  for z in [.01,.5,20.,100.]:
   a=scale_mixture_moments(self.P,self.H,self.R,np.array([z,0.]),128);b=scale_mixture_moments(self.P,self.H,self.R,np.array([z,0.]),256);np.testing.assert_allclose(a['mean'],b['mean'],atol=1e-9);np.testing.assert_allclose(a['covariance'],b['covariance'],atol=1e-9)
 def test_projected_full_state_reset_and_cross_terms(self):
  P=self.P.copy();P[0,1]=P[1,0]=.004;x=State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3));T=np.eye(4);T[1,3]=.02;U=np.eye(6)[1:];y,Q,info=ScaleMixturePose(np.zeros(3),('x','y','z','rx','ry','rz')).correct(x,P,T,np.eye(6)*.02,U);self.assertTrue(info['accepted']);self.assertNotEqual(y.p[0],0);self.assertGreater(np.linalg.eigvalsh(Q)[0],0);d=np.array(info['delta']);B=chart_map(d);H=np.zeros((6,15));H[:3,:3]=np.eye(3);H[3:,6:9]=np.eye(3);a=scale_mixture_moments(P,U@H,U@(np.eye(6)*.02)@U.T,U@np.r_[T[:3,3],[0.,0,0]]);np.testing.assert_allclose(Q,B@a['covariance']@B.T,atol=1e-14)

 def test_local_robust_sensitivity_matches_finite_difference(self):
  r=np.array([.5,.1]);a=scale_mixture_moments(self.P,self.H,self.R,r);eps=1e-6;J=np.column_stack([(scale_mixture_moments(self.P,self.H,self.R,r+eps*np.eye(2)[i])['mean']-scale_mixture_moments(self.P,self.H,self.R,r-eps*np.eye(2)[i])['mean'])/(2*eps) for i in range(2)]);np.testing.assert_allclose(J,a['sensitivity_gain'],atol=1e-9)
