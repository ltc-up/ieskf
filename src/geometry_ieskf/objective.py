"""Independent fixed-chart Huber MAP research objective. Not a production update."""
from pathlib import Path
import json,time
import numpy as np
from scipy.linalg import solve_triangular
from scipy.optimize import least_squares
from .common import C,sha,prior_update
from .state import State,inject,difference,chart_map
from .geometry import GeometryFactor
def dump(p,x):
 def conv(v):
  if isinstance(v,np.ndarray):return v.tolist()
  if isinstance(v,np.generic):return v.item()
  raise TypeError(str(type(v)))
 Path(p).write_text(json.dumps(x,indent=2,default=conv,allow_nan=False))
def state(s):return State(np.array(s['p_imu_map']),np.array(s['v_imu_map']),__import__('scipy').spatial.transform.Rotation.from_quat(s['q_map_body_xyzw']).as_matrix(),np.array(s['bg_body_rad_s']),np.array(s['ba_body_m_s2']),s['state_time_ns'])
def corr_npz(p):
 a=np.load(p);return {'p':a['source_supported'],'q':a['target_supported'],'n':a['normal_map'],'w':a['base_weights'],'ids':a['target_ids'],'indices':a['source_index']}
def violations(z):
 return [k for k,s,l in [('translation',slice(0,3),C['max_total_translation_m']),('rotation',slice(6,9),C['max_total_rotation_rad']),('gyro_bias',slice(9,12),C['max_bias_gyro_change_rad_s']),('accel_bias',slice(12,15),C['max_bias_accel_change_m_s2'])] if np.linalg.norm(z[s])>l]
class Objective:
 def __init__(self,prior,P,corr,factor):self.prior=prior;self.P=P;self.corr=corr;self.factor=factor;self.L=np.linalg.cholesky(P);self.Q=solve_triangular(self.L,np.eye(15),lower=True)
 def data(self,z):
  r,H,_=self.factor.linearize(inject(self.prior,z),self.corr['p'],self.corr['q'],self.corr['n']);J=H@chart_map(z);a=abs(r);d=C['huber_delta_m'];rho=np.where(a<=d,.5*r*r,d*(a-.5*d));rob=np.minimum(1.,d/np.maximum(a,1e-20));return r,J,rho,rob
 def cost(self,z):
  r,J,rho,h=self.data(z);return float(.5*np.sum((self.Q@z)**2)+self.corr['w']@rho)
 def gradient(self,z):
  r,J,rho,h=self.data(z);return self.Q.T@(self.Q@z)+J.T@(self.corr['w']*h*r)
 def residual(self,z):
  r,J,rho,h=self.data(z);return np.r_[self.Q@z,np.sqrt(self.corr['w'])*np.sign(r)*np.sqrt(2*rho)]
 def jacobian(self,z):
  r,J,rho,h=self.data(z);s=np.ones_like(r);m=abs(r)>C['huber_delta_m'];s[m]=C['huber_delta_m']/np.sqrt(2*rho[m]);return np.vstack([self.Q,np.sqrt(self.corr['w'])[:,None]*s[:,None]*J])
 def surrogate(self,at,z):
  _,_,rho,h=self.data(at);r,*_=self.data(z);r0,*_=self.data(at);constant=self.corr['w']@(rho-.5*h*r0*r0);return float(.5*np.sum((self.Q@z)**2)+.5*self.corr['w']@(h*r*r)+constant)
 def independent(self,free=None):
  # Geometry-only robust transform leaves prior quadratic; SciPy loss stays linear.
  free=np.arange(15) if free is None else np.asarray(free);history=[]
  def expand(u):z=np.zeros(15);z[free]=u;return z
  def fun(u):
   z=expand(u);r=self.residual(z);history.append({'evaluation':len(history),'objective':float(.5*r@r),'z':z.copy()});return r
  begin=time.perf_counter();sol=least_squares(fun,np.zeros(len(free)),jac=lambda u:self.jacobian(expand(u))[:,free],method='trf',loss='linear',x_scale='jac',ftol=1e-12,xtol=1e-12,gtol=1e-10,max_nfev=300)
  z=expand(sol.x);return {'z':z,'cost':self.cost(z),'gradient':self.gradient(z),'free_gradient_inf':float(np.max(abs(self.gradient(z)[free]))),'success':bool(sol.success),'status':int(sol.status),'message':sol.message,'nfev':int(sol.nfev),'optimality':float(sol.optimality),'guard_violations':violations(z),'history':history,'wall_ms':1000*(time.perf_counter()-begin)}
