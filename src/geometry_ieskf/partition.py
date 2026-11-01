"""Gaussian partition and historical constraint diagnostics, without private case loading."""
import time
import numpy as np
from scipy.linalg import cho_factor,cho_solve,solve_triangular
from scipy.optimize import least_squares,minimize,nnls,root
from .objective import *
POSE=np.array([0,1,2,6,7,8]);OTHER=np.array([3,4,5,9,10,11,12,13,14]);ORDER=np.r_[POSE,OTHER];PERM=np.eye(15)[ORDER]
BOUNDS=[('position',np.arange(3),C['max_total_translation_m']),('orientation',np.arange(6,9),C['max_total_rotation_rad']),('gyro_bias',np.arange(9,12),C['max_bias_gyro_change_rad_s']),('accel_bias',np.arange(12,15),C['max_bias_accel_change_m_s2'])]
class Partition:
 def __init__(self,o):
  self.o=o;self.P=PERM@o.P@PERM.T;self.Ppp=self.P[:6,:6];self.Ppn=self.P[:6,6:];self.Pnn=self.P[6:,6:];self.Lp=np.linalg.cholesky(self.Ppp);self.Qp=solve_triangular(self.Lp,np.eye(6),lower=True)
  self.K=cho_solve(cho_factor(self.Ppp),self.Ppn).T;self.S=self.Pnn-self.K@self.Ppn;self.S=(self.S+self.S.T)/2;self.Ln=np.linalg.cholesky(self.S);self.Sn=cho_solve(cho_factor(self.S),np.eye(9))
 def join(self,p,n):return PERM.T@np.r_[p,n]
 def recover(self,p):return self.join(p,self.K@p)
 def data(self,p):
  z=self.recover(p);r,J,rho,h=self.o.data(z);return r,J[:,POSE],rho,h
 def reduced_cost(self,p):return self.o.cost(self.recover(p))
 def residual(self,p):
  r,J,rho,h=self.data(p);return np.r_[self.Qp@p,np.sqrt(self.o.corr['w'])*np.sign(r)*np.sqrt(2*rho)]
 def jacobian(self,p):
  r,J,rho,h=self.data(p);v=np.ones_like(r);m=abs(r)>C['huber_delta_m'];v[m]=C['huber_delta_m']/np.sqrt(2*rho[m]);return np.vstack([self.Qp,np.sqrt(self.o.corr['w'])[:,None]*v[:,None]*J])
 def reduced_gradient(self,p):
  r,J,rho,h=self.data(p);return self.Qp.T@(self.Qp@p)+J.T@(self.o.corr['w']*h*r)
def margins(z):return np.array([1-z[ix]@z[ix]/l**2 for name,ix,l in BOUNDS])
def con_jac_z(z):
 A=[]
 for name,ix,l in BOUNDS:
  g=np.zeros(15);g[ix]=-2*z[ix]/l**2;A.append(g)
 return np.array(A)
def kkt(o,z):
 c=margins(z);J=con_jac_z(z)@o.L;g=o.L.T@o.gradient(z);active=np.flatnonzero(c<=1e-5)
 mu=np.zeros(4)
 if len(active):mu[active]=nnls(J[active].T,g)[0]
 station=g-J.T@mu;comp=mu*c;finite=np.isfinite(np.r_[z,c,g,station,mu]).all();feasible=finite and c.min()>=-1e-7;opt=feasible and np.max(abs(station))<=1e-3 and np.max(abs(comp))<=1e-5
 return {'margins':c,'active':[BOUNDS[i][0] for i in active],'multipliers':mu,'gradient_physical':o.gradient(z),'gradient_prior_whitened':g,'feasible_direction_gradient_prior_whitened':station,'stationarity_inf':float(np.max(abs(station))),'complementarity_inf':float(np.max(abs(comp))),'finite':bool(finite),'feasible':bool(feasible),'optimal':bool(opt),'status':('CONVERGED_BOUNDARY' if len(active) else 'CONVERGED_INTERIOR') if opt else ('FEASIBLE_NOT_CONVERGED' if feasible else 'INFEASIBLE')}
def least_reference(o,part=None):
 L=o.L if part is None else part.Lp;hist=[]
 def fun(u):
  z=L@u;r=o.residual(z) if part is None else part.residual(z);hist.append({'cost':float(.5*r@r),'coordinates':z.copy()});return r
 def jac(u):return (o.jacobian(L@u) if part is None else part.jacobian(L@u))@L
 begin=time.perf_counter();sol=least_squares(fun,np.zeros(len(L)),jac=jac,method='trf',loss='linear',x_scale='jac',ftol=1e-14,xtol=1e-14,gtol=1e-11,max_nfev=400)
 coords=L@sol.x;z=coords if part is None else part.recover(coords);r,J,rho,h=o.data(z);M=np.eye(15)+(J@o.L).T@((o.corr['w']*h)[:,None]*(J@o.L));g=o.L.T@o.gradient(z);gn=np.linalg.solve(M,-g)
 return {'z':z,'cost':o.cost(z),'gradient':o.gradient(z),'gradient_white':g,'GN_step_white_norm':float(np.linalg.norm(gn)),'GN_equation_residual':float(np.max(abs(M@gn+g))),'normal_cond_white15':float(np.linalg.cond(M)),'normal_cond_pose_white6':float(np.linalg.cond(np.eye(6)+(J[:,POSE]@(Partition(o).Lp)).T@((o.corr['w']*h)[:,None]*(J[:,POSE]@Partition(o).Lp)))),'status':int(sol.status),'success':bool(sol.success),'message':sol.message,'nfev':int(sol.nfev),'history':hist,'wall_ms':1000*(time.perf_counter()-begin)}
