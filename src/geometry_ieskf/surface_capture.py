"""Conditional fixed-R surface capture, feasible steps, frozen full15 prior.

No recursive posterior is returned. Each association has its own labeled local
surrogate; repeated association is not claimed to be one smooth likelihood.
"""
import time,numpy as np
from scipy.optimize import minimize
from .common import C
from .state import inject
from .partition import margins,con_jac_z,kkt
from .capture import step_margins,step_jacobian
from .posterior import valid_prior

class FrozenSurfaceObjective:
 def __init__(self,prior,P,observation):
  self.prior=prior;self.P=valid_prior(P);self.L=np.linalg.cholesky(P);self.obs=observation
  # The entire effective R is frozen at ORIGINAL prediction, even for new pairs.
  self.noise=observation.linearize(prior)[3]
 def data(self,z):
  r,H,_,_=self.obs.linearize(inject(self.prior,z),z);return r,H
 def cost(self,z):
  r,H=self.data(z);u=np.linalg.solve(self.L,z);return float(.5*u@u+.5*self.noise.quadratic(r))
 def gradient(self,z):
  r,H=self.data(z);return np.linalg.solve(self.P,z)+H.T@self.noise.solve(r)
 def local(self,z):
  r,H=self.data(z);A=H@self.L;g=self.L.T@self.gradient(z);M=np.eye(15)+A.T@self.noise.solve(A);M=(M+M.T)/2;du=np.linalg.solve(M,-g);K=kkt(self,z);K.update(gn_white_norm=float(np.linalg.norm(du)),newton_decrement=float(np.sqrt(max(0.,-g@du))),cost=self.cost(z))
  interior=not K['active'];K['qualified']=bool(K['optimal'] and (not interior or max(K['gn_white_norm'],K['newton_decrement'])<=C['convergence_whitened_step']));return g,M,du,K

def constrained_surface_solve(o,z_start=None,max_inner=60):
 z=np.zeros(15) if z_start is None else np.array(z_start,copy=True);history=[];begin=time.perf_counter();status='INNER_BUDGET_EXHAUSTED'
 if min(margins(z))<0:raise ValueError('INFEASIBLE_START')
 for it in range(max_inner):
  g,M,du,certificate=o.local(z)
  if certificate['qualified']:status=certificate['status'];break
  current=z.copy();qpsuccess=True;qpmessage='UNCONSTRAINED_GN_FEASIBLE';qpnit=0
  def c(d):return np.r_[margins(current+o.L@d),step_margins(current+o.L@d,current)]
  def cj(d):return np.vstack([con_jac_z(current+o.L@d),step_jacobian(current+o.L@d,current)])@o.L
  if min(c(du))<0:
   scale=max(1.,abs(o.cost(z)),float(np.max(abs(g))));s=minimize(lambda d:float((g@d+.5*d@M@d)/scale),np.zeros(15),jac=lambda d:(g+M@d)/scale,constraints=[dict(type='ineq',fun=c,jac=cj)],method='SLSQP',options=dict(ftol=1e-10,maxiter=200));du=s.x;qpsuccess=bool(s.success);qpmessage=str(s.message);qpnit=int(s.nit)
  rec=dict(iteration=it,cost_before=certificate['cost'],certificate_before=certificate,untruncated_direction_white=du.copy(),QP_success=qpsuccess,QP_message=qpmessage,QP_iterations=qpnit)
  if not qpsuccess or min(c(du)) < -1e-9 or not np.isfinite(du).all():status='QUADRATIC_SUBPROBLEM_FAILURE';history.append(rec);break
  gd=float(g@du)
  if gd>=0:status='NO_DESCENT_DIRECTION';history.append(rec);break
  accepted=False
  for bt in range(25):
   alpha=2.**(-bt);nextz=z+alpha*(o.L@du);f=o.cost(nextz)
   if min(margins(nextz))>=-1e-9 and min(step_margins(nextz,z))>=-1e-9 and np.isfinite(f) and f<=certificate['cost']+1e-4*alpha*gd:
    accepted=True;break
  if not accepted:status='FIXED_R_LINESEARCH_FAILURE';history.append(rec);break
  rec.update(alpha=alpha,cost_after=f,step=nextz-z,z_after=nextz.copy(),cumulative_margins=margins(nextz),step_margins=step_margins(nextz,z));history.append(rec);z=nextz
 else:it=max_inner
 final=o.local(z)[3]
 if final['qualified']:status=final['status']
 return dict(z=z,status=status,qualified=final['qualified'],certificate=final,history=history,elapsed_s=time.perf_counter()-begin,prior_frozen=True,covariance_updates=0)
