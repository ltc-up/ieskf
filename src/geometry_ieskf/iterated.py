from .geometry import *
from scipy.linalg import cho_factor,cho_solve
import time
class IteratedErrorStateUpdate:
 """Frozen-prior manifold-chart Gauss Newton / IRLS, one final covariance update."""
 def __init__(self,factor,config=C):self.factor=factor;self.c=config
 def robust(self,r):
  a=np.abs(r);d=self.c['huber_delta_m'];rho=np.where(a<=d,.5*r*r,d*(a-.5*d));h=np.minimum(1.,d/np.maximum(a,1e-20))
  return rho,h
 def correct(self,prior,P0,source,correspondence):
  start=time.perf_counter();z=np.zeros(15);x=prior.copy();iterations=[];L=np.linalg.cholesky(P0);reason='MAX_ITERATIONS';last=None
  stages={'target_query_ms':0.,'map_normal_ms':0.,'residual_Jacobian_ms':0.,'information_and_MAP_solve_ms':0.,'posterior_information_reset_ms':0.}
  def linearize(*args):
   t=time.perf_counter();value=self.factor.linearize(*args);stages['residual_Jacobian_ms']+=(time.perf_counter()-t)*1000;return value
  def get_correspondence(xx):
   value=correspondence.get(xx,source,self.factor);stages['target_query_ms']+=value['metrics'].get('query_ms',0.);stages['map_normal_ms']+=value['metrics'].get('normal_ms',0.);return value
  def cost(zz,corr):
   xx=inject(prior,zz);rr,_,_=linearize(xx,corr['p'],corr['q'],corr['n']);u=np.linalg.solve(L,zz)
   return .5*u@u+corr['w']@self.robust(rr)[0]
  for iteration in range(self.c['max_iterations']):
   corr=get_correspondence(x);last=corr
   if not corr['good']:return prior,P0,{'accepted':False,'reason':'LOW_SUPPORT','iterations':iterations,'support':corr['metrics'],'stage_ms':stages,'wall_ms':(time.perf_counter()-start)*1000}
   before=cost(z,corr);r,H,J6=linearize(x,corr['p'],corr['q'],corr['n']);B=chart_map(z);Hz=H@B;rho,robust=self.robust(r);w=corr['w']*robust
   # U solver returns new absolute frozen-chart MAP coordinate. Prior never changes.
   solve_start=time.perf_counter();candidate,_,_=prior_update(P0,Hz,r-Hz@z,w);stages['information_and_MAP_solve_ms']+=(time.perf_counter()-solve_start)*1000;step=candidate-z
   alpha=min(1.,self.c['max_translation_step_m']/max(np.linalg.norm(step[:3]),1e-20),self.c['max_rotation_step_rad']/max(np.linalg.norm(step[6:9]),1e-20));scale=None;after=before
   if np.linalg.norm(np.linalg.solve(L,step))<self.c['convergence_whitened_step']:
    reason='CONVERGED';iterations.append({'iteration':iteration,'cost_before':float(before),'cost_after':float(before),'scale':0.,'step_norm':float(np.linalg.norm(step)),'support':corr['metrics']});break
   for s in self.c['line_search_scales']:
    zz=z+alpha*s*step
    if np.linalg.norm(zz[:3])>self.c['max_total_translation_m'] or np.linalg.norm(zz[6:9])>self.c['max_total_rotation_rad'] or np.linalg.norm(zz[9:12])>self.c['max_bias_gyro_change_rad_s'] or np.linalg.norm(zz[12:15])>self.c['max_bias_accel_change_m_s2']:continue
    after=cost(zz,corr)
    if np.isfinite(after) and after<=before-1e-12:scale=alpha*s;break
   if scale is None:
    # Fail the whole frame, rather than publish a partially converged correction as success.
    return prior,P0,{'accepted':False,'reason':'LINE_SEARCH_OR_TRUST_LIMIT','iterations':iterations,'support':corr['metrics'],'stage_ms':stages,'wall_ms':(time.perf_counter()-start)*1000}
   z=z+scale*step;x=inject(prior,z);iterations.append({'iteration':iteration,'cost_before':float(before),'cost_after':float(after),'scale':float(scale),'step_norm':float(np.linalg.norm(scale*step)),'support':corr['metrics']})
  final=get_correspondence(x)
  if not final['good']:return prior,P0,{'accepted':False,'reason':'FINAL_LOW_SUPPORT','iterations':iterations,'support':final['metrics'],'stage_ms':stages,'wall_ms':(time.perf_counter()-start)*1000}
  r,H,J6=linearize(x,final['p'],final['q'],final['n']);B=chart_map(z);Hz=H@B;w=final['w']*self.robust(r)[1];posterior_start=time.perf_counter();A=Hz@L;M=np.eye(15)+A.T@(w[:,None]*A)
  # Conditional IRLS covariance in frozen chart, transported once to new local error.
  post=L@cho_solve(cho_factor(M),L.T);P=B@post@B.T;P=(P+P.T)/2;stages['posterior_information_reset_ms']+=(time.perf_counter()-posterior_start)*1000
  if not np.isfinite(np.r_[z,P.ravel()]).all() or np.linalg.eigvalsh(P).min()<=0:return prior,P0,{'accepted':False,'reason':'INVALID_POSTERIOR','iterations':iterations,'stage_ms':stages,'wall_ms':(time.perf_counter()-start)*1000}
  info={'accepted':True,'reason':reason,'delta':z.tolist(),'iterations':iterations,'support':final['metrics'],'rms_m':float(np.sqrt(np.mean(r*r))),'residual_p50_p95_p99_max_m':np.percentile(abs(r),[50,95,99,100]).tolist(),'Lambda_pose_U':(J6.T@(w[:,None]*J6)).tolist(),'Lambda_full':(Hz.T@(w[:,None]*Hz)).tolist(),'conditional_P_eigen':np.linalg.eigvalsh(P).tolist(),'stage_ms':stages,'wall_ms':(time.perf_counter()-start)*1000,'covariance_updates':1}
  return x,P,info
