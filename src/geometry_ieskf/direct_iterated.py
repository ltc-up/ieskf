"""Conditional fixed-map direct residual MAP, one prior and one information commit.

RPC returns freshly relinearized per-point geometric factors, not a pose/R.
Global coordinates z use the frozen prior's SO(3) right chart. Line searches
freeze associations and metric; outer refresh is explicitly a distinct objective.
"""
import time
import numpy as np
from scipy.linalg import cho_factor,cho_solve
from .state import inject,chart_map,difference
from .common import C,skew,jr,exp
from .posterior import valid_prior

def base_pose(x,lever):
 T=np.eye(4);T[:3,:3]=x.R;T[:3,3]=x.p-x.R@lever;return T

def pose_chart(x,lever,z):
 """Right-SE3 [rot,trans] from frozen-prior 15D coordinates at x(z)."""
 A=np.zeros((6,15));A[:3,6:9]=jr(z[6:9]);A[3:,:3]=x.R.T;A[3:,6:9]=skew(lever)@jr(z[6:9]);return A

def augmented_chart(x,lever,z):
 A=pose_chart(x,lever,z)
 if len(z)==15:return A
 nu=z[15:];Rn=exp(nu[:3]);M=np.zeros((6,6));M[:3,:3]=Rn.T;M[3:,3:]=Rn.T;M[3:,:3]=-Rn.T@skew(nu[3:])
 return np.column_stack([M@A,np.block([[jr(nu[:3]),np.zeros((3,3))],[np.zeros((3,3)),Rn.T]])])

class GeometryClient:
 def __init__(self,process):self.process=process;self.calls=0;self.timings=[]
 def call(self,request):
  import json
  tick=time.perf_counter()
  self.process.stdin.write(json.dumps(request)+'\n');self.process.stdin.flush();s=self.process.stdout.readline()
  if not s:raise ValueError('DIRECT_SERVICE_TERMINATED')
  r=json.loads(s);self.calls+=1;self.timings.append(dict(op=request.get('op','refresh'),wall_ms=1000*(time.perf_counter()-tick),service_ms=r.get('service_ms'),source_cache_hit=r.get('source_cache_hit')))
  if 'error' in r:raise ValueError('SERVICE_'+r['error'])
  return r

class DirectIterated:
 def __init__(self,lever,client,request,max_iterations=12):self.lever=np.asarray(lever);self.client=client;self.request=request;self.max_iterations=max_iterations
 def query(self,x,op,nu=None):
  T=base_pose(x,self.lever)
  if nu is not None:
   E=np.eye(4);E[:3,:3]=exp(nu[:3]);E[:3,3]=nu[3:];T=T@E
  return self.client.call(self.request|dict(shared_frame=False,seed_common=T.tolist(),op=op))
 def correct(self,x,P,unused_pose=None,unused_R=None,projection=None):
  tick=time.perf_counter();info=dict(accepted=False,covariance_updates=0,iterations=[],calibrated=False,direct_geometry=True)
  def fail(reason):return x.copy(),P.copy(),info|dict(reason=reason,wall_ms=1000*(time.perf_counter()-tick))
  try:
   valid_prior(P);augmented=self.request.get('shared_frame',False);dim=21 if augmented else 15;prior=np.zeros((dim,dim));prior[:15,:15]=P
   if augmented:prior[15:,15:]=np.diag([np.deg2rad(1.)**2]*3+[.5**2]*3)
   L=np.linalg.cholesky(prior);Pinv=cho_solve(cho_factor(prior),np.eye(dim));z=np.zeros(dim);y=x.copy();converged=False
   for it in range(self.max_iterations):
    q=self.query(y,'refresh' if it<2 else 'evaluate',z[15:] if augmented else None)
    if not q['candidate_valid']:return fail('DIRECT_INSUFFICIENT_GEOMETRY')
    A=augmented_chart(y,self.lever,z)
    H6=np.array(q['H']);g6=np.array(q['gradient']).ravel();H=A.T@H6@A;g=A.T@g6;N=Pinv+H;N=(N+N.T)/2;grad=Pinv@z+g;step=-cho_solve(cho_factor(N),grad);white=float(np.linalg.norm(np.linalg.solve(L,step)));objective=.5*z@Pinv@z+q['cost'];record=dict(iteration=it,objective=float(objective),cost=q['cost'],factors=q['factors'],groups=q['groups'],association_changes=q['correspondence_changes'],white_step=white,gradient_white=float(np.linalg.norm(L.T@grad)),information_eigenvalues=np.linalg.eigvalsh(H6).tolist());record['iterate_chart']=z.tolist();record['proposal_norms_p_v_theta_bg_ba']=[float(np.linalg.norm((z+step)[k:k+3])) for k in (0,3,6,9,12)];info['iterations'].append(record)
    if white<1e-3:converged=True;break
    accepted=False
    for alpha in (1.,.5,.25,.125,.0625,.03125):
     proposal=z+alpha*step
     if np.linalg.norm(proposal[6:9])>C['max_total_rotation_rad'] or np.linalg.norm(proposal[9:12])>C['max_bias_gyro_change_rad_s'] or np.linalg.norm(proposal[12:15])>C['max_bias_accel_change_m_s2']:continue
     candidate=inject(x,proposal[:15]);trial=self.query(candidate,'evaluate',proposal[15:] if augmented else None);f=.5*proposal@Pinv@proposal+trial['cost']
     if trial['candidate_valid'] and f<=objective+1e-10:
      z=proposal;y=candidate;record.update(step_scale=alpha,frozen_trial_objective=float(f));accepted=True;break
    if not accepted:return fail('DIRECT_FROZEN_LINESEARCH_FAILED')
   if not converged:return fail('DIRECT_MAX_ITERATIONS')
   # Final fixed-association linearization is at the converged state. Commit once.
   Qjoint=cho_solve(cho_factor(N),np.eye(dim));Qz=Qjoint[:15,:15];B=chart_map(z[:15]);Q=B@Qz@B.T;Q=(Q+Q.T)/2;valid_prior(Q)
   # Aggregate equivalent Joseph identity in the prior chart (no dense point R).
   KH=Qjoint@H;Joseph=(np.eye(dim)-KH)@prior@(np.eye(dim)-KH).T+Qjoint@H@Qjoint;err=float(np.max(abs(Joseph-Qjoint)))
   if err>1e-8:return fail('DIRECT_JOSEPH_MISMATCH')
   info['rpc_timings']=getattr(self.client,'timings',[]).copy()
   info['prior_state']={k:getattr(x,k).tolist() for k in ['p','v','R','bg','ba']};info['prior_state']['t']=x.t
   info.update(accepted=True,reason='DIRECT_GEOMETRY_UPDATE',delta=z[:15].tolist(),delta_proposed=z[:15].tolist(),shared_nuisance=z[15:].tolist(),covariance_updates=1,Joseph_error=err,final_H15=H[:15,:15].tolist(),final_gradient15=g[:15].tolist(),joint_dimension=dim,posterior_chart=Qz.tolist(),wall_ms=1000*(time.perf_counter()-tick),converged=True,convergence_scope='FROZEN_ASSOCIATION_CONDITIONAL_MAP',association_refreshes=min(2,len(info['iterations'])))
   return y,Q,info
  except (ValueError,np.linalg.LinAlgError,KeyError) as e:return fail(str(e))
