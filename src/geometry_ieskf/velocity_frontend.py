"""Causal [velocity, derivative, second derivative] optical frontend.
Independent implementation of standard integrated white-jerk equations.
"""
import numpy as np
class PolynomialVelocity:
 def __init__(self,legacy=False,q=1.,sigma=.01):self.legacy=legacy;self.q=q;self.r=sigma**2;self.t=None;self.x=None;self.P=np.eye(3)
 @staticmethod
 def matrices(dt,q=1.,legacy=False):
  h=float(dt);F=np.array([[1,h,h*h/2],[0,1,h],[0,0,1.]])
  Q=np.diag([0.,0.,h]) if legacy else np.array([[h**5/20,h**4/8,h**3/6],[h**4/8,h**3/3,h*h/2],[h**3/6,h*h/2,h]])
  return F,Q*q
 def push(self,t,z):
  if not np.isfinite([t,z]).all():raise ValueError('INVALID_INPUT')
  if self.t is not None and t<=self.t:raise ValueError('TIME_NOT_INCREASING')
  if self.t is None or (not self.legacy and t-self.t>1.):self.x=np.array([z,0.,0.]);self.P=np.eye(3);self.t=t;return
  F,Q=self.matrices(t-self.t,self.q,self.legacy);u=F@self.x;P=F@self.P@F.T+Q;K=P[:,0]/(P[0,0]+self.r);self.x=u+K*(z-u[0]);A=np.eye(3);A[:,0]-=K;self.P=A@P@A.T+self.r*np.outer(K,K);self.P=(self.P+self.P.T)/2;self.t=t
 def query(self,t):
  if self.t is None or not 0<=t-self.t<=1.:raise ValueError('STALE_OR_FUTURE_QUERY')
  F,Q=self.matrices(t-self.t,self.q,self.legacy);return F@self.x,F@self.P@F.T+Q

class BlockVelocity:
 """Once-only quadratic forecast from a completed past sensor block."""
 @staticmethod
 def estimate(stamps,values,query_ns,noise,common=.25):
  t=np.asarray(stamps,dtype=np.int64);Y=np.asarray(values,float);R=np.asarray(noise,float)
  if len(t)<6 or np.any(np.diff(t)<=0) or t[-1]>=query_ns or query_ns-t[-1]>10_000_000 or Y.shape!=(len(t),2):raise ValueError('BLOCK_CONTRACT')
  np.linalg.cholesky(R);u=(t-query_ns)*1e-9/.05;X=np.c_[np.ones(len(t)),u,u*u];w=np.ones(len(t));coef=None
  for k in range(3):
   N=X.T@(w[:,None]*X);coef=np.linalg.solve(N,X.T@(w[:,None]*Y));E=Y-X@coef;norm=np.sqrt(np.einsum('ni,ij,nj->n',E,np.linalg.inv(R),E));w=np.minimum(1.,3./np.maximum(norm,1e-12))
  N=X.T@(w[:,None]*X);coef=np.linalg.solve(N,X.T@(w[:,None]*Y));weights=np.linalg.solve(N,np.array([1.,0.,0.]))@X.T*w;E=Y-X@coef;sample=E.T@E/max(1,len(t)-3);val,V=np.linalg.eigh((sample-R+sample.T-R.T)/2);upper=R+(V*np.maximum(val,0))@V.T;out=np.eye(2)*common+float(weights@weights)*upper;out=(out+out.T)/2;np.linalg.cholesky(out)
  return coef[0],out,dict(raw_samples=len(t),fit_noise_leverage=float(weights@weights),downweighted=int(sum(w<.999)),R_sample=sample.tolist(),independence_qualified=False)
