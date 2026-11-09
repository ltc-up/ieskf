"""Fixed-track engineering evaluation; no estimator feedback or pose fitting."""
import numpy as np
from scipy.spatial import cKDTree
class Polyline:
 def __init__(self,xy):
  p=np.asarray(xy,float)[:,:2]
  if np.linalg.norm(p[-1]-p[0])<1e-6:p=p[:-1]
  keep=np.r_[True,np.linalg.norm(np.diff(p,axis=0),axis=1)>1e-8];self.p=p[keep];self.v=np.roll(self.p,-1,axis=0)-self.p;self.l=np.linalg.norm(self.v,axis=1)
  if len(self.p)<3 or np.any(self.l<1e-8):raise ValueError('TRACK_SEGMENTS')
  self.s=np.r_[0,np.cumsum(self.l)];self.length=self.s[-1];self.tree=cKDTree(self.p)
 def project(self,q):
  q=np.atleast_2d(q)[:,:2];_,near=self.tree.query(q,k=min(8,len(self.p)));near=np.asarray(near).reshape(len(q),-1);ids=np.concatenate([near,(near-1)%len(self.p)],axis=1);ids=np.broadcast_to(np.arange(len(self.p)),(len(q),len(self.p))) if len(self.p)<=512 else ids;a=self.p[ids];v=self.v[ids];u=np.clip(np.einsum('nki,nki->nk',q[:,None,:]-a,v)/(self.l[ids]**2),0,1);foot=a+u[:,:,None]*v;ds=np.sum((q[:,None,:]-foot)**2,axis=2);k=np.argmin(ds,axis=1);i=ids[np.arange(len(q)),k];t=u[np.arange(len(q)),k];foot=foot[np.arange(len(q)),k];tangent=self.v[i]/self.l[i,None];d=q-foot
  return dict(s=(self.s[i]+t*self.l[i])%self.length,d=np.einsum('ni,ni->n',d,np.c_[-tangent[:,1],tangent[:,0]]),distance=np.sqrt(ds[np.arange(len(q)),k]),tangent=tangent,index=i,foot=foot)
def paired_errors(estimate,reference,tangent):
 e=np.asarray(estimate)[:,:2]-np.asarray(reference)[:,:2];t=np.asarray(tangent);return np.einsum('ni,ni->n',e,t),np.einsum('ni,ni->n',e,np.c_[-t[:,1],t[:,0]])
def interpolate_reference(t,values,valid,query,max_gap=.1):
 t=np.asarray(t,float);q=np.asarray(query,float);values=np.asarray(values);valid=np.asarray(valid,bool)
 if np.any(np.diff(t)<=0):raise ValueError('REFERENCE_TIME_ORDER')
 i=np.searchsorted(t,q,side='right')-1;i=np.clip(i,0,len(t)-2);gap=t[i+1]-t[i];u=(q-t[i])/gap;ok=(q>=t[0])&(q<=t[-1])&(gap<=max_gap)&valid[i]&valid[i+1];out=values[i]+u.reshape((-1,)+(1,)*(values.ndim-1))*(values[i+1]-values[i]);return out,ok

def offline_quality(*,converged,valid,support,rank,innovation_flag=False,multimode=False):
 if not converged or not valid or support<=0:return 'INVALID'
 if multimode:return 'AMBIGUOUS'
 if rank<6 or support<.8 or innovation_flag:return 'DEGRADED'
 return 'VALID' # complete listed diagnostics; does NOT certify correct peak
