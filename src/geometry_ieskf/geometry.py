from .state import *
from scipy.spatial import cKDTree
import time
class GeometryFactor:
 """Reuse Phase U six-DOF residual/J, embed IMU-origin full-state error."""
 def __init__(self,lever):self.lever=np.array(lever)
 def transform(self,x):
  T=np.eye(4);T[:3,:3]=x.R;T[:3,3]=x.p-x.R@self.lever;return T
 def linearize(self,x,p,q,n):
  r,J=residual_jacobian(self.transform(x),p,q,n);H=np.zeros((len(p),15))
  H[:,:3]=J[:,:3]@x.R.T;H[:,6:9]=J[:,3:]+J[:,:3]@skew(self.lever)
  return r,H,J
class GeometryCorrespondence:
 def __init__(self,target,config=C):
  self.c=config;self.target=np.asarray(target);t=time.perf_counter();self.tree=cKDTree(self.target);self.build_ms=(time.perf_counter()-t)*1000
  self.normals=np.zeros_like(self.target);self.valid=np.zeros(len(target),bool);self.cached=np.zeros(len(target),bool)
  self.timings=[]
 def get(self,x,source,factor):
  begin=time.perf_counter();T=factor.transform(x);y=source@T[:3,:3].T+T[:3,3];distance,ids=self.tree.query(y,k=1);query_end=time.perf_counter()
  use=np.flatnonzero(distance<=self.c['correspondence_distance_m']);u=np.unique(ids[use]);missing=u[~self.cached[u]]
  if len(missing):
   k=min(self.c['normal_neighbors'],len(self.target));dd,ii=self.tree.query(self.target[missing],k=k)
   if k>=3:
    pts=self.target[ii];dev=pts-pts.mean(1,keepdims=True);ev,V=np.linalg.eigh(np.einsum('nki,nkj->nij',dev,dev)/k);n=V[:,:,0]
    self.normals[missing]=n;self.valid[missing]=(k==self.c['normal_neighbors'])&(dd[:,-1]<=self.c['normal_radius_m'])&(ev[:,0]/np.maximum(ev.sum(1),1e-20)<=self.c['normal_planarity_ratio'])&(ev[:,1]>=self.c['normal_second_eigenvalue_m2'])
   self.cached[missing]=True
  normal_end=time.perf_counter();use=use[self.valid[ids[use]]];tid=ids[use];p=source[use];q=self.target[tid];n=self.normals[tid]
  if len(use):
   _,inv,counts=np.unique(tid,return_inverse=True,return_counts=True);w=1./counts[inv]/self.c['lidar_sigma_m']**2
   blocks=len(np.unique(np.floor(p/self.c['source_block_m']).astype(int),axis=0))
  else:w=np.empty(0);blocks=0
  metrics={'source_n':len(source),'support_n':len(use),'unique_targets':len(np.unique(tid)),'source_blocks':blocks,'fraction':len(use)/max(1,len(source)),'new_normal_n':len(missing),'query_ms':(query_end-begin)*1000,'normal_ms':(normal_end-query_end)*1000,'correspondence_ms':(time.perf_counter()-begin)*1000}
  good=(len(use)>=self.c['minimum_support_points'] and metrics['unique_targets']>=self.c['minimum_unique_targets'] and blocks>=self.c['minimum_source_blocks'] and metrics['fraction']>=self.c['minimum_support_fraction'])
  return {'p':p,'q':q,'n':n,'w':w,'target_ids':tid,'good':good,'metrics':metrics}
class FixedCorrespondence:
 """Test-only fixed factors; never used by real replay."""
 def __init__(self,p,q,n,w):self.p=p;self.q=q;self.n=n;self.w=w
 def get(self,x,source,factor):return {'p':self.p,'q':self.q,'n':self.n,'w':self.w,'good':True,'metrics':{'support_n':len(self.p),'unique_targets':len(self.p),'source_n':len(self.p),'source_blocks':20,'fraction':1.}}
