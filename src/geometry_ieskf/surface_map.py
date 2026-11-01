"""Deterministic shared-surface hypotheses. No calibrated map uncertainty implied."""
import itertools
import numpy as np
from scipy.spatial import cKDTree
from .common import C
from .surface_observation import Plane,tangent_basis
NEIGHBORS=list(itertools.product([-1,0,1],repeat=3))
def fit_tls(points,config=C):
 if len(points)<config['normal_neighbors']:return None,'INSUFFICIENT_UNIQUE_SUPPORT'
 center=points.mean(axis=0);d=points-center;w,V=np.linalg.eigh(d.T@d/len(points));n=V[:,0];n*=1 if n[np.argmax(abs(n))]>=0 else -1
 ratio=w[0]/max(w.sum(),1e-20)
 if ratio>config['normal_planarity_ratio'] or w[1]<config['normal_second_eigenvalue_m2']:return None,'NONPLANAR_OR_COLLINEAR'
 return dict(normal=n,fit_center=center,eigenvalues=w,planarity_ratio=ratio,scatter_rms=float(np.sqrt(max(w[0],0)))),None
class SurfaceMap:
 def __init__(self,target,query_points,side,plane_covariance,config=C):
  self.config=config;self.side=side;self.target=np.unique(np.asarray(target),axis=0);self.planes={};self.groups=[];self.patches=[];self.unresolved=[];self.invalid=[]
  keys=np.floor(self.target/side).astype(int);cells,inv=np.unique(keys,axis=0,return_inverse=True);order=np.argsort(inv,kind='stable');counts=np.bincount(inv);starts=np.r_[0,np.cumsum(counts)];lookup={tuple(k):i for i,k in enumerate(cells)}
  tree=cKDTree(self.target);hits=tree.query_ball_point(query_points,config['correspondence_distance_m']);hitids=np.unique(np.concatenate([np.asarray(a,dtype=int) for a in hits])) if any(len(a) for a in hits) else np.array([],int)
  selected=set()
  for k in np.unique(keys[hitids],axis=0):
   for off in NEIGHBORS:
    i=lookup.get(tuple(k+off))
    if i is not None:selected.add(i)
  for i in sorted(selected):
   ids=order[starts[i]:starts[i+1]];f,why=fit_tls(self.target[ids],config)
   if f is None:self.invalid.append(dict(cell=cells[i],reason=why));continue
   g=(cells[i]+.5)*side;n=f['normal'];a=g-n*(n@(g-f['fit_center']));self.patches.append(dict(cell=cells[i],target_ids=ids,grid_center=g,anchor=a,**f))
  parent=list(range(len(self.patches)));pmap={tuple(p['cell']):i for i,p in enumerate(self.patches)}
  def root(i):
   while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
   return i
  cosine=np.cos(np.arctan(np.sqrt(config['normal_planarity_ratio'])));tol=config['huber_delta_m']
  for i,p in enumerate(self.patches):
   for off in NEIGHBORS:
    j=pmap.get(tuple(p['cell']+off))
    if j is None or j<=i:continue
    q=self.patches[j];d=p['anchor']-q['anchor']
    if abs(p['normal']@q['normal'])>=cosine and max(abs(p['normal']@d),abs(q['normal']@d))<=tol:
     a,b=root(i),root(j);parent[max(a,b)]=min(a,b)
  for component in sorted(set(root(i) for i in parent)):
   members=[i for i in range(len(parent)) if root(i)==component];ids=np.unique(np.concatenate([self.patches[i]['target_ids'] for i in members]));f,why=fit_tls(self.target[ids],config)
   if f is not None:
    if any(abs(self.patches[i]['normal']@f['normal'])<cosine or abs(f['normal']@(self.patches[i]['fit_center']-f['fit_center']))>tol for i in members):why='COMPONENT_MEMBER_DISAGREEMENT'
   if why:self.unresolved.append(dict(patch_ids=members,reason=why));continue
   origin=np.mean([self.patches[i]['grid_center'] for i in members],axis=0);n=f['normal'];a=origin-n*(n@(origin-f['fit_center']));key='surface_'+str(len(self.groups));self.planes[key]=Plane(key,n,a,tangent_basis(n),np.asarray(plane_covariance).copy());self.groups.append(dict(key=key,target_ids=ids,patch_ids=members,anchor=a,**f))
  self.support_ids=np.unique(np.concatenate([g['target_ids'] for g in self.groups])) if self.groups else np.array([],int)
  self.target_group=np.full(len(self.target),-1,int)
  for i,g in enumerate(self.groups):self.target_group[g['target_ids']]=i
  self.tree=cKDTree(self.target[self.support_ids]) if len(self.support_ids) else None
  self.metrics=dict(original_records=len(target),unique_records=len(self.target),selected_cells=len(selected),valid_patches=len(self.patches),groups=len(self.groups),unresolved_components=len(self.unresolved),support_records=len(self.support_ids),side_m=side)
 def associate(self,x,source,factor):
  T=factor.transform(x);y=source@T[:3,:3].T+T[:3,3]
  if self.tree is None:d=np.full(len(source),np.inf);near=np.zeros(len(source),int)
  else:d,near=self.tree.query(y)
  si=np.flatnonzero(d<=self.config['correspondence_distance_m']);ti=self.support_ids[near[si]] if len(si) else np.array([],int);pi=np.array([self.groups[self.target_group[i]]['key'] for i in ti]);blocks=len(np.unique(np.floor(source[si]/self.config['source_block_m']),axis=0));unique=len(np.unique(ti));fraction=len(si)/max(len(source),1)
  good=len(si)>=self.config['minimum_support_points'] and unique>=self.config['minimum_unique_targets'] and blocks>=self.config['minimum_source_blocks'] and fraction>=self.config['minimum_support_fraction']
  return dict(source_indices=si,target_ids=ti,plane_ids=pi,distances=d[si],metrics=dict(source_support=len(si),unique_targets=unique,source_blocks=blocks,support_fraction=fraction,good=bool(good)))
