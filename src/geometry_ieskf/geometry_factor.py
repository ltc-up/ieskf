"""Offline six-pose-DOF local factor. Not a full-state IESKF."""
import numpy as np
from scipy.linalg import cho_factor, cho_solve, expm

def skew(v):
 x,y,z=v
 return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])

def expse3(v):
 a=np.zeros((4,4));a[:3,:3]=skew(v[3:]);a[:3,3]=v[:3]
 return expm(a)

def adj(T):
 a=np.zeros((6,6));a[:3,:3]=T[:3,:3];a[3:,3:]=T[:3,:3];a[:3,3:]=skew(T[:3,3])@T[:3,:3]
 return a

def residual_jacobian(T,p,q,n):
 """T map_from_body; T_new=T Exp([rho_body,phi_body]); n in map."""
 y=p@T[:3,:3].T+T[:3,3]
 r=np.einsum('ni,ni->n',n,y-q)
 nb=n@T[:3,:3]
 return r,np.c_[nb,np.cross(p,nb)]

def weights(p,target_ids,mode,sigma=.2):
 if mode=='point': w=np.ones(len(p))
 elif mode=='block':
  _,ix,c=np.unique(np.floor(p/5).astype(int),axis=0,return_inverse=True,return_counts=True);w=1./c[ix]
 elif mode=='target':
  _,ix,c=np.unique(target_ids,return_inverse=True,return_counts=True);w=1./c[ix]
 else: raise ValueError(mode)
 return w/sigma**2

def update(P,J,r,w):
 """Whiten the prior, solve by Cholesky; never invert LiDAR information."""
 if not all(np.isfinite(a).all() for a in [P,J,r,w]) or np.any(w<0): raise ValueError('invalid factor')
 L=np.linalg.cholesky(P)
 # delta=L u, hence prior information is identity in u coordinates.
 A=J@L; H=np.eye(len(P))+A.T@(w[:,None]*A)
 b=-A.T@(w*r);u=cho_solve(cho_factor(H),b);delta=L@u
 Lam=J.T@(w[:,None]*J); bl=-J.T@(w*r)
 return delta,Lam,bl
