"""Independent CPU equation-level VGICP diagnostics, never a state updater.
Matches the D2D/GN equations; NOT the original closed-source localization or
GLIM GPU implementation. Surface-validation and joint smoothing are not run.
"""
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

def hat(p):
 p=np.asarray(p);z=np.zeros(p.shape[:-1]);x,y,w=np.moveaxis(p,-1,0)
 return np.stack([z,-w,y,w,z,-x,-y,x,z],axis=-1).reshape(p.shape[:-1]+(3,3))
def transform(T,p):return np.asarray(p)@T[:3,:3].T+T[:3,3]
def covariances(p,k=20):
 if len(p)<k:raise ValueError('INSUFFICIENT_COVARIANCE_NEIGHBORS')
 _,ix=cKDTree(p).query(p,k=k,workers=1);q=p[ix];q=q-q.mean(1,keepdims=True);C=np.einsum('nki,nkj->nij',q,q)/k;_,V=np.linalg.eigh(C)
 return (V*np.array([.001,1.,1.])[None,None,:])@V.transpose(0,2,1)
class Voxels:
 def __init__(self,p,C,resolution=.5):
  self.resolution=resolution;coord=np.floor(p/resolution).astype(np.int64);keys,ix,count=np.unique(coord,axis=0,return_inverse=True,return_counts=True);self.keys={tuple(k):i for i,k in enumerate(keys)};self.mean=np.zeros((len(keys),3));self.cov=np.zeros((len(keys),3,3));np.add.at(self.mean,ix,p);np.add.at(self.cov,ix,C);self.mean/=count[:,None];self.cov/=count[:,None,None]
 def lookup(self,q):return np.array([self.keys.get(tuple(k),-1) for k in np.floor(q/self.resolution).astype(np.int64)])
 def overlap(self,q):return self.lookup(q)>=0

def coverage(p,mask):
 p=np.asarray(p);q=p[mask];angles=(np.arctan2(p[:,1],p[:,0])+np.pi)/(2*np.pi);rad=np.linalg.norm(p[:,:2],axis=1)
 return dict(n=len(p),matched=int(mask.sum()),fraction=float(mask.mean()),matched_bbox=None if len(q)==0 else [q.min(0).tolist(),q.max(0).tolist()],occupied_2m=int(len(np.unique(np.floor(q/2),axis=0))),sectors_24=int(len(np.unique(np.minimum((angles[mask]*24).astype(int),23)))),range_fraction=[float(mask[(rad>=lo)&(rad<hi)].mean()) if ((rad>=lo)&(rad<hi)).any() else None for lo,hi in [(0,20),(20,50),(50,np.inf)]])

def evaluate(source,C,vox,T):
 q=transform(T,source);idx=vox.lookup(q);mask=idx>=0;p=source[mask];a=q[mask];ids=idx[mask];res=vox.mean[ids]-a;W=np.linalg.inv(vox.cov[ids]+T[:3,:3]@C[mask]@T[:3,:3].T)
 Jt=np.concatenate([-hat(a),np.broadcast_to(np.eye(3),(len(p),3,3))],axis=2);Js=np.concatenate([T[:3,:3]@hat(p),np.broadcast_to(-T[:3,:3],(len(p),3,3))],axis=2);J=np.concatenate([Jt,Js],axis=2);H=np.einsum('nki,nkl,nlj->ij',J,W,J);g=np.einsum('nki,nkl,nl->i',J,W,res);cost=float(np.einsum('ni,nij,nj->',res,W,res));Hs=H[6:,6:]
 # Local source x/y/L*yaw, L=10m. Conditional target fixed; other pose components Schur-eliminated separately.
 D=np.zeros((6,3));D[3,0]=1;D[4,1]=1;D[2,2]=.1;proj=D.T@Hs@D;other=np.eye(6)[:,[0,1,5]];schur=proj-D.T@Hs@other@np.linalg.pinv(other.T@Hs@other)@other.T@Hs@D;lam,V=np.linalg.eigh((schur+schur.T)/2);ev=np.linalg.eigvalsh(proj);full=np.linalg.eigvalsh((H+H.T)/2)
 step=-np.linalg.pinv(Hs,rcond=1e-10)@g[6:]
 return dict(cost=cost,mean_cost=None if not len(p) else cost/len(p),coverage=coverage(source,mask),H_joint=H.tolist(),gradient_joint=g.tolist(),joint_eigenvalues=full.tolist(),source_conditional_XY_Lyaw_eigenvalues=ev.tolist(),source_nuisance_schur_XY_Lyaw_eigenvalues=lam.tolist(),weak_direction_XY_Lyaw=V[:,0].tolist(),source_local_GN_candidate=step.tolist(),GN_candidate_is_not_optimized_pose=True,independent_covariance=False)

def motion(t0,t1,imu,wheel,Rbi):
 """Causal wheel longitudinal + gyro body-kinematic diagnostic, not full15D.
 Zero lateral/normal velocity is an explicit unverified model assumption.
 Measurement applies only at recorder availability; no future sample interpolation.
 """
 if not t0<t1 or t1-t0>1_500_000_000:raise ValueError('MOTION_INTERVAL')
 I=[e for e in imu if e['header_ns']<=t1 and e['recorder_ns']<=t1];V=[e for e in wheel if e['header_ns']<=t1 and e['recorder_ns']<=t1]
 oldi=[e for e in I if max(e['header_ns'],e['recorder_ns'])<=t0];oldv=[e for e in V if max(e['header_ns'],e['recorder_ns'])<=t0]
 if not oldi or not oldv:raise ValueError('MISSING_CAUSAL_MOTION_PREFIX')
 w=Rbi@np.array(oldi[-1]['omega']);v=oldv[-1]['speed'];T=np.eye(4);last=t0;li=oldi[-1]['header_ns'];lv=oldv[-1]['header_ns'];events=sorted([(max(e['header_ns'],e['recorder_ns']),0,e) for e in I if t0<max(e['header_ns'],e['recorder_ns'])<=t1]+[(max(e['header_ns'],e['recorder_ns']),1,e) for e in V if t0<max(e['header_ns'],e['recorder_ns'])<=t1]+[(t1,2,None)])
 for t,k,e in events:
  if t-li>30_000_000 or t-lv>40_000_000:raise ValueError('CAUSAL_MOTION_INPUT_GAP')
  dt=(t-last)*1e-9;T[:3,3]+=T[:3,:3]@Rotation.from_rotvec(w*dt/2).as_matrix()@np.array([v*dt,0,0]);T[:3,:3]=T[:3,:3]@Rotation.from_rotvec(w*dt).as_matrix();last=t
  if k==0:w=Rbi@np.array(e['omega']);li=e['header_ns']
  if k==1:v=e['speed'];lv=e['header_ns']
 return T
