"""Independent SE(3) causal definitions, not copied production Deskew code."""
import numpy as np
from scipy.spatial.transform import Rotation

def compensate(points,R,p,Rref,pref,lever,mode):
 # All p are already base-origin world positions. Do NOT reapply IMU transport.
 # points already include static LiDAR extrinsics; full rotation retains that lever.
 y=np.einsum('nij,nj->ni',R,points)
 if mode=='C':
  yaw=np.arctan2(R[:,1,0],R[:,0,0]);yr=np.arctan2(Rref[1,0],Rref[0,0]);d=yaw-yr;c=np.cos(d);s=np.sin(d);w=points@Rref.T;y=np.c_[c*w[:,0]-s*w[:,1],s*w[:,0]+c*w[:,1],w[:,2]]
 w=y+p-pref
 if mode=='B':w[:,2]-=p[:,2]-pref[2]
 return w@Rref

def interpolate(time,poses):
 # offset seconds (not epoch float): time, quaternion xyzw, xyz.
 ix=np.searchsorted(poses[:,0],time,side='left')-1;ix=np.clip(ix,0,len(poses)-2);a=poses[ix];b=poses[ix+1];f=np.clip((time-a[:,0])/(b[:,0]-a[:,0]),0,1);qa=a[:,1:5];qb=b[:,1:5].copy();dot=np.sum(qa*qb,axis=1);qb[dot<0]*=-1;dot=abs(dot);small=dot>.9995;angle=np.arccos(np.clip(dot,-1,1));sn=np.sin(angle);wa=np.divide(np.sin((1-f)*angle),sn,out=1-f,where=~small);wb=np.divide(np.sin(f*angle),sn,out=f.copy(),where=~small);q=wa[:,None]*qa+wb[:,None]*qb;q/=np.linalg.norm(q,axis=1)[:,None];return Rotation.from_quat(q).as_matrix(),a[:,5:8]+f[:,None]*(b[:,5:8]-a[:,5:8])
