"""Scan-pair body twist and conditional integrity, never a calibrated pose/velocityR."""
import numpy as np
from scipy.spatial.transform import Rotation
from .common import skew

def body_twist(T_previous_current,dt):
 T=np.asarray(T_previous_current,float)
 if not np.isfinite(dt) or not .025<=dt<=.15:raise ValueError('PAIR_INTERVAL')
 if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6) or np.linalg.det(T[:3,:3])<.999:raise ValueError('PAIR_TRANSFORM')
 w=Rotation.from_matrix(T[:3,:3]).as_rotvec();theta=np.linalg.norm(w);W=skew(w)
 J=np.eye(3)+(.5 if theta<1e-6 else (1-np.cos(theta))/theta**2)*W+(1/6 if theta<1e-6 else (theta-np.sin(theta))/theta**3)*W@W
 return np.r_[np.linalg.solve(J,T[:3,3]),w]/dt

def sensor_twist_velocity(twist,T_base_sensor):
 t=np.asarray(T_base_sensor,float);v=np.asarray(twist,float);return t[:3,:3].T@(v[:3]+np.cross(v[3:],t[:3,3]))

def geometry_quality(H):
 # small_gicp uses right [rotation,translation]; nondimensional z=[t/10m,r/rad].
 H=np.asarray(H,float);B=np.zeros((6,6));B[:3,3:]=np.eye(3);B[3:,:3]=np.eye(3)*10.;N=B.T@((H+H.T)/2)@B
 e=np.linalg.eigvalsh(N);tr=N[:3,:3]-N[:3,3:]@np.linalg.pinv(N[3:,3:],rcond=1e-10)@N[3:,:3];v,V=np.linalg.eigh((tr+tr.T)/2);ratio=max(0,float(v[0]))/max(float(v[-1]),1e-30)
 return dict(normalized_eigenvalues=e.tolist(),translation_schur_eigenvalues=v.tolist(),translation_schur_vectors=V.tolist(),translation_ratio=ratio,statistical_covariance=False)

def estimate_pair(result,dt):
 if not result.get('converged') or not result.get('candidate_valid'):raise ValueError('PAIR_NOT_CONVERGED')
 if result.get('pairs',0)<100 or result['pairs']/max(result.get('source_n',0),1)<.5:raise ValueError('PAIR_SUPPORT')
 v=body_twist(result['final'],dt)
 if np.linalg.norm(v[:3])>100 or np.linalg.norm(v[3:])>10:raise ValueError('PAIR_SPEED_DOMAIN')
 g=geometry_quality(result['H_right_rotation_translation'])
 if g['translation_ratio']<1e-4:raise ValueError('PAIR_WEAK_GEOMETRY')
 return v,g
