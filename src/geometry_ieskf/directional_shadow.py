"""Coupled physical XY-yaw diagnostics; no inverse-Hessian statistical R."""
import numpy as np
from scipy.spatial.transform import Rotation

def ndt_information(hessian, euler, tangent=(1.,0.), length=10.):
    rx,ry,rz=np.asarray(euler).reshape(3)
    Rx=Rotation.from_euler('x',rx).as_matrix();Ry=Rotation.from_euler('y',ry).as_matrix()
    J=np.column_stack(([1,0,0],Rx@[0,1,0],Rx@Ry@[0,0,1]))
    t=np.asarray(tangent,float);t=t/np.linalg.norm(t);U=np.array([[t[0],-t[1]],[t[1],t[0]]])
    B=np.zeros((6,3));B[:2,:2]=U;B[3:,2]=np.linalg.solve(J,[0,0,1])/length
    H=np.asarray(hessian,float).reshape(6,6);H=-(H+H.T)*.5
    L=B.T@H@B;values,vectors=np.linalg.eigh(L)
    return dict(information=L.tolist(),eigenvalues=values.tolist(),weak=vectors[:,0].tolist(),lateral_weak_alignment=float(abs(vectors[1,0])),length_m=length,covariance=False)

def slip(x,omega,lever,force=None):
    v=x.R.T@x.v-np.cross(np.asarray(omega)-x.bg,lever)
    z=dict(body_velocity=v.tolist(),beta_rad=float(np.arctan2(v[1],v[0])),yaw_rate=float(omega[2]-x.bg[2]))
    if force is not None:
        a=np.asarray(force)-x.ba+x.R.T@np.array([0.,0.,-9.80665])
        z['ay_minus_rvx']=float(a[1]-(omega[2]-x.bg[2])*v[0])
    return z

def gicp_information(hessian, rotation, tangent=(1.,0.), length=10.):
    R=np.asarray(rotation,float);t=np.asarray(tangent,float);t=t/np.linalg.norm(t)
    U=np.array([[t[0],-t[1]],[t[1],t[0]]]);B=np.zeros((6,3))
    B[:3,2]=R.T@np.array([0.,0.,1.])/length;B[3:,:2]=R.T[:,:2]@U
    H=np.asarray(hessian,float).reshape(6,6);L=B.T@((H+H.T)*.5)@B;v,V=np.linalg.eigh(L)
    return dict(information=L.tolist(),eigenvalues=v.tolist(),weak=V[:,0].tolist(),length_m=length,covariance=False)
