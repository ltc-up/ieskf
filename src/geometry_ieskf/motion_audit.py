"""Independent numerical derivatives of the implemented right-error ZOH model."""
import numpy as np
from .state import State,inject,difference
from .inertial import InertialPredictor
from .wheel import WheelObservation

def derivative_audit(x,omega,force,dt,lever,eps=1e-6):
 m=InertialPredictor();y,F,G,Q=m.model(x,omega,force,dt);Fn=np.zeros_like(F);Gn=np.zeros_like(G)
 for j in range(15):
  d=np.eye(15)[j]*eps;a=m.model(inject(x,d),omega,force,dt)[0];b=m.model(inject(x,-d),omega,force,dt)[0];Fn[:,j]=(difference(a,y)-difference(b,y))/(2*eps)
 for j in range(6):
  d=np.eye(6)[j]*eps;a=m.model(x,omega+d[:3],force+d[3:],dt)[0];b=m.model(x,omega-d[:3],force-d[3:],dt)[0];Gn[:,j]=(difference(a,y)-difference(b,y))/(2*eps)
 Gn[9:12,6:9]=np.eye(3);Gn[12:15,9:12]=np.eye(3)
 w=WheelObservation(lever);_,H=w.model(x,omega);Hn=np.zeros_like(H)
 for j in range(15):
  d=np.eye(15)[j]*eps;Hn[0,j]=(w.model(inject(x,d),omega)[0]-w.model(inject(x,-d),omega)[0])/(2*eps)
 return dict(F_max_abs=float(np.max(abs(F-Fn))),G_max_abs=float(np.max(abs(G-Gn))),H_max_abs=float(np.max(abs(H-Hn))))

def gain_decomposition(x,P,omega,z,variance,lever):
 w=WheelObservation(lever);pred,H=w.model(x,omega);noise=variance+w.c['wheel_slip_sigma_m_s']**2;S=float((H@P@H.T)[0,0]+noise);K=P@H.T/S;r=z-pred;parts={name:float(P[2,s]@H[0,s]/S) for name,s in [('position',slice(0,3)),('velocity',slice(3,6)),('attitude',slice(6,9)),('gyro_bias',slice(9,12)),('accel_bias',slice(12,15))]}
 return dict(predicted=pred,innovation=r,S=S,K=K[:,0].tolist(),H=H[0].tolist(),Kz_by_block=parts,delta=(K[:,0]*r).tolist(),Pzz=float(P[2,2]),correlation_z_velocity=(P[2,3:6]/np.sqrt(P[2,2]*np.diag(P)[3:6])).tolist())

def sensor_velocity_model(x,omega,lever_imu,lever_sensor,axis_body):
 """One actual sensor axis; r is base-to-sensor minus base-to-IMU."""
 n=np.asarray(axis_body);r=np.asarray(lever_sensor)-np.asarray(lever_imu);vb=x.R.T@x.v
 from .common import skew
 pred=float(n@(vb+np.cross(omega-x.bg,r)));H=np.zeros((1,15));H[0,3:6]=n@x.R.T;H[0,6:9]=n@skew(vb);H[0,9:12]=n@skew(r)
 return pred,H
