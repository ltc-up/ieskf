from .state import *
class InertialPredictor:
 """ZOH gyro/force midpoint orientation; analytic exact discrete nominal Jacobians."""
 def __init__(self,config=C):self.c=config
 def model(self,x,omega,force,dt):
  if not 0<dt<=self.c['max_imu_interval_s']:raise ValueError('IMU_INTERVAL')
  if not np.isfinite(np.r_[omega,force]).all():raise ValueError('IMU_NONFINITE')
  w=omega-x.bg;f=force-x.ba;E=exp(w*dt);Eh=exp(w*dt/2);Rm=x.R@Eh
  acceleration=Rm@f+np.array(self.c['gravity_map_m_s2'])
  y=x.copy();y.p+=x.v*dt+.5*acceleration*dt**2;y.v+=acceleration*dt;y.R=x.R@E
  # SO3 operations remain on the manifold. No Euler integration.
  y.R=Rotation.from_matrix(y.R).as_matrix();y.t=x.t+round(dt*1e9)
  Atheta=-Rm@skew(f)@Eh.T;Abg=Rm@skew(f)@jr(w*dt/2)*dt/2
  F=np.eye(15);F[:3,3:6]=np.eye(3)*dt
  F[:3,6:9]=.5*dt**2*Atheta;F[3:6,6:9]=dt*Atheta
  F[:3,9:12]=.5*dt**2*Abg;F[3:6,9:12]=dt*Abg
  F[:3,12:15]=-.5*dt**2*Rm;F[3:6,12:15]=-dt*Rm
  F[6:9,6:9]=E.T;F[6:9,9:12]=-jr(w*dt)*dt
  G=np.zeros((15,12));G[:3,:3]=-.5*dt**2*Abg;G[3:6,:3]=-dt*Abg;G[6:9,:3]=jr(w*dt)*dt
  G[:3,3:6]=.5*dt**2*Rm;G[3:6,3:6]=dt*Rm
  G[9:12,6:9]=np.eye(3);G[12:15,9:12]=np.eye(3)
  c=self.c;Q=np.diag(np.repeat([c['gyro_noise_density_rad_s_sqrt_hz']**2/dt,c['accel_noise_density_m_s2_sqrt_hz']**2/dt,c['gyro_bias_rw_rad_s2_sqrt_hz']**2*dt,c['accel_bias_rw_m_s3_sqrt_hz']**2*dt],3))
  return y,F,G,Q
 def predict(self,x,P,omega,force,dt):
  y,F,G,Q=self.model(x,omega,force,dt);Pnew=F@P@F.T+G@Q@G.T
  return y,(Pnew+Pnew.T)/2
