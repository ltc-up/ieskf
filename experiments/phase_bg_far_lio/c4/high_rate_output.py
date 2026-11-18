"""Read-only atomic output snapshot. No prediction or observation update in reader."""
import copy,threading
import numpy as np
class SnapshotStore:
 def __init__(self):self._lock=threading.Lock();self._value=None;self.revision=0
 def commit(self,x,P,last_imu_ns,last_lidar_ns=None):
  Q=np.array(P,copy=True)
  if Q.shape!=(15,15):raise ValueError('INVALID_P_SHAPE')
  if not np.isfinite(np.r_[x.p,x.v,x.R.ravel(),x.bg,x.ba]).all():raise ValueError('INVALID_STATE')
  if int(last_imu_ns)>int(x.t):raise ValueError('IMU_AFTER_STATE')
  if not np.isfinite(Q).all() or not np.allclose(Q,Q.T,atol=1e-10) or np.linalg.eigvalsh(Q).min()<-1e-10:raise ValueError('INVALID_P')
  v=dict(state_time_ns=int(x.t),reference_point='IMU',position_map=x.p.tolist(),velocity_map=x.v.tolist(),rotation_map_body=x.R.tolist(),gyro_bias=x.bg.tolist(),accel_bias=x.ba.tolist(),P=Q.tolist(),P_chart='map p/v; body right SO3; body biases',last_imu_ns=int(last_imu_ns),last_lidar_ns=last_lidar_ns)
  with self._lock:
   if self._value is not None and int(x.t)<self._value['state_time_ns']:raise ValueError('BACKWARD_COMMIT')
   self.revision+=1;v['revision']=self.revision;self._value=v
 def packet(self,now_ns,scheduled_ns,seq,max_age_ns=20_000_000):
  with self._lock:v=copy.deepcopy(self._value)
  if v is None:return dict(seq=seq,valid=False,health='UNINITIALIZED',publish_time_ns=now_ns,scheduled_time_ns=scheduled_ns)
  age=now_ns-v['state_time_ns'];imu_age=now_ns-v['last_imu_ns'];valid=0<=age<=max_age_ns and 0<=imu_age<=max_age_ns
  return v|dict(seq=seq,publish_time_ns=now_ns,scheduled_time_ns=scheduled_ns,state_age_ns=age,imu_age_ns=imu_age,valid=valid,health='CURRENT' if valid else 'STALE')
