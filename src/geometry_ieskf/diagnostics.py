from .state import *
import resource
class Diagnostics:
 def __init__(self):self.times={};self.events=[]
 def timing(self,name,ms):self.times.setdefault(name,[]).append(float(ms))
 def snapshot(self,x,P):
  ev=np.linalg.eigvalsh((P+P.T)/2)
  return {'state_time_ns':x.t,'p_imu_map':x.p.tolist(),'v_imu_map':x.v.tolist(),'q_map_body_xyzw':Rotation.from_matrix(x.R).as_quat().tolist(),'bg_body_rad_s':x.bg.tolist(),'ba_body_m_s2':x.ba.tolist(),'P':P.tolist(),'P_eigen_min':float(ev[0]),'P_eigen_max':float(ev[-1]),'P_symmetry_max':float(np.max(abs(P-P.T))),'finite':bool(np.isfinite(np.r_[x.p,x.v,x.R.ravel(),x.bg,x.ba,P.ravel()]).all())}
 def summary(self):
  return {'timings_ms':{k:dict(zip(['p50','p95','p99','max'],np.percentile(v,[50,95,99,100]).tolist()))|{'count':len(v),'total_ms':sum(v)} for k,v in self.times.items() if v},'peak_process_rss_kib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,'gpu':'not used','historical_callback_latency':'not recoverable'}
