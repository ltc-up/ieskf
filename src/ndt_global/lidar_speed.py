"""Bounded scan-pair velocity-only front end and solver-seed auxiliary.
No local pose chain, velocityKalman update, global map, or covariance synthesis.
"""
import json,subprocess,time,hashlib,select
from collections import deque
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from geometry_ieskf.lidar_velocity import estimate_pair

class ScanVelocity:
 def __init__(self,matcher,output,enabled=True,stride=1,drop_interval=None,feedback_aware=False):
  if stride not in (1,2):raise ValueError('VELOCITY_RATE')
  self.log=Path(output).open('w');self.proc=subprocess.Popen([str(matcher)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=Path(output).with_suffix('.stderr').open('w'),text=True);self.past=deque(maxlen=2);self.hint=None;self.enabled=enabled;self.count=0;self.stride=stride;self.drop_interval=drop_interval;self.origin=None;self.failed=False;self.feedback_aware=feedback_aware
 @staticmethod
 def auxiliary(seed,ref,hint):
  T=np.asarray(seed,float).copy();info=dict(applied=False,reason='NO_FRESH_VELOCITY')
  if hint is None:return T,info
  age=(ref-hint['end_ns'])*1e-9
  if not 0<age<=.075:return T,info
  dv=np.asarray(hint['delta_world'])
  if not np.isfinite(dv).all() or np.linalg.norm(dv[:2])>5:return T,dict(applied=False,reason='VELOCITY_DIFFERENCE_DOMAIN')
  # A single next-seed velocity extrapolation, never integrated as local pose.
  delta=dv[:2]*age;T[:2,3]+=delta
  return T,dict(applied=True,reason='VELOCITY_SEED_AUXILIARY_ONLY',delta_xy=delta.tolist(),age_s=age)
 def seed(self,seed,ref,replay=None):
  hint=self.hint if self.enabled else None
  if hint is not None and self.feedback_aware:
   snaps=[s[0] for s in replay.snapshots if s[0].t==hint['end_ns']]
   if not snaps:return np.asarray(seed).copy(),dict(applied=False,reason='POST_FEEDBACK_HISTORY_MISSING')
   hint=dict(hint);hint['delta_world']=(np.asarray(hint['delta_world'])+np.asarray(hint['predicted_end_v'])-snaps[-1].v).tolist()
  return self.auxiliary(seed,ref,hint)
 @staticmethod
 def midpoint_state(replay,t):
  snaps=[replay.base,*replay.snapshots];st=np.array([s[0].t for s in snaps],dtype=np.int64);j=np.searchsorted(st,t)
  if j==0 or j==len(st):raise ValueError('PAIR_MIDPOINT_HISTORY')
  a,b=snaps[j-1][0],snaps[j][0];u=(t-a.t)/(b.t-a.t);v=(1-u)*a.v+u*b.v
  R=Slerp([0.,1.],Rotation.from_matrix([a.R,b.R]))([u]).as_matrix()[0];bg=(1-u)*a.bg+u*b.bg
  return v,R,bg
 def measure(self,path,ref,seed,replay,lever):
  tick=time.perf_counter_ns();out=dict(index=self.count,end_ns=ref,pairs=[],qualified=False,usage='VELOCITY_DIAGNOSTIC_ONLY');self.count+=1
  if self.origin is None:self.origin=ref
  elapsed=(ref-self.origin)*1e-9;drop=self.drop_interval is not None and self.drop_interval[0]<=elapsed<self.drop_interval[1]
  scheduled=(self.count-1)%self.stride==0
  if drop or self.failed:self.hint=None
  if not scheduled or drop or self.failed:
   out['usage']='INJECTED_VELOCITY_UNAVAILABLE' if drop else 'SERVICE_UNAVAILABLE' if self.failed else 'SCHEDULED_NO_NEW_VELOCITY'
   self.past.append(dict(ref=ref,path=str(path),seed=np.asarray(seed).copy(),identity=hashlib.sha256(str(path).encode()).hexdigest()));out['frontend_wall_ms']=(time.perf_counter_ns()-tick)*1e-6;self.log.write(json.dumps(out)+'\n');self.log.flush();return out
  for prev in reversed(self.past):
   dt=(ref-prev['ref'])*1e-9;relative=np.linalg.inv(prev['seed'])@seed;q=dict(case_id=f'BB_{ref}_{prev["ref"]}',source_path=str(path),target_path=str(prev['path']),target_sha=prev['identity'],seed_common=relative.tolist(),roi=[False,1,0,0,0,0,0,0,0,-10000,10000,120,0]);
   try:
    self.proc.stdin.write(json.dumps(q)+'\n');self.proc.stdin.flush()
    if not select.select([self.proc.stdout],[],[],.1)[0]:raise TimeoutError('PAIR_100MS_TIMEOUT')
    line=self.proc.stdout.readline()
    if not line:raise RuntimeError('SCAN_PAIR_SERVICE_TERMINATED')
    r=json.loads(line)
   except (OSError,RuntimeError,ValueError,TimeoutError) as err:
    self.failed=True;self.hint=None;out['service_failure']=str(err)
    if self.proc.poll() is None:self.proc.terminate()
    break
   r.update(dt=dt,start_ns=prev['ref'])
   try:v,g=estimate_pair(r,dt);r.update(twist=v.tolist(),geometry=g,quality='CONDITIONAL_LOCAL_VELOCITY')
   except (ValueError,KeyError,np.linalg.LinAlgError) as e:r['quality']=str(e)
   out['pairs'].append(r)
  good=[r for r in out['pairs'] if r['quality']=='CONDITIONAL_LOCAL_VELOCITY'];self.hint=None
  if len(good)==2:
   a,b=good;va,vb=np.array(a['twist']),np.array(b['twist']);limit=2*np.sqrt(2)*.05*np.sqrt(1/a['dt']**2+1/b['dt']**2);difference=float(np.linalg.norm(va[:2]-vb[:2]));out.update(cross_interval_difference_m_s=difference,consensus_envelope_m_s=limit)
   if difference<=limit:
    # Longer interval has lower differencing amplification; not a statistical R.
    r=max(good,key=lambda z:z['dt']);v=np.array(r['twist']);mid=(r['start_ns']+ref)//2
    try:
     vm,Rm,bg=self.midpoint_state(replay,mid);base_world=vm-Rm@np.cross(v[3:],lever);dv=Rm@v[:3]-base_world;ends=[s[0] for s in replay.snapshots if s[0].t==ref]
     if not ends:raise ValueError('PAIR_END_HISTORY')
     end=ends[-1];self.hint=dict(end_ns=ref,delta_world=dv.tolist(),predicted_end_v=end.v.tolist());out.update(qualified=True,midpoint_ns=mid,interval_twist=v.tolist(),delta_world=dv.tolist(),usage='NEXT_SEED_VELOCITY_AUXILIARY' if self.enabled else 'DIAGNOSTIC_ONLY')
    except ValueError as e:out['history_error']=str(e)
   else:out['usage']='CROSS_INTERVAL_INCONSISTENT'
  self.past.append(dict(ref=ref,path=str(path),seed=np.asarray(seed).copy(),identity=hashlib.sha256(str(path).encode()).hexdigest()));out['frontend_wall_ms']=(time.perf_counter_ns()-tick)*1e-6;self.log.write(json.dumps(out)+'\n');self.log.flush();return out
 def close(self):

  try:self.proc.stdin.close()
  except BrokenPipeError:pass
  self.proc.wait(timeout=30);self.log.close()
