"""Trace frozen motion equations with recorded NDT observations (NOT rematching).

Envelope replay uses original available/complete times; canonical replay retains
one final Header-order history. Both must agree with saved AR before attribution.
"""
import argparse,json,collections
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.state import State
from geometry_ieskf.common import C,sha
from geometry_ieskf.ndt_pose_observation import engineering_R
from geometry_ieskf.external_pose import ExternalPose
from ndt_global.ar_replay import AuditedPoseReplay
from ndt_global.validation import static_transform

def pack(x):return np.r_[x.p,x.v,x.R.ravel(),x.bg,x.ba]
def unpack(a,t):return State(a[:3].copy(),a[3:6].copy(),a[6:15].reshape(3,3).copy(),a[15:18].copy(),a[18:21].copy(),int(t))
def base(x,lever):return x.p-x.R@lever

class Recorder:
 def __init__(self):self.rows=[];self.context={};self.ingest_index=-1
 def add(self,stage,x,P,y,Q,**kw):
  r=dict(stage=stage,ingest_index=self.ingest_index,event_kind={'imu':0,'wheel':1,'clock':2,'ndt':3}[self.context['kind']],event_id=self.context.get('message_id',-1),header=self.context['header_ns'],t0=x.t,t1=y.t,before=pack(x),after=pack(y),P0=P.copy(),P1=Q.copy(),accepted=True,omega=np.zeros(3),force=np.zeros(3),dt=0.,H=np.zeros((6,15)),K=np.zeros((15,6)),innovation=np.zeros(6),delta=np.zeros(15),F=np.eye(15),measurement=np.zeros(6),Robs=np.zeros((6,6)))
  r.update(kw);self.rows.append(r)
 def save(self,path):
  np.savez_compressed(path,**{k:np.asarray([r[k] for r in self.rows]) for k in self.rows[0]})

class TracedObserver(ExternalPose):
 def __init__(self,lever,rec):super().__init__(lever,('x','y','z','rx','ry','rz'));self.rec=rec
 def correct(self,x,P,T,R,projection=None):
  y,Q,info=super().correct(x,P,T,R,projection);rr,H=self.residual(x,T);self.rec.add(2,x,P,y,Q,H=H,K=np.asarray(info.get('K',np.zeros((15,6)))),innovation=rr,delta=np.asarray(info.get('delta',np.zeros(15))),accepted=info['accepted'],Robs=np.asarray(R),measurement=np.r_[np.asarray(T)[:3,3],Rotation.from_matrix(np.asarray(T)[:3,:3]).as_rotvec()]);return y,Q,info

class TracedReplay(AuditedPoseReplay):
 def __init__(self,*args,rec,**kw):
  super().__init__(*args,**kw);self.rec=rec;predict=self.predictor.predict;wheel=self.wheel.correct
  def traced_predict(x,P,w,f,dt):
   y,Q=predict(x,P,w,f,dt);_,F,_,_=self.predictor.model(x,w,f,dt);rec.add(0,x,P,y,Q,omega=np.asarray(w),force=np.asarray(f),dt=dt,F=F);return y,Q
  def traced_wheel(x,P,w,z,var,age=0.):
   y,Q,info=wheel(x,P,w,z,var,age);_,h=self.wheel.model(x,w);S=float((h@P@h.T)[0,0]+var+self.wheel.c['wheel_slip_sigma_m_s']**2);H=np.zeros((6,15));H[:1]=h;K=np.zeros((15,6));K[:,:1]=P@h.T/S;R=np.zeros((6,6));R[0,0]=var+self.wheel.c['wheel_slip_sigma_m_s']**2;inn=np.zeros(6);inn[0]=info['innovation'];meas=np.zeros(6);meas[0]=z
   rec.add(1,x,P,y,Q,omega=np.asarray(w),H=H,K=K,innovation=inn,delta=np.asarray(info.get('delta',np.zeros(15))),accepted=info['accepted'],Robs=R,measurement=meas);return y,Q,info
  self.predictor.predict=traced_predict;self.wheel.correct=traced_wheel
 def step(self,checkpoint,e):self.rec.context=e;return super().step(checkpoint,e)

def reconstruct(a):
 a.private.mkdir(parents=True,exist_ok=True);data=json.loads(a.motion.read_text());tx=json.loads((a.run/'transactions.json').read_text());published=json.loads((a.run/'states.json').read_text());jobs=sorted(json.loads(a.bindings.read_text()),key=lambda j:j['native_reference_ns']);assert len(tx)==len(jobs);assert [r['ref'] for r in tx]==[j['native_reference_ns'] for j in jobs]
 lo=min(c['scan_start_ns'] for c in jobs[0]['clouds'])-100_000_000;ev=data['events'];init=max((e for e in ev if e['kind']=='pose' and e['header_ns']<=lo),key=lambda e:e['header_ns']);t0=init['header_ns'];im=max((e for e in ev if e['kind']=='imu' and e['header_ns']<=t0),key=lambda e:e['header_ns']);wh=max((e for e in ev if e['kind']=='wheel' and e['header_ns']<=t0),key=lambda e:e['header_ns']);Tbi=static_transform(data['transforms'],'base_link','imu_bosch');lever=Tbi[:3,3];Rb=Tbi[:3,:3];R=Rotation.from_quat(init['quaternion_xyzw']).as_matrix();omega=Rb@im['omega'];x0=State(np.array(init['position'])+R@lever,R@(np.array([wh['speed'],0,0])+np.cross(omega,lever)),R,np.zeros(3),np.zeros(3),t0);P0=np.diag(np.repeat(C['initial_std_p_v_theta_bg_ba'],3)**2);held=dict(header_ns=im['header_ns'],omega=omega,force=Rb@im['force']);sensor=[]
 for e in ev:
  if e['kind'] not in ('imu','wheel') or e['header_ns']<=t0:continue
  z=dict(e)
  if z['kind']=='imu':z['omega']=(Rb@e['omega']).tolist();z['force']=(Rb@e['force']).tolist()
  sensor.append(z)
 for i,j in enumerate(jobs):sensor.append(dict(kind='clock',header_ns=j['native_reference_ns'],message_id=-i-1))
 sensor.sort(key=AuditedPoseReplay.key);rec=Recorder();replay=TracedReplay(x0,P0,held,lever,rec=rec);obs=TracedObserver(lever,rec);cursor=0;now=t0;errors=collections.defaultdict(list);ingests=[];used=[];native=[]
 def advance(until):
  nonlocal cursor,now
  while cursor<len(sensor) and sensor[cursor]['header_ns']<=until:
   e=sensor[cursor];rec.ingest_index=len(ingests);result=replay.ingest(e);ingests.append(dict(index=rec.ingest_index,kind=e['kind'],header=e['header_ns'],scheduled_until=until,recorder=e.get('recorder_ns'),result=result));used.append(e);cursor+=1
  now=max(now,until)
 def event_for(r,i,observer):return dict(kind='ndt',header_ns=r['ref'],message_id=10_000_000+i,observer=observer,pose=np.array(r['ndt']['final']),R=np.array(r['update']['R']),projection=None,correlation_group=r['source_group'])
 for i,r in enumerate(tx):
  advance(max(now,r['available']));x,P,_=next(s for s in reversed(replay.snapshots) if s[0].t==r['ref']);errors['prior_p'].append(float(np.max(abs(x.p-r['prior']['p']))));errors['prior_R'].append(float(np.max(abs(x.R-r['prior']['R']))));errors['prior_P'].append(float(np.max(abs(P-r['prior']['P']))));seed=np.eye(4);seed[:3,:3]=x.R;seed[:3,3]=base(x,lever);errors['seed'].append(float(np.max(abs(seed-r['seed']))));legacy=engineering_R(r['ndt']['fitness']);expected=np.diag([legacy[0,0]]*3+[legacy[2,2]]*3);errors['R_rule'].append(float(np.max(abs(expected-r['update']['R']))))
  # Exact subinterval of original pre-ingest wall time is absent. Moving motion
  # advancement to final complete must pass full mean/P equivalence before use.
  advance(max(now,r['complete']));e=event_for(r,i,obs);rec.ingest_index=len(ingests);result=replay.ingest(e);ingests.append(dict(index=rec.ingest_index,kind='ndt',header=e['header_ns'],scheduled_until=now,result=result));u=e['last_update'];errors['delta'].append(float(np.max(abs(np.array(u['delta'])-r['update']['delta']))));errors['K'].append(float(np.max(abs(np.array(u['K'])-r['update']['K']))));errors['acceptance'].append(int(u['accepted']!=r['update']['accepted']));cur,PC,_=replay.current();saved=published[i];savedx=State(*(np.asarray(saved[k]) for k in ['p','v','R','bg','ba']),saved['time_ns']);errors['published_mean'].append(float(np.max(abs(pack(cur)-pack(savedx)))));errors['published_P'].append(float(np.max(abs(PC-saved['P']))));errors['published_time_ns'].append(abs(cur.t-savedx.t));native.append(dict(index=i,ref=r['ref'],prior=pack(x).tolist(),P=P.tolist()))
  if i%200==0:print('recovered',i+1,flush=True)
 rec.save(a.private/'envelope_trace.npz');(a.private/'ingests.json').write_text(json.dumps(ingests));(a.private/'native_full_priors.json').write_text(json.dumps(native));maxima={k:max(v,default=0) for k,v in errors.items()};qualified=all(v<= (0 if k in ['acceptance','published_time_ns'] else 1e-10) for k,v in maxima.items());summary=dict(frames=len(tx),fixed_observation_replay=True,CUDA_rematched=False,envelope_trace_rows=len(rec.rows),max_errors=maxima,equivalent=qualified,schedule='original available/complete envelope; pre-ingest advancement boundary not individually retained',new_sensor_latency_claim=False)
 a.output.write_text(json.dumps(summary,indent=2)+'\n')
 if not qualified:raise RuntimeError('EQUIVALENCE_GATE_FAILED')
 # Final canonical history: one application per information event, no duplicate
 # fusion of the posterior. Reprop revisions above are execution, not extra data.
 can=Recorder();cr=TracedReplay(x0,P0,held,lever,rec=can);co=TracedObserver(lever,can);events=used+[event_for(r,i,co) for i,r in enumerate(tx)];events.sort(key=cr.key);checkpoint=(x0,P0,held);checkpoints=[];description=[];canonical_errors=[]
 for i,e in enumerate(events):
  can.ingest_index=i;before=checkpoint;checkpoint=cr.step(checkpoint,e);xx,PP,ii=checkpoint;checkpoints.append(dict(before=pack(before[0]),after=pack(xx),P0=before[1].copy(),P1=PP.copy(),t0=before[0].t,t1=xx.t,imu_header0=before[2]['header_ns'],imu_omega0=before[2]['omega'],imu_force0=before[2]['force']));desc={k:v for k,v in e.items() if k not in ['observer','last_update']};description.append(desc)
  if e['kind']=='ndt':
   j=e['message_id']-10_000_000;canonical_errors.append(float(np.max(abs(pack(before[0])-np.asarray(native[j]['prior'])))))
 can.save(a.private/'canonical_trace.npz');np.savez_compressed(a.private/'canonical_checkpoints.npz',**{k:np.asarray([r[k] for r in checkpoints]) for k in checkpoints[0]});(a.private/'canonical_events.json').write_text(json.dumps(description,default=lambda x:x.tolist() if isinstance(x,np.ndarray) else x));np.savez(a.private/'initial.npz',state=pack(x0),P=P0,time=t0,imu_header=held['header_ns'],omega=held['omega'],force=held['force'],lever=lever);summary.update(canonical_trace_rows=len(can.rows),canonical_events=len(events),canonical_prior_max_error=max(canonical_errors,default=0));a.output.write_text(json.dumps(summary,indent=2)+'\n');assert summary['canonical_prior_max_error']<=1e-10
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for k in ['motion','bindings','run','private','output']:p.add_argument('--'+k,type=Path,required=True)
 reconstruct(p.parse_args())
