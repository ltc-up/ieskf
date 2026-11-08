"""Existing 15D dynamics on a whole preselected real stream. Full trace stays private.
No global update, no artificial timestamp substitution, no new kinematic constraints.
"""
import argparse,json,time,hashlib,collections
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.inertial import InertialPredictor
from geometry_ieskf.wheel import WheelObservation
from geometry_ieskf.state import State
from geometry_ieskf.common import C,CONFIG_PATH,sha
from ndt_global.validation import static_transform,closed_traverse,percentiles,timestamp_qualified

def run(data,private):
 events=data['events'];by={k:sorted([e for e in events if e['kind']==k],key=lambda e:e['header_ns']) for k in ['imu','wheel','pose']}
 base=dict(configuration_sha256=sha(CONFIG_PATH),configuration_calibrated=False,raw_summary=data['summary'],closed_traverse=closed_traverse(by['pose']),input_scope='whole selected recording; no post-run window selection',Kistler_fusion='NOT_QUALIFIED: no separately verified independent velocity channel model',map_feedback=False,online_equivalence=False)
 if base['closed_traverse'] is not None:base['closed_traverse']={k:v for k,v in base['closed_traverse'].items() if k not in ('begin_ns','end_ns')}
 if not timestamp_qualified(by['wheel']):return base|dict(status='NOT_QUALIFIED',failure='WHEEL_ZERO_OR_MISSING_HEADER',processed_s=0,outputs=0)
 if {e['frame'] for e in by['imu']}!={'imu_bosch'}:return base|dict(status='NOT_QUALIFIED',failure='IMU_FRAME_CONTRACT_MISMATCH',processed_s=0,outputs=0)
 try:T=static_transform(data['transforms'],'base_link','imu_bosch')
 except ValueError as e:return base|dict(status='NOT_QUALIFIED',failure=str(e),processed_s=0,outputs=0)
 lever=T[:3,3];Rb=T[:3,:3];imus=by['imu'];wheels=by['wheel'];it=np.array([e['header_ns'] for e in imus]);wt=np.array([e['header_ns'] for e in wheels]);init=None
 for e in by['pose']:
  ti=e['header_ns'];i=int(np.searchsorted(it,ti,side='right')-1);j=int(np.searchsorted(wt,ti,side='right')-1)
  if i>=0 and j>=0 and 0<=ti-it[i]<=C['max_imu_interval_s']*1e9 and 0<=ti-wt[j]<=C['wheel_max_age_s']*1e9:init=e;imu=imus[i];wh=wheels[j];break
 if init is None:return base|dict(status='NOT_QUALIFIED',failure='NO_CAUSAL_INITIALIZATION',processed_s=0,outputs=0)
 R=Rotation.from_quat(init['quaternion_xyzw']).as_matrix();last=dict(omega=Rb@imu['omega'],force=Rb@imu['force'],header_ns=imu['header_ns']);t0=init['header_ns'];end=max(it[-1],wt[-1]);x=State(np.array(init['position'])+R@lever,R@(np.array([wh['speed'],0,0])+np.cross(last['omega'],lever)),R,np.zeros(3),np.zeros(3),t0);P=np.diag(np.repeat(C['initial_std_p_v_theta_bg_ba'],3)**2)
 es=[e for e in events if e['kind'] in ('imu','wheel') and e['header_ns']>t0];es+=[dict(kind='output',header_ns=t) for t in range(t0+10_000_000,end+1,10_000_000)];priority={'imu':0,'wheel':1,'output':2};es.sort(key=lambda e:(e['header_ns'],priority[e['kind']],e.get('message_id',0)))
 predictor=InertialPredictor();wheel=WheelObservation(lever);trace=np.lib.format.open_memmap(private/'local_trace.npy',mode='w+',dtype='float64',shape=((end-t0)//10_000_000+1,242));n=0;reasons=collections.Counter();timings=collections.defaultdict(list);dZ=[];dvz=[];failure=None;seen=set();begin=time.perf_counter()
 for e in es:
  t=e['header_ns'];kind=e['kind'];key=(kind,t)
  if kind!='output' and key in seen:reasons['DUPLICATE']+=1;continue
  seen.add(key)
  if (t-last['header_ns'])*1e-9>C['max_imu_interval_s']:failure='IMU_GAP_STOP_SEGMENT';break
  dt=(t-x.t)*1e-9
  if dt>0:
   tick=time.perf_counter_ns();x,P=predictor.predict(x,P,last['omega'],last['force'],dt);x.t=t;timings['propagation_ms'].append((time.perf_counter_ns()-tick)/1e6)
  if kind=='imu':last=dict(omega=Rb@e['omega'],force=Rb@e['force'],header_ns=t);reasons['IMU']+=1
  elif kind=='wheel':
   tick=time.perf_counter_ns();x,P,info=wheel.correct(x,P,last['omega'],e['speed'],e['variance']);timings['wheel_ms'].append((time.perf_counter_ns()-tick)/1e6);reasons['wheel_'+info['reason']]+=1
   if info['accepted']:dZ.append(abs(info['delta'][2]));dvz.append(abs(info['delta'][5]))
  else:
   # timestamp + nominal (16) + full 225 covariance entries.
   trace[n]=np.r_[float(t),x.p,x.v,Rotation.from_matrix(x.R).as_quat(),x.bg,x.ba,P.ravel()];n+=1
   if not np.isfinite(trace[n-1]).all():failure='NONFINITE_STATE_OR_P';break
   if np.linalg.eigvalsh(P).min()<-1e-7:failure='P_NOT_PSD';break
 wall=time.perf_counter()-begin;trace.flush();np.save(private/'local_stamps_ns.npy',np.arange(t0+10_000_000,t0+(n+1)*10_000_000,10_000_000,dtype=np.int64));(private/'local_contract.json').write_text(json.dumps(dict(lever=lever.tolist(),initial_ns=t0))); np.save(private/'local_valid_rows.npy',np.array([n],dtype=np.int64));rows=np.asarray(trace[:n]);z=rows[:,3] if n else np.array([])
 base.update(status='COMPLETED_UNQUALIFIED' if failure is None else 'FAIL',failure=failure,processed_s=(x.t-t0)*1e-9,requested_s=(end-t0)*1e-9,outputs=n,counts=dict(reasons),wheel_max_abs_delta_Z_m=max(dZ,default=None),wheel_max_abs_delta_vz_m_s=max(dvz,default=None),Z_span_m=float(np.ptp(z)) if n else None,final_vz_m_s=float(x.v[2]),final_sigma_Z_model_m=float(np.sqrt(P[2,2])),final_bg_norm=float(np.linalg.norm(x.bg)),final_ba_norm=float(np.linalg.norm(x.ba)),timing={k:percentiles(v) for k,v in timings.items()},wall_s=wall,local_trace_sha256=sha(private/'local_trace.npy'),initialization='one historical fused pose; causal wheel velocity; zero bias means, uncalibrated P',local_P_retained=True)
 return base
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--private-output',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.private_output.mkdir(parents=True,exist_ok=True);d=json.loads(a.input.read_text());r=run(d,a.private_output);r['input_extraction_sha256']=sha(a.input);a.output.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k not in ('raw_summary','timing')},indent=2))
