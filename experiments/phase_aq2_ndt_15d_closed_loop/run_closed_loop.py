"""Raw-cloud -> own-state CUDA Deskew -> own-seed CUDA-NDT -> delayed 15D update.
Restricted binaries/raw geometry are external authorized inputs, never vendored.
"""
import argparse,json,subprocess,time,hashlib,collections,copy
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.state import State
from geometry_ieskf.common import C,sha
from geometry_ieskf.ndt_pose_observation import engineering_R
from geometry_ieskf.external_pose import ExternalPose
from ndt_global.pose_delay import PoseDelayReplay
from ndt_global.validation import static_transform,percentiles
from ndt_global.integrity import MapSupport

def pose(x,lever):
 T=np.eye(4);T[:3,:3]=x.R;T[:3,3]=x.p-x.R@lever;return T

def pose_covariance(ndt,components,prior):
 small=engineering_R(ndt['fitness']);xy=small[0,0];rr=small[2,2]
 return np.diag([xy if k in ('x','y','z') else rr for k in components])

def main(a):
 a.private.mkdir(parents=True,exist_ok=True);data=json.loads(a.input.read_text());bindings=json.loads(a.bindings.read_text());jobs=sorted([j for j in bindings if j['window']==a.window],key=lambda j:j['native_reference_ns']);assert jobs
 if a.limit:jobs=jobs[:a.limit]
 lo=min(c['scan_start_ns'] for c in jobs[0]['clouds'])-100_000_000;ev=data['events'];init=max((e for e in ev if e['kind']=='pose' and e['header_ns']<=lo),key=lambda e:e['header_ns']);t0=init['header_ns'];im=max((e for e in ev if e['kind']=='imu' and e['header_ns']<=t0),key=lambda e:e['header_ns']);wh=max((e for e in ev if e['kind']=='wheel' and e['header_ns']<=t0),key=lambda e:e['header_ns']);Tbi=static_transform(data['transforms'],'base_link','imu_bosch');lever=Tbi[:3,3];Rb=Tbi[:3,:3];R=Rotation.from_quat(init['quaternion_xyzw']).as_matrix();omega=Rb@im['omega'];x=State(np.array(init['position'])+R@lever,R@(np.array([wh['speed'],0,0])+np.cross(omega,lever)),R,np.zeros(3),np.zeros(3),t0);P=np.diag(np.repeat(C['initial_std_p_v_theta_bg_ba'],3)**2);held=dict(header_ns=im['header_ns'],omega=omega,force=Rb@im['force']);replay=PoseDelayReplay(x,P,held,lever);components=('x','y','yaw') if a.mode=='B' else ('x','y','z','rx','ry','rz') if a.mode=='D' else ('x','y','z','yaw');obs=ExternalPose(lever,components);sensor=[]
 for e in ev:
  if e['kind'] not in ('imu','wheel') or e['header_ns']<=t0:continue
  z=dict(e)
  if z['kind']=='imu':z['omega']=Rb@e['omega'];z['force']=Rb@e['force']
  sensor.append(z)
 for i,j in enumerate(jobs):sensor.append(dict(kind='clock',header_ns=j['native_reference_ns'],message_id=-i-1))
 sensor.sort(key=PoseDelayReplay.key);cursor=0;now=t0;ledger=[];outputs=[];wall=[];dec=collections.Counter();wheel_max=0.;gaps=[];lastaccept=None;stop_reason=None;sourcegroups=set();targetsha=sha(a.target);support=None
 if a.mode=='D':support=MapSupport(np.fromfile(a.target,dtype='<f4',offset=8).reshape(-1,3))
 proc=None;deskproc=None;startup_ms=0.
 if a.mode!='A':proc=subprocess.Popen([str(a.matcher)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=(a.private/'matcher_stderr.log').open('w'),text=True)
 if a.mode!='A' and a.deskew_service:
  boot=time.perf_counter_ns();deskproc=subprocess.Popen([str(a.deskew_service),str(a.private/'deskew_meta_stream.jsonl'),str(a.private)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=(a.private/'deskew_service_stderr.log').open('w'),text=True);assert deskproc.stdout.readline().strip()=='READY'
  empty=a.private/'startup_empty.bin';np.array([0],'<u8').tofile(empty);warm=dict(case_id='INITIALIZE_MAP_ONLY',header=t0,group='NO_OBSERVATION',scene='STARTUP',source_path=str(empty),target_path=str(a.target),target_sha=targetsha,roi=[False,1,0,0,0,0,0,0,0,-10000,10000,120,0],seed_common=np.eye(4).tolist(),q=50.);proc.stdin.write(json.dumps(warm)+'\n');proc.stdin.flush();boot_result=json.loads(proc.stdout.readline());startup_ms=(time.perf_counter_ns()-boot)/1e6;(a.private/'startup_no_observation.json').write_text(json.dumps(boot_result))
 def advance(until):
  nonlocal cursor,now,wheel_max
  while cursor<len(sensor) and sensor[cursor]['header_ns']<=until:
   e=sensor[cursor];before=replay.current()[0];replay.ingest(e);after=replay.current()[0]
   if e['kind']=='wheel':wheel_max=max(wheel_max,abs(e.get('last_update',{}).get('delta',[0,0,0])[2]))
   cursor+=1
  now=max(now,until)
 def checkpoint(stamp):
  choices=[s for s in replay.snapshots if s[0].t<=stamp]
  if not choices:raise ValueError('SOURCE_HISTORY_MISSING')
  s=choices[-1];assert s[0].t==stamp;return s
 for i,job in enumerate(jobs):
  ref=job['native_reference_ns'];available=max(c['arrival_ns'] for c in job['clouds']);advance(max(now,available));prior,Q,_=checkpoint(ref);seed=pose(prior,lever);update=dict(accepted=False,reason='NO_NDT_CONTROL');ndt=None;geometry=None;latency=0.;job_now=now;group=hashlib.sha256(''.join(c['cdr_sha'] for c in job['clouds']).encode()).hexdigest();assert group not in sourcegroups;sourcegroups.add(group);start=time.perf_counter_ns()
  if a.mode!='A':
   for c in job['clouds']:assert sha(Path(c['raw_path']))==c['raw_sha'];assert c['scan_end_ns']<=ref
   spanlo=min(c['scan_start_ns'] for c in job['clouds']);history=[s[0] for s in replay.snapshots if spanlo-20_000_000<=s[0].t<=ref];assert len(history)>=2 and history[0].t<=spanlo
   rows=[]
   for s in history:
    T=pose(s,lever);rows.append(np.r_[(s.t-ref)*1e-9,Rotation.from_matrix(T[:3,:3]).as_quat(),T[:3,3]])
   poses=a.private/f'{i:04d}_poses.bin';np.asarray(rows,'<f8').tofile(poses);j=copy.deepcopy(job);j.update(header=ref,pose_path=str(poses),T_odom_native=seed.tolist(),T_common_from_native=np.eye(4).tolist(),gates_pass=True,scene='AQ_causal_'+a.mode);jp=a.private/'deskew_job.json';jp.write_text(json.dumps([j]));meta=a.private/'deskew_meta.jsonl'
   if deskproc:
    deskproc.stdin.write(json.dumps(j)+'\n');deskproc.stdin.flush();ack=deskproc.stdout.readline()
    if not ack:raise RuntimeError('DESKEW_SERVICE_TERMINATED')
    meta=a.private/'deskew_meta_stream.jsonl'
   else:
    with (a.private/'deskew_stdout.log').open('a') as log:subprocess.run([str(a.deskew),str(jp),str(meta),str(a.private)],check=True,stdout=log,stderr=log)
   ds=json.loads(meta.read_text().splitlines()[-1]);path=Path(ds['source_path']);roi=[True,1,*seed[:3,3].tolist(),0,0,0,0,float(seed[2,3]-10),float(seed[2,3]+30),120,i+1];request=dict(case_id=f'AQ_{a.mode}_{i}',header=ref,group=group,scene='AQ_normal_causal',source_path=str(path),target_path=str(a.target),target_sha=targetsha,roi=roi,seed_common=seed.tolist(),q=50.)
   proc.stdin.write(json.dumps(request)+'\n');proc.stdin.flush();line=proc.stdout.readline()
   if not line:raise RuntimeError('CUDA_MATCHER_TERMINATED')
   ndt=json.loads(line);T=np.array(ndt['final']);latency=(time.perf_counter_ns()-start)/1e6
   if a.mode!='E' and ndt['converged'] and ndt['candidate_valid']:
    try:
     Rv=pose_covariance(ndt,components,prior);projection=None
     if a.mode=='D':
      src=np.fromfile(path,dtype='<f4',offset=8).reshape(-1,3);geometry=support.diagnose(src,T)
      if geometry.get('rank',0)==0:raise ValueError('NO_GEOMETRIC_DIRECTIONS')
      basis=np.array(geometry['geometry']['basis']);B=np.zeros((6,6));B[:3,:3]=np.eye(3);B[3:,3:]=10*prior.R;projection=basis@B
     latency=(time.perf_counter_ns()-start)/1e6;advance(job_now+round(latency*1e6))
     event=dict(kind='ndt',header_ns=ref,message_id=10_000_000+i,observer=obs,pose=T,R=Rv,projection=projection,correlation_group=group);reason=replay.ingest(event);update=event.get('last_update',dict(accepted=False,reason=reason))
    except ValueError as e:update=dict(accepted=False,reason=str(e))
   elif a.mode=='E':update=dict(accepted=False,reason='MAP_ODOM_SHADOW_ONLY')
   else:update=dict(accepted=False,reason='NDT_NOT_CONVERGED_OR_INVALID')
   advance(job_now+round((time.perf_counter_ns()-start)))
   if a.mode=='E' and ndt['converged'] and ndt['candidate_valid']:update['shadow_correction']=(T@np.linalg.inv(seed)).tolist()
  if update['accepted']:
   if lastaccept is not None:gaps.append((ref-lastaccept)*1e-9)
   lastaccept=ref
  current,PC,_=replay.current();outputs.append(dict(time_ns=current.t,p=current.p.tolist(),v=current.v.tolist(),bg=current.bg.tolist(),ba=current.ba.tolist(),R=current.R.tolist(),P=PC.tolist()));dec[update['reason']]+=1;wall.append((time.perf_counter_ns()-start)/1e6);ledger.append(dict(index=i,ref=ref,available=available,complete=now,source_group=group,seed=seed.tolist(),prior=dict(p=prior.p.tolist(),R=prior.R.tolist(),P=Q.tolist()),ndt=ndt,geometry=geometry,update=update,source_sha256=sha(Path(ds['source_path'])) if ndt is not None else None));(a.private/'transactions.json').write_text(json.dumps(ledger));(a.private/'states.json').write_text(json.dumps(outputs));print(a.mode,i+1,update['reason'],flush=True)
  if a.stop_loss is not None and a.mode not in ('A','E') and (ref-(lastaccept if lastaccept is not None else t0))*1e-9>a.stop_loss:stop_reason='NDT_OBSERVATION_LOSS_CONTINUATION_GATE';break
 if proc:proc.stdin.close();proc.wait(timeout=30)
 if deskproc:deskproc.stdin.close();deskproc.wait(timeout=30)
 accepted=[r for r in ledger if r['update']['accepted']];z=[r['p'][2] for r in outputs];vz=[r['v'][2] for r in outputs];summary=dict(requested_frames=len(jobs),processed_frames=len(ledger),stop_reason=stop_reason,startup_ms=startup_ms,mode=a.mode,frames=len(ledger),requested_duration_s=(jobs[-1]['native_reference_ns']-jobs[0]['native_reference_ns'])*1e-9,duration_s=(ledger[-1]['ref']-ledger[0]['ref'])*1e-9,actual_raw_Deskew=a.mode!='A',actual_CUDA=a.mode!='A',own_causal_seed=True,delayed_15D_updates=len(accepted),component_acceptance={k:len(accepted) for k in components} if a.mode not in ('A','E') else {},decisions=dict(dec),Z_span_m=float(np.ptp(z)),vz_min_max=[min(vz),max(vz)],max_wheel_Z_state_change_m=wheel_max,max_ndt_Z_update_m=max((abs(r['update']['delta'][2]) for r in accepted),default=0),max_ndt_velocity_delta_m_s=max((float(np.linalg.norm(r['update']['delta'][3:6])) for r in accepted),default=0),final_bg_norm=float(np.linalg.norm(outputs[-1]['bg'])),final_ba_norm=float(np.linalg.norm(outputs[-1]['ba'])),final_sigmaZ_model_m=float(np.sqrt(outputs[-1]['P'][2][2])),max_accepted_gap_s=max(gaps,default=None),max_no_NDT_observation_s=max([r['ref'] for r in ledger][0]-t0, max((b-a for a,b in zip([t0]+[r['ref'] for r in accepted],[r['ref'] for r in accepted]+[ledger[-1]['ref']])),default=0))*1e-9,matching_converged=sum(r['ndt'] is not None and r['ndt']['converged'] for r in ledger),wall_ms=percentiles(wall),pose_statistical_covariance='UNQUALIFIED_HISTORICAL_ENGINEERING_RULE',independent_truth=False,GNSS_used=False,online_equivalence=False,transactions_sha256=sha(a.private/'transactions.json'),input_sha256=sha(a.input),target_sha256=targetsha,qualification='EXPLORATORY_CAUSAL_EXECUTION_ONLY')
 a.output.write_text(json.dumps(summary,indent=2)+'\n')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--bindings',type=Path,required=True);p.add_argument('--target',type=Path,required=True);p.add_argument('--deskew',type=Path,required=True);p.add_argument('--matcher',type=Path,required=True);p.add_argument('--private',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--mode',choices=list('ABCDE'),required=True);p.add_argument('--deskew-service',type=Path);p.add_argument('--window',default='normal');p.add_argument('--limit',type=int);p.add_argument('--stop-loss',type=float);main(p.parse_args())
