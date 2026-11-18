"""One original15D baseline, real recorder availability and bounded delayed replay.
Matching is asynchronous; IMU/wheel continue while the CUDA service works.
Only the current source/pose scratch slot is overwritten, never historical data.
"""
import argparse,json,sys,time,hashlib,copy
from pathlib import Path
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from scipy.spatial.transform import Rotation
from continuous_inputs import START,INIT,END,stream,Prefetch,AsyncLog
from clock_pump import ClockPump
REPO=Path(__file__).resolve().parents[3];sys.path.insert(0,str(REPO/'src'));sys.path.insert(0,str(REPO/'experiments/phase_be_e0_e4'))
from runtime import Service,read_cloud,write_cloud
from geometry_ieskf.state import State
from geometry_ieskf.common import C
from geometry_ieskf.external_pose import ExternalPose
from geometry_ieskf.ndt_pose_observation import engineering_R
from ndt_global.managed_replay import LegacyComparisonReplay

def pose(x,lever):
 T=np.eye(4);T[:3,:3]=x.R;T[:3,3]=x.p-x.R@lever;return T

def main(a):
 root=Path(a.root);out=root/'eskf15_recorded_init';out.mkdir(exist_ok=False);scratch=out/'active_scratch';scratch.mkdir()
 init=json.load(open(root/'recorded_common_init/initialization.json'));assert json.load(open(root/'realtime_contract/summary.json'))['reason']=='COMPLETED'
 audit=json.load(open(a.audit));Tbi=np.array(json.load(open(a.extrinsics))['T_base_imu']);lever=Tbi[:3,3];R=np.array(init['mean']['T_map_base'])[:3,:3];p=np.array(init['mean']['position']);paths=json.load(open(a.paths));ndt=Service(paths['ndt'],out/'ndt.log');desk=Service(paths['deskew'],out/'deskew.log',(scratch/'deskew_metadata.jsonl',scratch));assert desk.proc.stdout.readline().strip()=='READY';meta=(scratch/'deskew_metadata.jsonl').open();pool=ThreadPoolExecutor(1)
 # Same empty-source map warmup as originalBE, never admitted as an observation.
 empty=scratch/'empty.bin';write_cloud(empty,np.empty((0,3)));ndt.call(dict(case_id='C1_startup',header=INIT,group='startup',scene='startup',source_path=str(empty),target_path=paths['target'],target_sha=a.target_sha,roi=[False,1,0,0,0,0,0,0,0,-10000,10000,120,0],seed_common=np.eye(4).tolist(),q=50.,max_iterations=35,bd_config=dict(transformation_epsilon=.01,relative_objective_epsilon=.001,line_search=True,mode='gll')))
 feed=Prefetch(stream(a.bag,audit,Tbi));ilog=AsyncLog(out/'inputs.jsonl');states=AsyncLog(out/'states.jsonl');matches=AsyncLog(out/'matches.jsonl');clock=ClockPump(out/'clock.csv',START,END);epoch=clock.start();replay=None;observer=ExternalPose(lever,('x','y','z','rx','ry','rz'));lastimu=None;pending=deque();future=None;active=None;counts={};digest=hashlib.sha256();reason='UNKNOWN';index=0;late=[];firstP=None;waiting=None;end_seen=False;init_record=None
 def logstate(cause):
  x,P,_=replay.current();T=pose(x,lever);states.put(dict(stamp_ns=x.t,received_wall_ns=time.monotonic_ns(),sim_ns=START+time.monotonic_ns()-epoch,position=T[:3,3],quaternion=Rotation.from_matrix(T[:3,:3]).as_quat(),velocity_map_imu=x.v,bg=x.bg,ba=x.ba,P=P,cause=cause))
 def compute(e,prior,Q,hist,seq):
  begin=time.monotonic_ns();f=e['data'];clouds=[]
  # Validated time-preserving static fusion is regrouped by sensor_id, no new crop.
  # CUDA deskew sees base coordinates/identity extrinsics; the three original IDs remain in meta.
  for sid in range(3):
   z=f[f['sensor_id']==sid];path=scratch/f'active_sensor_{sid}.raw';path.write_bytes(z.tobytes());fields={k:dict(offset=z.dtype.fields[k][1],datatype=7,count=1) for k in ['x','y','z','timestamp','intensity']};clouds.append(dict(raw_path=str(path),fields=fields,width=len(z),height=1,row_step=len(z)*z.dtype.itemsize,point_step=z.dtype.itemsize,bigendian=False,header_ns=e['header'],T_body_lidar=np.eye(4).tolist()))
  rows=[]
  for x in hist:
   T=pose(x,lever);rows.append(np.r_[(x.t-e['header'])*1e-9,Rotation.from_matrix(T[:3,:3]).as_quat(),T[:3,3]])
  posepath=scratch/'active_poses.bin';np.asarray(rows,'<f8').tofile(posepath);seed=pose(prior,lever)
  req=dict(header=0,native_reference_ns=e['header'],scene='C1_15D',clouds=clouds,pose_path=str(posepath),T_odom_native=seed.tolist(),T_common_from_native=np.eye(4).tolist(),gates_pass=True)
  t=time.monotonic_ns();desk.proc.stdin.write(json.dumps(req)+'\n');desk.proc.stdin.flush();assert desk.proc.stdout.readline().strip()=='DONE';ds=json.loads(meta.readline());deskew_ms=(time.monotonic_ns()-t)/1e6
  # Ensure the existing service has finished its last source-file destructor.
  read_cloud(ds['source_path'])
  req=dict(case_id=f'C1_{seq}',header=e['header'],group=f'C1_{seq}',scene='C1',source_path=ds['source_path'],target_path=paths['target'],target_sha=a.target_sha,roi=[True,1,*seed[:3,3].tolist(),0,0,0,0,seed[2,3]-10,seed[2,3]+30,120,seq+1],seed_common=seed.tolist(),q=50.,max_iterations=35,bd_config=dict(transformation_epsilon=.01,relative_objective_epsilon=.001,line_search=True,mode='gll'))
  N,nt=ndt.call(req);return dict(ndt=N,deskew=ds,deskew_wall_ms=deskew_ms,ndt_wall_ms=nt,matcher_start_wall_ns=begin,matcher_end_wall_ns=time.monotonic_ns(),seed=seed.tolist(),prior=dict(p=prior.p.tolist(),v=prior.v.tolist(),R=prior.R.tolist(),bg=prior.bg.tolist(),ba=prior.ba.tolist(),P=Q.tolist()))
 try:
  while True:
   if future is not None and future.done():
    result=future.result();N=result['ndt'];update=dict(accepted=False,reason='MATCHER_INVALID');start=time.monotonic_ns()
    if N['converged'] and N.get('candidate_valid',True):
     try:old=engineering_R(N['fitness'])
     except ValueError as exc:update=dict(accepted=False,reason='OBSERVATION_COVARIANCE_UNAVAILABLE',detail=str(exc))
     else:
      Rv=np.diag([old[0,0]]*3+[old[2,2]]*3);event=dict(kind='ndt',header_ns=active['header'],message_id=10_000_000+index,observation_id=f'C1:{index}',correlation_group=f'C1:{index}',observer=observer,pose=np.asarray(N['final']),R=Rv);why=replay.ingest(event);update=event.get('last_update',dict(accepted=False,reason=why))
    replay.validate();finish=time.monotonic_ns();result.update(stamp_ns=active['header'],available_ns=active['available'],input_meta=active['meta'],update=update,complete_wall_ns=finish,complete_sim_ns=START+finish-epoch,delayed_update_ms=(finish-start)/1e6,after_available_ms=(finish-(epoch+active['available']-START))/1e6);matches.put(result);logstate('ndt_completion');future=None;active=None
   if waiting is None and not end_seen:
    waiting=feed.get()
    if waiting['kind']=='end':end_seen=True;tail=waiting;waiting=None
   sim=START+time.monotonic_ns()-epoch
   if waiting is not None and sim>=waiting['available']:
    e=waiting;waiting=None;tick=time.monotonic_ns();counts[e['kind']]=counts.get(e['kind'],0)+1
    ids=e['meta']['raw_identities'] if e['kind']=='cloud' else [dict(id=e['id'],cdr_sha256=e['sha'])];digest.update(json.dumps(ids,sort_keys=True).encode())
    if replay is None and e['available']>=INIT:
     if lastimu is None:raise RuntimeError('MISSING_CAUSAL_INITIAL_IMU')
     x=State(p+R@lever,R@np.cross(np.array(lastimu['omega']),lever),R.copy(),np.zeros(3),np.zeros(3),INIT);P=np.diag(np.repeat(C['initial_std_p_v_theta_bg_ba'],3)**2);replay=LegacyComparisonReplay(x,P,lastimu,lever);replay.validate();firstP=P.tolist();init_record=dict(t=INIT,p=x.p.tolist(),v=x.v.tolist(),R=x.R.tolist(),bg=x.bg.tolist(),ba=x.ba.tolist(),P=P.tolist(),source_mean_id=init['source']['message_id']);logstate('initial_state')
    if e['kind']=='imu':
     if replay is not None:replay.ingest(dict(kind='imu',header_ns=e['header'],message_id=e['id'],omega=e['omega'],force=e['force']))
     lastimu=dict(header_ns=e['header'],omega=np.array(e['omega']),force=np.array(e['force']))
    elif e['kind']=='wheel' and replay is not None:
     replay.ingest(dict(kind='wheel',header_ns=e['header'],message_id=e['id'],speed=e['speed'],variance=e['variance']));logstate('wheel')
    elif e['kind']=='cloud' and replay is not None:
     pending.append(e)
     if len(pending)>10:raise RuntimeError('MATCH_QUEUE_RESOURCE_LIMIT')
    finish=time.monotonic_ns();delay=(finish-(epoch+e['available']-START))/1e6;late.append(delay);ilog.put(dict(kind=e['kind'],header=e['header'],available=e['available'],identities=ids,delivered_wall_ns=finish,actual_available_sim_ns=START+finish-epoch,process_ms=(finish-tick)/1e6,lateness_ms=delay,meta=e.get('meta')))
    if delay>100:raise RuntimeError('SEVERE_INPUT_DELIVERY_DELAY')
   if replay is not None and future is None and pending:
    active=pending.popleft();index+=1;ref=active['header'];span=active['meta']['point_start_ns']
    if span<INIT:
     matches.put(dict(stamp_ns=ref,available_ns=active['available'],input_meta=active['meta'],update=dict(accepted=False,reason='SOURCE_SPANS_PRE_INITIALIZATION'),no_alignment=True));active=None;continue
    replay.ingest(dict(kind='clock',header_ns=ref,message_id=-index));allhist={x.t:x for x,_,_ in [replay.base]+replay.snapshots if span-20_000_000<=x.t<=ref};hist=sorted(allhist.values(),key=lambda x:x.t)
    if not hist or hist[-1].t!=ref or hist[0].t>span:raise RuntimeError('SOURCE_DESKEW_HISTORY_CONTRACT')
    snap=next((z for z in reversed(replay.snapshots) if z[0].t==ref),None)
    if snap is None:raise RuntimeError('PRIOR_TIMESTAMP_MISSING')
    future=pool.submit(compute,active,snap[0].copy(),snap[1].copy(),[x.copy() for x in hist],index)
   if end_seen and waiting is None and not pending and future is None:break
   if waiting is None or sim<waiting['available']:time.sleep(.0002)
  if digest.hexdigest()!=json.load(open(root/'timing_contract/summary.json'))['ordered_identity_sha256']:raise RuntimeError('INPUT_IDENTITY_MISMATCH')
  reason='COMPLETED'
 except Exception as ex:reason=repr(ex)
 finally:
  feed.close();pool.shutdown(wait=True);ndt.close();desk.close();meta.close();cs=clock.close();ilog.close();states.close();matches.close()
  (out/'initial_state.json').write_text(json.dumps(init_record,indent=2));(out/'run_status.json').write_text(json.dumps(dict(reason=reason,counts=counts,clock=cs,ordered_identity_sha256=digest.hexdigest(),prefetch_peak_bytes=feed.peak,maximum_delivery_delay_ms=max(late,default=0),alignments_started=index),indent=2));print(reason,counts)
 if reason!='COMPLETED':raise SystemExit(2)
if __name__=='__main__':
 p=argparse.ArgumentParser();[p.add_argument('--'+n,required=True) for n in ['root','bag','audit','extrinsics','paths','target-sha']];main(p.parse_args())
