"""Actual recorder-time A replay with asynchronous geometry and real 100Hz ROS IO.

Separate from frozen serial experiments. Does not replay saved posterior logs.
Geometry is a single GPU worker; IMU/Wheel event ingestion and the read-only ROS
publisher continue while that worker runs. Exact-prior commit/retry is explicit.
An unimplemented post-commit historical geometry rebuild fails honestly rather
than silently using stale geometry or doing GPU work on the sensor event thread.
"""
import argparse,copy,hashlib,json,sys,time,importlib.util,shutil,subprocess,os
from pathlib import Path
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from scipy.spatial.transform import Rotation
REPO=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(REPO/'src'),str(REPO/'experiments/phase_bg_far_lio/c1'),str(REPO/'experiments/phase_bg_far_lio/c5'),str(REPO/'experiments/phase_be_e0_e4')]
import continuous_inputs as ci
from runtime import Service,read_cloud,write_cloud
from high_rate_real import RealOutput
from resource_monitor import ResourceMonitor
from clock_pump import ClockPump
from geometry_ieskf.state import State
from geometry_ieskf.common import C,CONFIG_PATH
from ndt_global.managed_replay import ManagedReplay
from score_map import prior_identity
from strategy import factory,LegacyObserver
from realtime_support import PreparedGeometry,replay_prior,AttemptCost

def base_pose(x,lever):
 T=np.eye(4);T[:3,:3]=x.R;T[:3,3]=x.p-x.R@lever;return T

def record(replay,lever):
 x,P,imu=replay.current()
 return dict(t=x.t,p=x.p,v=x.v,R=x.R,bg=x.bg,ba=x.ba,P=P,T_map_base=base_pose(x,lever),held_imu_ns=imu['header_ns'])

def preflight_and_snapshot(cfg,out):
    shared=Path(cfg['shared_module_path']).resolve()
    spec=importlib.util.spec_from_file_location('kg_s3_rt_provenance',shared/'provenance.py')
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    volume=module.require_run_output(out)  # resolves symlinks + expected UUID/RW/reserve
    out.mkdir(exist_ok=False);snapshot=out/'source_snapshot';snapshot.mkdir()
    files={}
    for name,mod in list(sys.modules.items()):
        raw=getattr(mod,'__file__',None)
        if not raw:continue
        source=Path(raw).resolve()
        if source.suffix!='.py' or not source.is_file():continue
        if source.is_relative_to(REPO):relative=Path('research')/source.relative_to(REPO)
        elif source.is_relative_to(shared):relative=Path('shared')/source.relative_to(shared)
        else:continue
        destination=snapshot/relative;destination.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,destination)
        files[str(source)]=hashlib.sha256(source.read_bytes()).hexdigest()
    for key in ['ndt','deskew','score_service']:
        binary=Path(cfg['paths'][key]).resolve();files[str(binary)]=module.digest(binary)
        dynamic=subprocess.run(['ldd',str(binary)],capture_output=True,text=True,check=False)
        (snapshot/f'{key}.ldd.txt').write_text(dynamic.stdout+dynamic.stderr)
        for line in dynamic.stdout.splitlines():
            candidate=line.split('=>',1)[1].split(' (',1)[0].strip() if '=>' in line else ''
            if candidate.startswith('/') and Path(candidate).is_file():files[candidate]=module.digest(candidate)
    for key in ['audit','extrinsics','init_source']:
        source=Path(cfg[key]).resolve();files[str(source)]=module.digest(source)
    model_config=Path(CONFIG_PATH).resolve();files[str(model_config)]=module.digest(model_config)
    target=Path(cfg['paths']['target']).resolve();actual_target_sha=module.digest(target)
    if actual_target_sha!=cfg['target_sha']:raise ValueError('MAP_IDENTITY_MISMATCH')
    files[str(target)]=actual_target_sha
    record=dict(ros_environment={key:os.environ.get(key) for key in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION')},files=files,volume=volume,scope='loaded research Python source copied; binaries/dependencies/configuration hashed; private paths retained only in archive')
    (out/'runtime_identity.json').write_text(json.dumps(record,indent=2))
    return record

def run(path):
 cfg=json.loads(Path(path).read_text());out=Path(cfg['output'])
 if cfg.get('strategy') not in ('A1','A2'):raise ValueError('USE_UNCHANGED_C5_FOR_A0_REALTIME_BASELINE')
 identity=preflight_and_snapshot(cfg,out)
 scratch=out/'active_scratch';scratch.mkdir();slots=out/'source_slots';slots.mkdir()
 (out/'config.frozen.json').write_text(json.dumps(cfg,indent=2))
 ci.START=cfg['start_ns'];ci.INIT=cfg['init_ns'];ci.END=cfg['end_ns']
 audit=json.loads(Path(cfg['audit']).read_text());Tbi=np.asarray(json.loads(Path(cfg['extrinsics']).read_text())['T_base_imu']);lever=Tbi[:3,3]
 init=json.loads(Path(cfg['init_source']).read_text());T0=np.asarray(init['mean']['T_map_base']);p0=T0[:3,3];R0=T0[:3,:3];paths=cfg['paths']
 ndt=Service(paths['ndt'],out/'ndt.stderr');desk=Service(paths['deskew'],out/'deskew.stderr',(scratch/'deskew_metadata.jsonl',scratch))
 if desk.proc.stdout.readline().strip()!='READY':raise RuntimeError('DESKEW_NOT_READY')
 meta=(scratch/'deskew_metadata.jsonl').open();strategy=factory(cfg,paths,out,lever)
 empty=scratch/'empty.bin';write_cloud(empty,np.empty((0,3)))
 ndt.call(dict(case_id='KG_S3_RT_WARMUP',header=ci.INIT,group='startup',scene='startup',source_path=str(empty),target_path=paths['target'],target_sha=cfg['target_sha'],roi=[False,1,0,0,0,0,0,0,0,-10000,10000,120,0],seed_common=np.eye(4).tolist(),q=50.,max_iterations=35,bd_config=dict(transformation_epsilon=.01,relative_objective_epsilon=.001,line_search=True,mode='gll')))
 feed=ci.Prefetch(ci.stream(cfg['bag'],audit,Tbi));frames=ci.AsyncLog(out/'frames.jsonl');inputs=ci.AsyncLog(out/'input_ledger.jsonl');states=ci.AsyncLog(out/'states.jsonl')
 resource=ResourceMonitor(out,cfg['resource'],ci.AsyncLog(out/'resource.jsonl'));clock=ClockPump(out/'clock.csv',ci.START,ci.END)
 high=RealOutput(out,ci.START,ci.END,ci.AsyncLog);pool=ThreadPoolExecutor(1);epoch=clock.start();high.start(epoch)
 replay=None;lastimu=None;lastwheel=None;lastaccepted=None;lastattempt=None;counts={};ordered=hashlib.sha256();pending=deque();future=None;active=None;waiting=None;end_seen=False;seq=0;source_slots=[];reason='UNKNOWN';init_record=None;max_delivery=0;retries=0;accepted=0;cycle=[]
 def commit_snapshot():
  x,P,imu=replay.current();high.commit(x,P,imu['header_ns'],lastwheel,lastaccepted,lastattempt,replay.replayed)
 def ingest(event):
  result=replay.ingest(event);commit_snapshot();return result
 def worker(e,prior,P,hist,index,revision):
  begin=time.monotonic_ns();clouds=[];f=e['data'];ref=e['header'];ph=prior_identity(prior,P)
  for sid in range(3):
   z=f[f['sensor_id']==sid];filename=scratch/f'sensor_{sid}.raw';filename.write_bytes(z.tobytes())
   fields={k:dict(offset=z.dtype.fields[k][1],datatype=7,count=1) for k in ['x','y','z','timestamp','intensity']}
   clouds.append(dict(raw_path=str(filename),fields=fields,width=len(z),height=1,row_step=len(z)*z.dtype.itemsize,point_step=z.dtype.itemsize,bigendian=False,header_ns=ref,T_body_lidar=np.eye(4).tolist()))
  rows=[]
  for h in hist:
   T=base_pose(h,lever);rows.append(np.r_[(h.t-ref)*1e-9,Rotation.from_matrix(T[:3,:3]).as_quat(),T[:3,3]])
  posepath=scratch/'poses.bin';np.asarray(rows,'<f8').tofile(posepath);seed=base_pose(prior,lever)
  req=dict(header=0,native_reference_ns=ref,scene='KG_S3_RT',clouds=clouds,pose_path=str(posepath),T_odom_native=seed.tolist(),T_common_from_native=np.eye(4).tolist(),gates_pass=True)
  ds_start=time.monotonic_ns();desk.proc.stdin.write(json.dumps(req)+'\n');desk.proc.stdin.flush()
  if desk.proc.stdout.readline().strip()!='DONE':raise RuntimeError('DESKEW_FAILED')
  ds=json.loads(meta.readline());source=read_cloud(ds['source_path']);source_bytes=Path(ds['source_path']).read_bytes();sha=hashlib.sha256(source_bytes).hexdigest()
  source_path=slots/f'{ref}_v{revision}.bin';source_path.write_bytes(source_bytes)
  roi=[True,1,*seed[:3,3].tolist(),0,0,0,0,seed[2,3]-10,seed[2,3]+30,120,index]
  request=dict(case_id=f'KG_S3_RT_{index}',header=ref,group=f'KG_S3_RT_{index}',scene=cfg['strategy'],source_path=str(source_path),target_path=paths['target'],target_sha=cfg['target_sha'],roi=roi,seed_common=seed.tolist(),q=50.,max_iterations=35,bd_config=dict(transformation_epsilon=.01,relative_objective_epsilon=.001,line_search=True,mode='gll'))
  deskew_ms=(time.monotonic_ns()-ds_start)/1e6;N,align_ms=ndt.call(request)
  obs=strategy.create_observer(source_path,sha,cfg['target_sha'],roi,ph,N)
  Rpayload=N.get('fitness') if cfg['strategy']=='A0' else None
  geometry_start=time.monotonic_ns();y,Q,info=obs.correct(prior,P,np.asarray(N['final']),Rpayload)
  uid=f'KG_S3_RT:{index}';prepared=PreparedGeometry(uid,ph,y,Q,info)
  return dict(prepared=prepared,prior_hash=ph,source_path=source_path,source_sha256=sha,source_n=len(source),roi=roi,ndt=N,update=info,prior=dict(p=prior.p,v=prior.v,R=prior.R,bg=prior.bg,ba=prior.ba,P=P),seed=seed,deskew_history_sha256=hashlib.sha256(np.asarray(rows,'<f8').tobytes()).hexdigest(),deskew_ms=deskew_ms,align_ms=align_ms,geometry_ms=(time.monotonic_ns()-geometry_start)/1e6,worker_wall_ms=(time.monotonic_ns()-begin)/1e6,worker_begin_ns=begin,worker_end_ns=time.monotonic_ns(),revision=revision)
 def submit(e,index,revision):
  ref=e['header'];span=e['meta']['point_start_ns']
  prior,P,_=replay_prior(replay,ref,-index*1000-revision)
  hist=sorted({x.t:x for x,_,_ in [replay.base]+replay.snapshots if span-20_000_000<=x.t<=ref}.values(),key=lambda x:x.t)
  if not hist or hist[0].t>span or hist[-1].t!=ref:raise RuntimeError('SOURCE_DESKEW_HISTORY_CONTRACT')
  return pool.submit(worker,e,prior.copy(),P.copy(),[x.copy() for x in hist],index,revision)
 try:
  while True:
   if resource.error:raise RuntimeError(resource.error)
   if high.errors:raise RuntimeError('OUTPUT100_FAILURE:'+str(high.errors))
   if future is not None and future.done():
    result=future.result();lastattempt=active['header'];ref=active['header']
    cost=active.setdefault('attempt_cost',AttemptCost());cost.add(result);totals=cost.totals
    try:current_prior,currentP,_=replay_prior(replay,ref,-active['index']*1000-result['revision']-500)
    except ValueError as exc:
     failed={k:v for k,v in result.items() if k not in ('prepared','source_path')}
     frames.put(failed|dict(ref_ns=ref,status='FAILED_BEFORE_ADMISSION',reason=str(exc),all_attempt_totals=copy.deepcopy(totals)))
     raise
    if prior_identity(current_prior,currentP)!=result['prior_hash']:
     retry_log={k:v for k,v in result.items() if k not in ('prepared','source_path')}
     frames.put(retry_log|dict(ref_ns=ref,status='PRIOR_CHANGED_ASYNC_RETRY',prior_hash_actual=prior_identity(current_prior,currentP),attempt_totals=copy.deepcopy(totals)))
     result['source_path'].unlink();retries+=1
     if result['revision']>=cfg.get('max_prior_retries',3):raise RuntimeError('ASYNC_PRIOR_RETRY_LIMIT')
     future=submit(active,active['index'],result['revision']+1)
    else:
     start=time.monotonic_ns();event=dict(kind='ndt',header_ns=ref,message_id=10_000_000+active['index'],observation_id=f"KG_S3_RT:{active['index']}",correlation_group=f"KG_S3_RT:{active['index']}",observer=result.pop('prepared'),pose=np.asarray(result['ndt']['final']),R=None)
     why=ingest(event);info=event.get('last_update',dict(accepted=False,reason=why));accepted+=bool(info.get('accepted'))
     if info.get('accepted'):lastaccepted=ref
     commit_snapshot();finish=time.monotonic_ns();cycle.append(cost.elapsed_ms(finish))
     source_slots.append((ref,result['source_path']));result.pop('source_path');result.update(ref_ns=ref,available_ns=active['available'],raw_identities=active['meta']['raw_identities'],update=info,current=record(replay,lever),complete_wall_ns=finish,complete_sim_ns=ci.START+finish-epoch,after_available_ms=(finish-(epoch+active['available']-ci.START))/1e6,commit_ms=(finish-start)/1e6,full_cycle_ms=cycle[-1],full_cycle_from_dispatch_ms=(finish-active['first_dispatch_wall_ns'])/1e6,all_attempt_totals=copy.deepcopy(totals),execution='actual-recorder-clock-asynchronous-worker',real_time_publication=True)
     frames.put(result);states.put(record(replay,lever));future=None;active=None
     while source_slots and source_slots[0][0]<=replay.base[0].t:
      _,path=source_slots.pop(0);path.unlink(missing_ok=True)
   if waiting is None and not end_seen:
    waiting=feed.get()
    if waiting['kind']=='end':end_seen=True;tail=waiting;waiting=None
   sim=ci.START+time.monotonic_ns()-epoch
   if waiting is not None and sim>=waiting['available']:
    e=waiting;waiting=None;counts[e['kind']]=counts.get(e['kind'],0)+1
    ids=e['meta']['raw_identities'] if e['kind']=='cloud' else [dict(id=e['id'],cdr_sha256=e['sha'])];ordered.update(json.dumps(ids,sort_keys=True).encode())
    if replay is None and e['available']>=ci.INIT:
     if lastimu is None:raise RuntimeError('MISSING_CAUSAL_INITIAL_IMU')
     x=State(p0+R0@lever,R0@np.cross(lastimu['omega'],lever),R0.copy(),np.zeros(3),np.zeros(3),ci.INIT);P=np.diag(np.repeat(C['initial_std_p_v_theta_bg_ba'],3)**2)
     replay=ManagedReplay(x,P,lastimu,lever);init_record=record(replay,lever);commit_snapshot()
    if e['kind']=='imu':
     if replay is not None:ingest(dict(kind='imu',header_ns=e['header'],message_id=e['id'],omega=e['omega'],force=e['force']))
     lastimu=dict(header_ns=e['header'],omega=np.asarray(e['omega']),force=np.asarray(e['force']))
    elif e['kind']=='wheel' and replay is not None:
     lastwheel=e['header'];ingest(dict(kind='wheel',header_ns=e['header'],message_id=e['id'],speed=e['speed'],variance=e['variance']));states.put(record(replay,lever))
    elif e['kind']=='cloud' and replay is not None:
     pending.append(e)
     if len(pending)>10:raise RuntimeError('MATCH_QUEUE_RESOURCE_LIMIT')
    now=time.monotonic_ns();delay=(now-(epoch+e['available']-ci.START))/1e6;max_delivery=max(max_delivery,delay)
    inputs.put(dict(kind=e['kind'],header_ns=e['header'],available_ns=e['available'],identities=ids,lateness_ms=delay))
    if delay>100:raise RuntimeError('SEVERE_INPUT_DELIVERY_DELAY')
   if replay is not None and future is None and pending:
    active=pending.popleft();seq+=1;active['index']=seq
    if active['meta']['point_start_ns']<ci.INIT:
     frames.put(dict(ref_ns=active['header'],no_alignment=True,reason='SOURCE_SPANS_PRE_INITIALIZATION'));active=None
    else:
     active['first_dispatch_wall_ns']=time.monotonic_ns();future=submit(active,seq,0)
   if end_seen and waiting is None and not pending and future is None:break
   if waiting is None or sim<waiting['available']:time.sleep(.0002)
  if cfg.get('expected_ordered_identity_sha256') and ordered.hexdigest()!=cfg['expected_ordered_identity_sha256']:raise RuntimeError('INPUT_IDENTITY_MISMATCH')
  if any(tail['pending_tail'].values()):raise RuntimeError('UNPAIRED_TAIL')
  reason='COMPLETED'
 except Exception as exc:reason=repr(exc)
 finally:
  feed.close();pool.shutdown(wait=True)
  if reason!='COMPLETED' and future is not None:
   try:
    abandoned=future.result();safe={k:v for k,v in abandoned.items() if k not in ('prepared','source_path')}
    frames.put(safe|dict(ref_ns=active['header'] if active else None,status='ABORTED_PENDING_WORKER',reason=reason))
   except Exception as exc:frames.put(dict(status='ABORTED_WORKER_EXCEPTION',reason=repr(exc)))
  ndt.close();desk.close();strategy.close();meta.close();cs=clock.close();frames.close();inputs.close();states.close();hs=high.close(abort=reason!='COMPLETED');rs=resource.close()
  for _,filename in source_slots:filename.unlink(missing_ok=True)
  if init_record is not None:(out/'initial_state.json').write_text(json.dumps(init_record,default=lambda x:x.tolist() if isinstance(x,np.ndarray) else x))
  changed_files=[filename for filename,sha in identity['files'].items() if not Path(filename).is_file() or hashlib.sha256(Path(filename).read_bytes()).hexdigest()!=sha]
  if changed_files and reason=='COMPLETED':reason='SOURCE_MODULE_OR_BINARY_CHANGED_DURING_RUN'
  stat=lambda a:dict(n=len(a),p50=float(np.median(a)),p95=float(np.percentile(a,95)),p99=float(np.percentile(a,99)),max=float(max(a))) if a else {}
  status=dict(reason=reason,counts=counts,scans_started=seq,accepted=accepted,prior_retries=retries,ordered_input_identity=ordered.hexdigest(),maximum_delivery_delay_ms=max_delivery,full_cycle_ms=stat(cycle),clock=cs,output100=hs,resource=rs,execution='actual-recorder-clock-asynchronous-worker',live_100Hz_executed=True,production_qualified=False,post_commit_prior_change_policy='FAIL_EXPLICIT_ASYNC_REBUILD_UNIMPLEMENTED',runtime_identity_changed_files=changed_files)
  (out/'run_status.json').write_text(json.dumps(status,indent=2));print(json.dumps(status),flush=True)
 if reason!='COMPLETED':raise SystemExit(2)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('config');run(p.parse_args().config)
