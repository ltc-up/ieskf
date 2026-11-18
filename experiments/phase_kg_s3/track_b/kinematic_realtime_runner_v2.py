"""Actual recorder-time KINEMATIC_BK PREVIOUS with one asynchronous GPU worker.

Main consumes real IMU/Wheel and A0 delayed observations. Worker alone owns all
Deskew/global/local services and local history. Publisher integrates released
raw motion from immutable committed local Pose, never a saved trajectory.
"""
import argparse,copy,hashlib,json,os,sys,time,shutil,threading
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation
from kinematic_causal_runner import base_pose,at_reference,state_record
from kinematic_realtime_support_v2 import MotionStore,PoseStore
from kinematic_realtime_output_v2 import KinematicOutput
from kinematic_bridge import original_legacy_initial_O0
from kinematic_input_audit import build_header_index,verify_frame_binding
from bridge import ShadowGroup
import continuous_inputs as ci
from runtime import Service,read_cloud,write_cloud
from geometry_ieskf.state import State
from geometry_ieskf.common import C
from geometry_ieskf.external_pose import ExternalPose
from geometry_ieskf.ndt_pose_observation import engineering_R
from ndt_global.managed_replay import LegacyComparisonReplay
from resource_monitor import ResourceMonitor
from clock_pump import ClockPump


def run(config):
    cfg=json.loads(Path(config).read_text())
    if os.environ.get('ROS_DOMAIN_ID')!='173' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise ValueError('ROS_ISOLATION_REQUIRED')
    if cfg.get('mode')!='A0' or cfg.get('strategy_spec'):raise ValueError('UNCHANGED_A0_SOURCE_PARENT_REQUIRED')
    if cfg.get('kinematic_targets')!=['PREVIOUS']:raise ValueError('ISOLATED_PREVIOUS_ONLY')
    sys.path.insert(0,cfg['shared_module_path']);from provenance import require_run_output,digest
    out=Path(cfg['output']);require_run_output(out)
    branch_root=Path(cfg.get('b_branch_resource_root',out.parent)).resolve()
    if not out.resolve().is_relative_to(branch_root):raise ValueError('OUTPUT_OUTSIDE_B_BRANCH_ROOT')
    resource_limits=dict(cfg['resource']);resource_limits['disk_cap_bytes']=min(resource_limits['disk_cap_bytes'],2*1024**3)
    if sum(p.stat().st_size for p in branch_root.rglob('*') if p.is_file())>=resource_limits['disk_cap_bytes']:raise RuntimeError('B_BRANCH_DISK_PREFLIGHT_LIMIT')
    if digest(cfg['paths']['target'])!=cfg['target_sha']:raise ValueError('MAP_IDENTITY_MISMATCH')
    out.mkdir(exist_ok=False);scratch=out/'active_scratch';scratch.mkdir();(out/'config.frozen.json').write_text(json.dumps(cfg,indent=2))
    snapshot=out/'source_snapshot';snapshot.mkdir();identity={}
    for p in Path(__file__).parent.iterdir():
        if p.is_file() and p.suffix in ('.py','.json'):shutil.copy2(p,snapshot/p.name);identity[str(p)]=digest(p)
    for k in ('ndt','deskew'):identity[cfg['paths'][k]]=digest(cfg['paths'][k])
    identity[cfg['b_executable']]=digest(cfg['b_executable']);(out/'runtime_identity.json').write_text(json.dumps(identity,indent=2))
    ci.START=cfg['start_ns'];ci.INIT=cfg['init_ns'];ci.END=cfg['end_ns']
    audit=json.loads(Path(cfg['audit']).read_text());Tbi=np.asarray(json.loads(Path(cfg['extrinsics']).read_text())['T_base_imu']);lever=Tbi[:3,3]
    headeraudit,metadata=build_header_index(cfg['bag'],ci.START,ci.END);metadata['frame_binding']=verify_frame_binding(metadata,audit,Tbi)
    (out/'header_identity.private.json').write_text(json.dumps(metadata))
    init=json.loads(Path(cfg['init_source']).read_text());T0=np.asarray(init['mean']['T_map_base']);R0=T0[:3,:3];p0=T0[:3,3]
    ndt=Service(cfg['paths']['ndt'],out/'ndt.stderr');desk=Service(cfg['paths']['deskew'],out/'deskew.stderr',(scratch/'deskew_metadata.jsonl',scratch))
    if desk.proc.stdout.readline().strip()!='READY':raise RuntimeError('DESKEW_NOT_READY')
    meta=(scratch/'deskew_metadata.jsonl').open();empty=scratch/'empty.bin';write_cloud(empty,np.empty((0,3)))
    ndt.call(dict(case_id='KIN_RT_WARMUP',header=ci.INIT,group='startup',scene='startup',source_path=str(empty),target_path=cfg['paths']['target'],target_sha=cfg['target_sha'],roi=[False,1,0,0,0,0,0,0,0,-10000,10000,120,0],seed_common=np.eye(4).tolist(),q=50.,max_iterations=35,bd_config=dict(transformation_epsilon=.01,relative_objective_epsilon=.001,line_search=True,mode='gll')))
    local=ShadowGroup(cfg['b_executable'],scratch/'local',modes=('B0',));motion=MotionStore(ci.INIT,headeraudit);poses=PoseStore()
    frames=ci.AsyncLog(out/'frames.jsonl');inputs=ci.AsyncLog(out/'input_ledger.jsonl');resource=ResourceMonitor(branch_root,resource_limits,ci.AsyncLog(out/'resource.jsonl'))
    feed=ci.Prefetch(ci.stream(cfg['bag'],audit,Tbi));clock=ClockPump(out/'clock.csv',ci.START,ci.END);publisher=KinematicOutput(out,ci.START,ci.END,ci.AsyncLog,poses,motion);pool=ThreadPoolExecutor(1)
    # Worker-only local state; main never reads or updates these mutable fields.
    worker_state=dict(last_ref=ci.INIT,Ukin=np.eye(4),H=None)
    initial_axis_committed=threading.Event()
    def worker(e,prior,hist,U0,index,predictor,query_available):
        begin=time.monotonic_ns();ref=e['header'];interval=predictor.query(worker_state['last_ref'],ref,query_available,e['available'])
        if interval['status']!='VALID':return dict(status=interval['status'],interval=interval,ref_ns=ref)
        f=e['data'];clouds=[]
        for sid in range(3):
            z=f[f['sensor_id']==sid];p=scratch/f'sensor_{sid}.raw';p.write_bytes(z.tobytes());fields={k:dict(offset=z.dtype.fields[k][1],datatype=7,count=1) for k in ['x','y','z','timestamp','intensity']}
            clouds.append(dict(raw_path=str(p),fields=fields,width=len(z),height=1,row_step=len(z)*z.dtype.itemsize,point_step=z.dtype.itemsize,bigendian=False,header_ns=ref,T_body_lidar=np.eye(4).tolist()))
        rows=[]
        for h in hist:
            T=base_pose(h,lever);rows.append(np.r_[(h.t-ref)*1e-9,Rotation.from_matrix(T[:3,:3]).as_quat(),T[:3,3]])
        posepath=scratch/'poses.bin';np.asarray(rows,'<f8').tofile(posepath);seed=base_pose(prior,lever)
        desk_start=time.monotonic_ns();req=dict(header=0,native_reference_ns=ref,scene='KIN_RT',clouds=clouds,pose_path=str(posepath),T_odom_native=seed.tolist(),T_common_from_native=np.eye(4).tolist(),gates_pass=True)
        desk.proc.stdin.write(json.dumps(req)+'\n');desk.proc.stdin.flush()
        if desk.proc.stdout.readline().strip()!='DONE':raise RuntimeError('DESKEW_FAILED')
        ds=json.loads(meta.readline());source=read_cloud(ds['source_path']);dense=np.fromfile(ds['common_xyzi'],dtype='<f4').reshape(-1,4)[:,:3];desk_ms=(time.monotonic_ns()-desk_start)/1e6
        roi=[True,1,*seed[:3,3].tolist(),0,0,0,0,seed[2,3]-10,seed[2,3]+30,120,index]
        N,align_ms=ndt.call(dict(case_id=f'KIN_RT_{index}',header=ref,group=f'KIN_RT_{index}',scene='A0',source_path=ds['source_path'],target_path=cfg['paths']['target'],target_sha=cfg['target_sha'],roi=roi,seed_common=seed.tolist(),q=50.,max_iterations=35,bd_config=dict(transformation_epsilon=.01,relative_objective_epsilon=.001,line_search=True,mode='gll')))
        worker_state['Ukin']=worker_state['Ukin']@np.asarray(interval['delta_T'])
        if worker_state['H'] is None:worker_state['H']=original_legacy_initial_O0(U0)@np.linalg.inv(worker_state['Ukin'])
        dh=hashlib.sha256(np.asarray(rows,'<f8').tobytes()).hexdigest();matched=local.on_frame(frame_id=str(index),ref_ns=ref,available_ns=query_available,dense=dense,source=source,U_T_base=worker_state['H']@worker_state['Ukin'],deskew_history=dh)
        sha=hashlib.sha256(np.ascontiguousarray(source,dtype='<f4').tobytes()).hexdigest()
        if matched['source_identity']!=sha:raise ValueError('GLOBAL_LOCAL_SOURCE_HASH_MISMATCH')
        worker_state['last_ref']=ref
        return dict(status='COMMITTED',ref_ns=ref,original_available_ns=e['available'],query_available_ns=query_available,interval=interval,source_payload_sha256=sha,deskew_history_sha256=dh,seed=seed,ndt=N,local=matched,deskew_ms=desk_ms,align_ms=align_ms,worker_begin_ns=begin,worker_end_ns=time.monotonic_ns(),worker_ms=(time.monotonic_ns()-begin)/1e6)
    epoch=clock.start();publisher.start(epoch);replay=None;pure=None;lastimu=None;pending=deque();waiting=None;future=None;active=None;end=False;seq=0;reason='UNKNOWN';counts={};cycle=[];max_delay=0.;ordered=hashlib.sha256();last_retry_available=None;latest_release=ci.START
    try:
        while True:
            if resource.error:raise RuntimeError(resource.error)
            if publisher.errors:raise RuntimeError('POSE_OUTPUT_FAILURE:'+str(publisher.errors))
            now=ci.START+time.monotonic_ns()-epoch
            if future is not None and future.done():
                result=future.result();future=None
                if result['status']=='DELAY':
                    active['last_query_available']=result['interval']['query_available_ns'];frames.put(result|dict(frame_id=active['index'],execution='wallclock delayed, no fabricated motion'))
                elif result['status']!='COMMITTED':
                    frames.put(result);raise RuntimeError('KINEMATIC_TERMINAL:'+result['status'])
                else:
                    N=result['ndt'];update=dict(accepted=False,reason='MATCHER_INVALID')
                    if N['converged'] and N.get('candidate_valid',True):
                        try:R=engineering_R(N['fitness'])
                        except ValueError as exc:update=dict(accepted=False,reason=str(exc))
                        else:
                            event=dict(kind='ndt',header_ns=active['header'],message_id=10_000_000+active['index'],observation_id='KIN_RT:'+str(active['index']),correlation_group='KIN_RT:'+str(active['index']),observer=ExternalPose(lever,('x','y','z','rx','ry','rz')),pose=np.asarray(N['final']),R=np.diag([R[0,0]]*3+[R[2,2]]*3))
                            why=replay.ingest(event);update=event.get('last_update',dict(accepted=False,reason=why))
                    z=result['local']['variants']['B0'];poses.commit(result['ref_ns'],z['O_T_base'],active['index'],result['source_payload_sha256'],result['interval']['used_motion_ids']);motion.prune(result['ref_ns'])
                    finish=time.monotonic_ns();result.update(update=update,current=state_record(replay,lever),complete_sim_ns=ci.START+finish-epoch,after_available_ms=(finish-epoch-active['available']+ci.START)/1e6,full_cycle_ms=(finish-active['dispatch_wall'])/1e6,execution='actual-recorder-clock-single-worker',real_time_publication=True);result['dispatch_to_commit_ms']=result['full_cycle_ms'];result['full_cycle_including_prepare_ms']=(finish-active['prepare_begin_wall'])/1e6;result['full_cycle_ms']=result['full_cycle_including_prepare_ms'];result['prepare_ms']=active.get('prepare_total_ms',0.);initial_axis_committed.set();cycle.append(result['full_cycle_including_prepare_ms']);frames.put(result);active=None
            if waiting is None and not end:
                waiting=feed.get()
                if waiting['kind']=='end':tail=waiting;waiting=None;end=True
            if waiting is not None and now>=waiting['available']:
                e=waiting;waiting=None;latest_release=e['available'];counts[e['kind']]=counts.get(e['kind'],0)+1
                ids=e['meta']['raw_identities'] if e['kind']=='cloud' else [dict(id=e['id'],cdr_sha256=e['sha'])];ordered.update(json.dumps(ids,sort_keys=True).encode())
                if e['kind'] in ('imu','wheel'):motion.append(e)
                if replay is None and e['available']>=ci.INIT:
                    if lastimu is None:raise RuntimeError('MISSING_INITIAL_IMU')
                    x=State(p0+R0@lever,R0@np.cross(lastimu['omega'],lever),R0.copy(),np.zeros(3),np.zeros(3),ci.INIT);P=np.diag(np.repeat(C['initial_std_p_v_theta_bg_ba'],3)**2)
                    replay=LegacyComparisonReplay(x,P,lastimu,lever);pure=LegacyComparisonReplay(x,P,lastimu,lever)
                if e['kind']=='imu':
                    if replay is not None:
                        event=dict(kind='imu',header_ns=e['header'],message_id=e['id'],omega=e['omega'],force=e['force']);replay.ingest(copy.deepcopy(event));
                        if not initial_axis_committed.is_set():pure.ingest(event)
                    lastimu=dict(header_ns=e['header'],omega=np.asarray(e['omega']),force=np.asarray(e['force']))
                elif e['kind']=='wheel' and replay is not None:
                    event=dict(kind='wheel',header_ns=e['header'],message_id=e['id'],speed=e['speed'],variance=e['variance']);replay.ingest(copy.deepcopy(event))
                    if not initial_axis_committed.is_set():pure.ingest(event)
                elif e['kind']=='cloud' and replay is not None:
                    pending.append(e)
                    if len(pending)>10:raise RuntimeError('SCAN_QUEUE_LIMIT')
                if replay is None:motion.prune(ci.INIT)
                delay=(time.monotonic_ns()-epoch-e['available']+ci.START)/1e6;max_delay=max(max_delay,delay);inputs.put(dict(kind=e['kind'],header_ns=e['header'],available_ns=e['available'],identities=ids,delivery_lateness_ms=delay))
                if delay>100:raise RuntimeError('SEVERE_INPUT_DELIVERY_DELAY')
            if replay is not None and future is None:
                if active is None and pending:
                    prepare_begin=time.monotonic_ns();active=pending.popleft();active['prepare_begin_wall']=prepare_begin;seq+=1;active['index']=seq
                    if active['meta']['point_start_ns']<ci.INIT:frames.put(dict(ref_ns=active['header'],status='SKIPPED_PREINIT_SOURCE'));active=None
                if active is not None and latest_release>active.get('last_query_available',-1):
                    active.setdefault('prepare_begin_wall',time.monotonic_ns());prepare_start=time.monotonic_ns()
                    ref=active['header'];span=active['meta']['point_start_ns'];prior,_,_=at_reference(replay,ref,seq)
                    U0=None
                    if not initial_axis_committed.is_set():
                        u,_,_=at_reference(pure,ref,seq);U0=base_pose(u,lever)
                    hist=sorted({x.t:x for x,_,_ in [replay.base]+replay.snapshots if span-20_000_000<=x.t<=ref}.values(),key=lambda x:x.t)
                    if not hist or hist[0].t>span or hist[-1].t!=ref:raise RuntimeError('EXPIRED_OR_INVALID_DESKEW_HISTORY')
                    active.setdefault('dispatch_wall',time.monotonic_ns());predictor=motion.predictor();history_copy=[x.copy() for x in hist];prior_copy=prior.copy();active['prepare_total_ms']=active.get('prepare_total_ms',0.)+(time.monotonic_ns()-prepare_start)/1e6;future=pool.submit(worker,active,prior_copy,history_copy,U0,seq,predictor,latest_release)
            if end and waiting is None and not pending and future is None and active is None:break
            if end and active is not None and future is None:raise RuntimeError('DELAY_UNRESOLVED_AT_STREAM_END')
            time.sleep(.0002)
        if cfg.get('expected_ordered_identity_sha256') and ordered.hexdigest()!=cfg['expected_ordered_identity_sha256']:raise ValueError('INPUT_IDENTITY_MISMATCH')
        if any(tail['pending_tail'].values()):raise ValueError('UNPAIRED_TAIL')
        reason='COMPLETED'
    except Exception as exc:
        reason=repr(exc);frames.put(dict(status='WALLCLOCK_ENGINE_FAILURE',reason=reason,qualification='engineering/scheduling failure, not proof of physical input gap',active_ref_ns=None if active is None else active['header']))
    finally:
        feed.close();pool.shutdown(wait=True);ndt.close();desk.close();local.close();meta.close();cs=clock.close();hs=publisher.close(abort=reason!='COMPLETED');frames.close();inputs.close();rs=resource.close()
        changed=[p for p,sha in identity.items() if not Path(p).is_file() or digest(p)!=sha]
        if changed and reason=='COMPLETED':reason='RUNTIME_SOURCE_CHANGED'
        def stat(a):return dict(n=len(a),p50=float(np.median(a)),p95=float(np.percentile(a,95)),p99=float(np.percentile(a,99)),max=float(max(a)),over_50ms=sum(x>50 for x in a)) if a else {}
        status=dict(reason=reason,counts=counts,started_scans=seq,completed_local_scans=len(cycle),full_cycle_ms=stat(cycle),max_delivery_lateness_ms=max_delay,ordered_identity=ordered.hexdigest(),clock=cs,output100=hs,resource=rs,runtime_changed_files=changed,qualification='actual PREVIOUS v2 immutable motion cache, post-init pure disabled; full cycle includes preparation; no joint P',production_qualified=False)
        (out/'run_status.json').write_text(json.dumps(status,indent=2));print(json.dumps(status),flush=True)
    if reason!='COMPLETED':raise SystemExit(2)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('config');run(parser.parse_args().config)
