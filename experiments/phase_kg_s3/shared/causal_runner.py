"""Full raw-input, recorder-order, own-feedback research replay.

This is serial causal algorithm evaluation: actual computation time is measured
but is NOT advanced into the recorder clock. It is deliberately not evidence
of real-time scheduling or 100 Hz publication. The separate real-time harness
must qualify those. No reference reader or saved posterior is used online.
"""
import argparse, copy, hashlib, importlib, json, os, struct, sys, time, shutil, subprocess
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from scipy.spatial.transform import Rotation
REPO=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(REPO/'src'),str(REPO/'experiments/phase_bg_far_lio/c1'),str(REPO/'experiments/phase_bg_far_lio/c5'),str(REPO/'experiments/phase_be_e0_e4')]
from provenance import require_run_output, digest
import continuous_inputs as ci
from runtime import Service, read_cloud, write_cloud
from geometry_ieskf.state import State
from geometry_ieskf.common import C
from geometry_ieskf.external_pose import ExternalPose
from geometry_ieskf.ndt_pose_observation import engineering_R
from ndt_global.managed_replay import LegacyComparisonReplay, ManagedReplay
from resource_monitor import ResourceMonitor
from lifecycle_audit import LifecycleAudit


def base_pose(x,lever):
    T=np.eye(4);T[:3,:3]=x.R;T[:3,3]=x.p-x.R@lever;return T

def state_record(replay,lever):
    x,P,imu=replay.current()
    return dict(t=x.t,p=x.p,v=x.v,R=x.R,bg=x.bg,ba=x.ba,P=P,T_map_base=base_pose(x,lever),held_imu_ns=imu['header_ns'])

def at_reference(replay,ref,seq):
    replay.ingest(dict(kind='clock',header_ns=ref,message_id=-seq))
    return next(z for z in reversed(replay.snapshots) if z[0].t==ref)

def summary(values):
    a=np.asarray(values,float)
    return dict(n=len(a),p50=float(np.median(a)),p95=float(np.percentile(a,95)),p99=float(np.percentile(a,99)),max=float(a.max()),over_50ms=int((a>50).sum())) if len(a) else dict(n=0)


def run(config_file):
    cfg=json.loads(Path(config_file).read_text());out=Path(cfg['output']);require_run_output(out)
    out.mkdir(exist_ok=False);scratch=out/'active_scratch';scratch.mkdir();slots=out/'source_slots';slots.mkdir()
    (out/'config.frozen.json').write_text(json.dumps(cfg,indent=2))
    audit=json.loads(Path(cfg['audit']).read_text());Tbi=np.asarray(json.loads(Path(cfg['extrinsics']).read_text())['T_base_imu']);lever=Tbi[:3,3]
    init=json.loads(Path(cfg['init_source']).read_text());T0=np.asarray(init['mean']['T_map_base']);R0=T0[:3,:3];p0=T0[:3,3]
    paths=cfg['paths'];ci.START=cfg['start_ns'];ci.INIT=cfg['init_ns'];ci.END=cfg['end_ns']
    ndt=Service(paths['ndt'],out/'ndt.stderr');desk=Service(paths['deskew'],out/'deskew.stderr',(scratch/'deskew_metadata.jsonl',scratch))
    if desk.proc.stdout.readline().strip()!='READY':raise RuntimeError('DESKEW_NOT_READY')
    meta=(scratch/'deskew_metadata.jsonl').open()
    empty=scratch/'empty.bin';write_cloud(empty,np.empty((0,3)))
    # Map warmup is never admitted as LiDAR information.
    ndt.call(dict(case_id='KG_S3_WARMUP',header=ci.INIT,group='startup',scene='startup',source_path=str(empty),target_path=paths['target'],target_sha=cfg['target_sha'],roi=[False,1,0,0,0,0,0,0,0,-10000,10000,120,0],seed_common=np.eye(4).tolist(),q=50.,max_iterations=35,bd_config=dict(transformation_epsilon=.01,relative_objective_epsilon=.001,line_search=True,mode='gll')))
    strategy=None
    if cfg.get('strategy_spec'):
        sys.path.insert(0,cfg['strategy_module_path']);mod,fn=cfg['strategy_spec'].split(':');strategy=getattr(importlib.import_module(mod),fn)(cfg,paths,out,lever)
    bgroup=None
    if cfg.get('b_modes'):
        sys.path.insert(0,cfg['b_module_path']);from bridge import ShadowGroup
        bgroup=ShadowGroup(cfg['b_executable'],scratch/'local',tuple(cfg['b_modes']))
    # Snapshot actual implementation before raw input, including loaded plugins.
    snap=out/'source_snapshot';snap.mkdir()
    implementation={}
    roots=[Path(__file__).parent]
    if cfg.get('strategy_module_path'):roots.append(Path(cfg['strategy_module_path']))
    if cfg.get('b_module_path'):roots.append(Path(cfg['b_module_path']))
    for i,root in enumerate(roots):
        dest=snap/str(i);dest.mkdir()
        for file in sorted(root.iterdir()):
            if file.is_file() and file.suffix in ('.py','.json'):
                implementation[str(file)]=digest(file);shutil.copy2(file,dest/file.name)
    binaries={k:digest(v) for k,v in paths.items() if k in ('ndt','deskew','score_service')}
    if cfg.get('b_executable'):binaries['b_executable']=digest(cfg['b_executable'])
    (out/'implementation_identity.json').write_text(json.dumps(dict(files=implementation,binaries=binaries,config_sha256=digest(config_file)),indent=2))
    log=ci.AsyncLog(out/'frames.jsonl');inputs=ci.AsyncLog(out/'input_ledger.jsonl');resources=ResourceMonitor(out,cfg['resource'],ci.AsyncLog(out/'resource.jsonl'))
    lifecycle_log=ci.AsyncLog(out/'event_lifecycle.jsonl');lifecycle=LifecycleAudit(lifecycle_log)
    feed=ci.Prefetch(ci.stream(cfg['bag'],audit,Tbi));counts={};ordered=hashlib.sha256();lastimu=None;replay=None;pure=None;seq=0;cycles=[];source_hashes=[];source_slots=[];reason='UNKNOWN';wall_start=time.monotonic_ns();init_record=None;accepted=0;skips=0
    try:
        while True:
            if resources.error:raise RuntimeError(resources.error)
            e=feed.get()
            if e['kind']=='end':tail=e;break
            counts[e['kind']]=counts.get(e['kind'],0)+1
            ids=e['meta']['raw_identities'] if e['kind']=='cloud' else [dict(id=e['id'],cdr_sha256=e['sha'])]
            ordered.update(json.dumps(ids,sort_keys=True).encode())
            inputs.put(dict(kind=e['kind'],header_ns=e['header'],available_ns=e['available'],identities=ids))
            if replay is None and e['available']>=ci.INIT:
                if lastimu is None:raise RuntimeError('MISSING_CAUSAL_INITIAL_IMU')
                x=State(p0+R0@lever,R0@np.cross(lastimu['omega'],lever),R0.copy(),np.zeros(3),np.zeros(3),ci.INIT)
                P=np.diag(np.repeat(C['initial_std_p_v_theta_bg_ba'],3)**2)
                replay=(LegacyComparisonReplay if cfg['mode']=='A0' else ManagedReplay)(x,P,lastimu,lever)
                pure=LegacyComparisonReplay(x,P,lastimu,lever);init_record=state_record(replay,lever)
            if e['kind']=='imu':
                event=dict(kind='imu',header_ns=e['header'],message_id=e['id'],omega=e['omega'],force=e['force'])
                if replay is not None:replay.ingest(copy.deepcopy(event));pure.ingest(copy.deepcopy(event))
                lastimu=dict(header_ns=e['header'],omega=np.asarray(e['omega']),force=np.asarray(e['force']))
            elif e['kind']=='wheel' and replay is not None:
                event=dict(kind='wheel',header_ns=e['header'],message_id=e['id'],speed=e['speed'],variance=e['variance'])
                replay.ingest(copy.deepcopy(event));pure.ingest(copy.deepcopy(event))
            elif e['kind']=='cloud' and replay is not None:
                seq+=1;ref=e['header'];span=e['meta']['point_start_ns'];tick=time.monotonic_ns()
                if span<ci.INIT:
                    skips+=1;log.put(dict(ref_ns=ref,available_ns=e['available'],no_alignment=True,reason='SOURCE_SPANS_PRE_INITIALIZATION'));continue
                prior,Q,_=at_reference(replay,ref,seq);prior=prior.copy();Q=Q.copy();u,_,_=at_reference(pure,ref,seq)
                hist=sorted({x.t:x for x,_,_ in [replay.base]+replay.snapshots if span-20_000_000<=x.t<=ref}.values(),key=lambda x:x.t)
                if not hist or hist[0].t>span or hist[-1].t!=ref:raise RuntimeError('SOURCE_DESKEW_HISTORY_CONTRACT')
                f=e['data'];clouds=[]
                for sid in range(3):
                    z=f[f['sensor_id']==sid];path=scratch/f'sensor_{sid}.raw';path.write_bytes(z.tobytes())
                    fields={k:dict(offset=z.dtype.fields[k][1],datatype=7,count=1) for k in ['x','y','z','timestamp','intensity']}
                    clouds.append(dict(raw_path=str(path),fields=fields,width=len(z),height=1,row_step=len(z)*z.dtype.itemsize,point_step=z.dtype.itemsize,bigendian=False,header_ns=ref,T_body_lidar=np.eye(4).tolist()))
                rows=[]
                for h in hist:
                    T=base_pose(h,lever);rows.append(np.r_[(h.t-ref)*1e-9,Rotation.from_matrix(T[:3,:3]).as_quat(),T[:3,3]])
                posepath=scratch/'poses.bin';np.asarray(rows,'<f8').tofile(posepath);seed=base_pose(prior,lever)
                dh=hashlib.sha256(np.asarray(rows,'<f8').tobytes()).hexdigest()
                req=dict(header=0,native_reference_ns=ref,scene='KG_S3',clouds=clouds,pose_path=str(posepath),T_odom_native=seed.tolist(),T_common_from_native=np.eye(4).tolist(),gates_pass=True)
                dstart=time.monotonic_ns();desk.proc.stdin.write(json.dumps(req)+'\n');desk.proc.stdin.flush()
                if desk.proc.stdout.readline().strip()!='DONE':raise RuntimeError('DESKEW_FAILED')
                ds=json.loads(meta.readline());source=read_cloud(ds['source_path']);deskew_ms=(time.monotonic_ns()-dstart)/1e6
                source_sha=digest(ds['source_path']);source_payload_sha=hashlib.sha256(np.ascontiguousarray(source,dtype='<f4').tobytes()).hexdigest();source_hashes.append(source_sha)
                # Full geometry replay binds immutable source files, never active scratch.
                source_path=Path(ds['source_path'])
                if strategy is not None:
                    source_path=slots/f'{ref}.bin';source_path.write_bytes(Path(ds['source_path']).read_bytes());source_slots.append((ref,source_path))
                roi=[True,1,*seed[:3,3].tolist(),0,0,0,0,seed[2,3]-10,seed[2,3]+30,120,seq]
                request=dict(case_id=f'KG_S3_{seq}',header=ref,group=f'KG_S3_{seq}',scene=cfg['mode'],source_path=str(source_path),target_path=paths['target'],target_sha=cfg['target_sha'],roi=roi,seed_common=seed.tolist(),q=50.,max_iterations=35,bd_config=dict(transformation_epsilon=.01,relative_objective_epsilon=.001,line_search=True,mode='gll'))
                N,align_ms=ndt.call(request);bresult=None
                if bgroup is not None:
                    dense=np.fromfile(ds['common_xyzi'],dtype='<f4').reshape(-1,4)[:,:3]
                    bresult=bgroup.on_frame(frame_id=str(seq),ref_ns=ref,available_ns=e['available'],dense=dense,source=source,U_T_base=base_pose(u,lever),deskew_history=dh)
                    if bresult['source_identity']!=source_payload_sha:raise RuntimeError('GLOBAL_LOCAL_SOURCE_IDENTITY_MISMATCH')
                update=dict(accepted=False,reason='MATCHER_INVALID');before=replay.replayed;ustart=time.monotonic_ns()
                if strategy is not None:
                    observer=strategy.create_observer(source_path=source_path,source_version=source_sha,target_version=cfg['target_sha'],roi=roi,worker_prior_hash=None,align_result=N)
                    observer=lifecycle.wrap(observer,f'KG_S3:{seq}')
                    event=dict(kind='ndt',header_ns=ref,message_id=10_000_000+seq,observation_id=f'KG_S3:{seq}',correlation_group=f'KG_S3:{seq}',observer=observer,pose=np.asarray(N['final']),R=np.eye(6))
                    why=replay.ingest(event);update=event.get('last_update',dict(accepted=False,reason=why))
                elif N['converged'] and N.get('candidate_valid',True):
                    try:old=engineering_R(N['fitness'])
                    except ValueError as exc:update=dict(accepted=False,reason='OBSERVATION_COVARIANCE_UNAVAILABLE',detail=str(exc))
                    else:
                        Rv=np.diag([old[0,0]]*3+[old[2,2]]*3)
                        event=dict(kind='ndt',header_ns=ref,message_id=10_000_000+seq,observation_id=f'KG_S3:{seq}',correlation_group=f'KG_S3:{seq}',observer=ExternalPose(lever,('x','y','z','rx','ry','rz')),pose=np.asarray(N['final']),R=Rv)
                        why=replay.ingest(event);update=event.get('last_update',dict(accepted=False,reason=why))
                replay.validate();accepted+=bool(update.get('accepted'));update_ms=(time.monotonic_ns()-ustart)/1e6
                cycle=(time.monotonic_ns()-tick)/1e6;cycles.append(cycle)
                log.put(dict(ref_ns=ref,available_ns=e['available'],raw_identities=ids,point_start_ns=span,source_sha256=source_sha,source_payload_sha256=source_payload_sha,deskew_history_sha256=dh,source_n=len(source),seed=seed,prior=dict(p=prior.p,v=prior.v,R=prior.R,bg=prior.bg,ba=prior.ba,P=Q),ndt=N,update=update,current=state_record(replay,lever),independent_U=base_pose(u,lever),B=bresult,deskew_ms=deskew_ms,align_ms=align_ms,delayed_update_ms=update_ms,replayed_events=replay.replayed-before,full_cycle_ms=cycle,execution='recorder-order-causal-serial',real_time_publication=False))
                cutoff=replay.base[0].t
                # all retained event sources remain addressable; aged checkpoints
                # cannot legally relinearize sources older than their base.
                while source_slots and source_slots[0][0]<=cutoff:
                    _,oldpath=source_slots.pop(0);oldpath.unlink()
                if seq%200==0:print(json.dumps(dict(scan=seq,accepted=accepted,elapsed_s=(time.monotonic_ns()-wall_start)/1e9,cycle_ms=cycle)),flush=True)
        expected=cfg.get('expected_ordered_identity_sha256')
        if expected and ordered.hexdigest()!=expected:raise RuntimeError('INPUT_IDENTITY_MISMATCH')
        if any(tail['pending_tail'].values()):raise RuntimeError('UNPAIRED_TAIL')
        reason='COMPLETED'
    except KeyboardInterrupt:
        reason='INTERRUPTED_RESEARCH_RUN_PREFIX_RETAINED'
    except Exception as exc:
        reason=repr(exc)
    finally:
        feed.close();log.close();inputs.close();lifecycle_log.close();ndt.close();desk.close();meta.close()
        (out/'event_lifecycle_summary.json').write_text(json.dumps(lifecycle.finish(replay),indent=2))
        if bgroup is not None:bgroup.close()
        if strategy is not None:strategy.close()
        rs=resources.close()
        for _,path in source_slots:path.unlink(missing_ok=True)
        (out/'initial_state.json').write_text(json.dumps(init_record,default=lambda x:x.tolist() if isinstance(x,np.ndarray) else x))
        status=dict(reason=reason,mode=cfg['mode'],map=cfg['map'],counts=counts,scans=seq,preinit_skips=skips,accepted=accepted,ordered_input_identity=ordered.hexdigest(),source_identity_chain=hashlib.sha256(''.join(source_hashes).encode()).hexdigest(),full_cycle_ms=summary(cycles),resource=rs,wall_s=(time.monotonic_ns()-wall_start)/1e9,execution='recorder-order-causal-serial',compute_latency_advanced_into_recorder_clock=False,live_100Hz_validated=False,production_qualified=False)
        (out/'run_status.json').write_text(json.dumps(status,indent=2));print(json.dumps(status),flush=True)
    if reason!='COMPLETED':raise SystemExit(2)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('config');run(p.parse_args().config)
