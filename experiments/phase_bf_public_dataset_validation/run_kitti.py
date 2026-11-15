"""BF1 bounded single-scene causal experiment using existing15D/NDT models.
The online runner never opens offline_reference.npz or map-side navigation records.
"""
import argparse,bisect,hashlib,importlib.util,json,os,time
from pathlib import Path
import numpy as np
from geometry_ieskf.state import State
from geometry_ieskf.common import C
from geometry_ieskf.external_pose import ExternalPose
from geometry_ieskf.ndt_pose_observation import engineering_R
from geometry_ieskf.pose_uncertainty import native_pose_R
from geometry_ieskf.shadow_candidate import linear_shadow_candidate
from geometry_ieskf.vertical_velocity import fixed_normal_model,body_normal_model
from ndt_global.managed_replay import ManagedReplay
from geometry_ieskf.inertial import InertialPredictor

spec=importlib.util.spec_from_file_location('be_runtime',Path(__file__).parents[1]/'phase_be_e0_e4/runtime.py');rt=importlib.util.module_from_spec(spec);spec.loader.exec_module(rt)
COMPONENTS=('x','y','z','rx','ry','rz')
CONFIG=dict(transformation_epsilon=.01,relative_objective_epsilon=.001,line_search=True,mode='gll')

def clean(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    raise TypeError(type(x).__name__)

def save(path,v):Path(path).write_text(json.dumps(v,default=clean,indent=2,allow_nan=False)+'\n')
def snap(x,P):return dict(t_ns=x.t,p=x.p,v=x.v,R=x.R,bg=x.bg,ba=x.ba,P=P)

def legacy_R(f):
    r=engineering_R(f);return np.diag([r[0,0]]*3+[r[2,2]]*3)

def request(root,targetsha,source,i,t,T):
    return dict(case_id=f'KITTI_{i}',header=int(t),group=f'kitti0002:{i}',scene='KITTI0002',source_path=str(source),target_path=str(root/'map.bin'),target_sha=targetsha,
        roi=[True,1,*T[:3,3].tolist(),0,0,0,0,float(T[2,3]-10),float(T[2,3]+30),120,i+1],seed_common=T.tolist(),q=50.,max_iterations=35,bd_config=CONFIG)

def get_at(replay,t):
    checkpoints=[replay.base]+replay.snapshots
    found=[s for s in checkpoints if s[0].t<=t]
    if not found:raise ValueError('SEED_OUTSIDE_HISTORY')
    x,P,imu=found[-1];x=x.copy();P=P.copy()
    if x.t<t:
        if t-imu['header_ns']>25000000:raise ValueError('SEED_IMU_GAP')
        x,P=InertialPredictor().predict(x,P,imu['omega'],imu['force'],(t-x.t)*1e-9);x.t=t
    return x,P,imu

def advance(replay,online,cursor,t):
    if t<replay.current()[0].t:return cursor
    it=online['imu_t']
    while cursor<len(it) and int(it[cursor])<=t:
        e=dict(kind='imu',header_ns=int(it[cursor]),message_id=cursor,omega=online['omega'][cursor],force=online['force'][cursor])
        replay.ingest(e);cursor+=1
    replay.ingest(dict(kind='clock',header_ns=int(t),message_id=1000000000+int(t-online['initial_t'])))
    return cursor

def valid_match(n):
    if not n.get('converged') or not n.get('candidate_valid'):return False,'MATCHER_NOT_VALID'
    T=np.asarray(n['final']);
    if not np.isfinite(T).all():return False,'NONFINITE_POSE'
    return True,'VALID_LOCAL_MATCH_NOT_GLOBAL_CERTIFICATE'

def run(root,out,matcher,mode,raw,arrival_schedule=None):
    root=Path(root);out=Path(out);out.mkdir(parents=True,exist_ok=False)
    schedule=json.loads(Path(arrival_schedule).read_text()) if arrival_schedule else None
    online=np.load(root/'online.npz');assert set(online.files)=={'imu_t','omega','force','scan_t','scan_start','scan_end','initial_p','initial_R','initial_v','T_IMU_velo','initial_t'}
    t0=int(online['initial_t']);x=State(online['initial_p'].copy(),online['initial_v'].copy(),online['initial_R'].copy(),np.zeros(3),np.zeros(3),t0)
    P=np.diag(np.repeat(C['initial_std_p_v_theta_bg_ba'],3)**2);j=int(np.searchsorted(online['imu_t'],t0,side='right')-1)
    imu=dict(header_ns=int(online['imu_t'][j]),omega=online['omega'][j],force=online['force'][j]);replay=ManagedReplay(x,P,imu,np.zeros(3));cursor=j+1
    obs=ExternalPose(np.zeros(3),COMPONENTS);targetsha=rt.hashfile(root/'map.bin');service=rt.Service(matcher,out/'matcher.log')
    save(out/'config.json',dict(mode=mode,baseline='143ce47adfb60cbc8d1f069d4945b33641e1ca96',ndt=CONFIG,max_iterations=35,q=50,resolution=1.5,step_size=.2,neighbor='DIRECT7',initial=snap(x,P),arrival_contract='fixed empirical B0 availability' if schedule else 'measured-wall-time availability simulation',gnss_updates=False,wheel=False,C=False,A='shadow only',source_root_identity=hashlib.sha256((root/'config.json').read_bytes()).hexdigest(),matcher_sha256=rt.hashfile(matcher)))
    empty=out/'empty.bin';rt.write_cloud(empty,np.empty((0,3)));warm=request(root,targetsha,empty,-1,t0,np.eye(4));warm['roi'][0]=False
    warmstart=time.perf_counter();warmresult,warm_ms=service.call(warm);save(out/'startup.json',dict(ms=warm_ms,result=warmresult,excluded_from_tracking_cycle=True))
    failure=None;count=0;rows=[]
    try:
        with (out/'frames.jsonl').open('x') as f, (out/'cycle_profile.jsonl').open('x') as profile:
            for i in range(20,77):
                tick=time.perf_counter_ns();t=int(online['scan_t'][i]);release=int(online['scan_end'][i]);current=replay.current()[0].t
                cursor=advance(replay,online,cursor,max(t,current));prior,Q,held=get_at(replay,t)
                seed=np.eye(4);seed[:3,:3]=prior.R;seed[:3,3]=prior.p
                prepare_ms=(time.perf_counter_ns()-tick)*1e-6
                input_tick=time.perf_counter_ns()
                rawpath=Path(raw)/'2011_09_26/2011_09_26_drive_0002_sync/velodyne_points/data'/f'{i:010d}.bin'
                points=np.fromfile(rawpath,dtype='<f4').reshape(-1,4)[:,:3]
                if not np.isfinite(points).all():raise ValueError('RAW_POINT_NONFINITE')
                T=online['T_IMU_velo'];points=points@T[:3,:3].T+T[:3,3]
                source=out/'current_source.bin';rt.write_cloud(source,points)
                input_decode_transform_ms=(time.perf_counter_ns()-input_tick)*1e-6
                N,service_ms=service.call(request(root,targetsha,source,i,t,seed))
                common,why=valid_match(N);r,H=obs.residual(prior,np.asarray(N['final']))
                frozen={};Rall={}
                for name in ('B0','B1'):
                    try:
                        R=legacy_R(N['fitness']) if name=='B0' else native_pose_R(N,COMPONENTS,prior,np.eye(4))
                        Rall[name]=R;frozen[name]=linear_shadow_candidate(Q,H,r,R)|dict(common_match_valid=common)
                    except (ValueError,np.linalg.LinAlgError,KeyError,TypeError) as err:frozen[name]=dict(committed=False,reason=str(err))
                # Known IMU->Velodyne lever only. No unprovided rear-axle/contact point.
                lever=-online['T_IMU_velo'][:3,3]
                h1,H1=fixed_normal_model(prior,held['omega'],lever,[0,0,1],normal_source='ENU map up, NOT measured road normal')
                h2,H2=body_normal_model(prior,held['omega'],lever)
                ashadow=dict(A1=dict(residual=-h1,H=H1),A2=dict(residual=-h2,H=H2),committed=False,reference_point='Velodyne sensor origin expressed in IMU axes; not ground contact/rear axle')
                # Physical observation cannot be available before spin completion and measured compute.
                # Clock is deterministic sensor-time simulation, not a ROS2 deadline qualification.
                process_before_update_ns=time.perf_counter_ns()-tick
                measured_available=max(release,current)+process_before_update_ns
                available=int(schedule[str(i)]) if schedule else measured_available
                if available<max(release,current):raise ValueError('INVALID_FIXED_AVAILABILITY')
                cursor=advance(replay,online,cursor,available)
                correction_tick=time.perf_counter_ns();uid=f'BF1:{mode}:{i}'
                if common and mode in Rall:
                    event=dict(kind='ndt',header_ns=t,message_id=100000+i,observation_id=uid,correlation_group=f'scan:{i}',observer=obs,pose=np.asarray(N['final']),R=Rall[mode])
                    status=replay.ingest(event);update=event.get('last_update',dict(accepted=False,reason=status))
                else:replay.reject(uid,t,why if not common else 'R_UNAVAILABLE');update=dict(accepted=False,reason=why if not common else 'R_UNAVAILABLE')
                replay.validate();replay_ms=(time.perf_counter_ns()-correction_tick)*1e-6
                posterior,Qpost=replay.current()[:2];at_scan,atP,_=get_at(replay,t)
                cycle_ms=(time.perf_counter_ns()-tick)*1e-6
                row=dict(frame=i,observation_id=uid,scan_epoch_ns=t,scan_end_ns=release,available_time_ns=available,measured_available_if_uncontrolled_ns=measured_available,scheduled_availability=bool(schedule),prior=snap(prior,Q),seed=seed,ndt=N,
                         frozen=frozen,A_shadow=ashadow,actual_update=update,actual_published_state=snap(posterior,Qpost),revised_scan_state=snap(at_scan,atP),
                         cycle_ms=cycle_ms,service_ms=service_ms,prepare_and_prior_ms=prepare_ms,input_decode_transform_ms=input_decode_transform_ms,rollback_replay_ms=replay_ms,
                         epoch_to_available_ms=(available-t)*1e-6,existing_replayed_events=replay.replayed)
                f.write(json.dumps(row,default=clean,allow_nan=False)+'\n');f.flush()
                end_ms=(time.perf_counter_ns()-tick)*1e-6
                profile.write(json.dumps(dict(frame=i,compute_before_serialization_ms=cycle_ms,instrumented_full_cycle_ms=end_ms,serialization_log_ms=end_ms-cycle_ms,over_50ms=end_ms>50))+'\n');profile.flush();count+=1
                print(mode,i,'fitness',N['fitness'],'exit',N['exit'],'accepted',update['accepted'],'cycle',round(cycle_ms,2),flush=True)
    except Exception as err:
        failure=dict(reason=type(err).__name__+': '+str(err),completed=count,next_frame=20+count);save(out/'failure.json',failure);print('FAILED',failure,flush=True)
    finally:service.close()
    save(out/'summary.json',dict(mode=mode,completed=count,expected=57,failure=failure,ledger={u:vars(v) for u,v in replay.records.items()},replayed=replay.replayed))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--out',required=True);p.add_argument('--matcher',required=True);p.add_argument('--mode',choices=['B0','B1'],required=True);p.add_argument('--raw',required=True);p.add_argument('--arrival-schedule');a=p.parse_args();run(a.root,a.out,a.matcher,a.mode,a.raw,a.arrival_schedule)
