import bisect
"""Actual own-state CUDA deskew/rematching and 15D GMF, GNSS-free event API.
Restricted geometry/trajectories are written only to caller's private output.
"""
import argparse,json,time,copy,collections,gc,sys
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.state import State
from geometry_ieskf.common import C
from geometry_ieskf.external_pose import ExternalPose
from geometry_ieskf.optical_velocity import OpticalVelocity
from geometry_ieskf.directional_shadow import ndt_information,gicp_information,slip
from ndt_global.managed_replay import ManagedReplay
from ndt_global.multihypothesis import GaussianBank,ContinuousMapOdom
from ndt_global.validation import static_transform
from runtime import Service,Validator,read_cloud,write_cloud,partition,hashfile

def pose(x,lever):
    T=np.eye(4);T[:3,:3]=x.R;T[:3,3]=x.p-x.R@lever;return T

def run(a):
    assert not a.out.exists();a.out.mkdir(parents=True)
    sources=[Path(__file__),Path(__file__).with_name('runtime.py'),Path('src/ndt_global/managed_replay.py'),Path('src/ndt_global/multihypothesis.py'),Path('src/geometry_ieskf/directional_shadow.py')]
    (a.out/'source_version.json').write_text(json.dumps({str(p):hashfile(p) for p in sources},indent=2))
    c=json.load(open(a.paths));data=json.load(open(c['motion']));bindings=json.load(open(Path(c['data'])/('phase_ar_loop44_map0105/bindingslap.json' if a.window=='lap' else 'phase_as_accuracy_quality/bindings_multilap.json')))
    jobs=sorted([j for j in bindings if j['window']==a.window],key=lambda j:j['native_reference_ns']);jobs=jobs[:a.limit] if a.limit else jobs
    lo=min(v['scan_start_ns'] for v in jobs[0]['clouds'])-100_000_000;events=data['events'];init=max((e for e in events if e['kind']=='pose' and e['header_ns']<=lo),key=lambda e:e['header_ns']);t0=init['header_ns'];im=max((e for e in events if e['kind']=='imu' and e['header_ns']<=t0),key=lambda e:e['header_ns']);wh=max((e for e in events if e['kind']=='wheel' and e['header_ns']<=t0),key=lambda e:e['header_ns'])
    Tbi=static_transform(data['transforms'],'base_link','imu_bosch');lever=Tbi[:3,3];Rb=Tbi[:3,:3];R=Rotation.from_quat(init['quaternion_xyzw']).as_matrix();omega=Rb@im['omega'];x=State(np.array(init['position'])+R@lever,R@(np.array([wh['speed'],0,0])+np.cross(omega,lever)),R,np.zeros(3),np.zeros(3),t0);P=np.diag(np.repeat(C['initial_std_p_v_theta_bg_ba'],3)**2)
    replay=ManagedReplay(x,P,dict(header_ns=im['header_ns'],omega=omega,force=Rb@im['force']),lever)
    if a.optical!='off':replay.policy='optical_xy' if a.optical=='xy' else 'optical_lateral';replay.optical=OpticalVelocity(lever,static_transform(data['transforms'],'base_link','kistler'),(0,1) if a.optical=='xy' else (1,))
    continuous=replay.fork();bank=GaussianBank(replay);obs=ExternalPose(lever,('x','y','z','rx','ry','rz'));sensor=[]
    for e in events:
        if e['kind'] not in ('imu','wheel') or e['header_ns']<=t0:continue
        e=copy.deepcopy(e)
        if e['kind']=='imu':e['omega']=Rb@e['omega'];e['force']=Rb@e['force']
        sensor.append(e)
    if a.optical!='off':sensor.extend(e for e in json.load(open(c['optical']))['events'] if e['header_ns']>t0)
    for i,j in enumerate(jobs):sensor.append(dict(kind='clock',header_ns=j['native_reference_ns'],message_id=-i-1))
    sensor.sort(key=ManagedReplay.key);cursor=0;now=t0;source_archive={'ndt':{}};source_stamps={'ndt':[]};archive_lo=min(v['scan_start_ns'] for v in jobs[0]['clouds'])-20_000_000;archive_hi=jobs[-1]['native_reference_ns']
    def advance(until):
        nonlocal cursor,now
        while cursor<len(sensor) and sensor[cursor]['header_ns']<=until:
            e=sensor[cursor]
            for b in bank.branches:b.replay.ingest(e)
            continuous.ingest(e)
            cursor+=1
        now=max(now,until)
        if a.archive_source_history:
            for b in bank.branches:
                cache=source_archive.setdefault(b.uid,{})
                stamps=source_stamps.setdefault(b.uid,[])
                for snapshot in b.replay.snapshots:
                    stamp=snapshot[0].t
                    if archive_lo<=stamp<=archive_hi:
                        if stamp not in cache:bisect.insort(stamps,stamp)
                        cache[stamp]=snapshot
    services={'ndt':Service(c['ndt'],a.out/'ndt.log'),'gicp':Service(c['gicp'],a.out/'gicp.log')}
    desks={}
    for uid in (('ndt','gicp') if a.gmf else ('ndt',)):
        folder=a.out/('deskew_'+uid);folder.mkdir();desks[uid]=Service(c['deskew'],folder/'stderr.log',(folder/'meta.jsonl',folder));assert desks[uid].proc.stdout.readline().strip()=='READY';desks[uid].meta=(folder/'meta.jsonl').open()
    targetsha=hashfile(c['target']);validator=Validator(c['target']) if a.candidates else None;pool=ThreadPoolExecutor(2)
    # Cache map/GICP preprocessing before timing; startup is reported separately.
    empty=a.out/'empty.bin';write_cloud(empty,np.empty((0,3)))
    warm=dict(case_id='startup',header=t0,group='warm',scene='startup',source_path=str(empty),target_path=c['target'],target_sha=targetsha,roi=[False,1,0,0,0,0,0,0,0,-10000,10000,120,0],seed_common=np.eye(4).tolist(),q=50.,max_iterations=a.max_iterations,bd_config=dict(transformation_epsilon=.01,relative_objective_epsilon=.001,line_search=True,mode='gll'))
    boot=time.perf_counter_ns();services['ndt'].call(warm)
    if a.candidates:services['gicp'].call(warm)
    # Empty warmup prepares map structures only; its result is never fused. Startup retained.
    startup_ms=(time.perf_counter_ns()-boot)*1e-6;stats=[];counts=collections.Counter();lastref=None;frame=ContinuousMapOdom();gc.collect();gc.freeze()
    with (a.out/'frames.jsonl').open('w') as log:
      for i,job in enumerate(jobs):
        cycle=time.perf_counter_ns();ref=job['native_reference_ns'];available=max(v['arrival_ns'] for v in job['clouds']);advance(max(now,available));job_now=now;result=[];held_common=None;times=collections.Counter()
        # Fork on actual differing candidates, never manufacture a mode.
        original_branches=list(bank.branches)
        for b in original_branches:
            span=min(v['scan_start_ns'] for v in job['clouds'])
            if a.archive_source_history:
                stamps=source_stamps[b.uid];left=bisect.bisect_left(stamps,span-20_000_000);right=bisect.bisect_right(stamps,ref)
                hist=[source_archive[b.uid][stamp] for stamp in stamps[left:right]]
            else:hist=sorted((s for s in b.replay.snapshots if span-20_000_000<=s[0].t<=ref),key=lambda s:s[0].t)
            if not hist or hist[-1][0].t!=ref:raise ValueError('PRIOR_TIMESTAMP_MISSING')
            prior,Q,imu=hist[-1];seed=pose(prior,lever);rows=[]
            for s,_,_ in hist:
                if span-20_000_000<=s.t<=ref:
                    T=pose(s,lever);rows.append(np.r_[(s.t-ref)*1e-9,Rotation.from_matrix(T[:3,:3]).as_quat(),T[:3,3]])
            if len(rows)<2 or rows[0][0]>(span-ref)*1e-9:raise ValueError('SOURCE_DESKEW_HISTORY_MISSING')
            poses=a.out/f'{i:04d}_{b.uid}_poses.bin';np.asarray(rows,'<f8').tofile(poses);j=copy.deepcopy(job);j.update(header=ref,pose_path=str(poses),T_odom_native=seed.tolist(),T_common_from_native=np.eye(4).tolist(),gates_pass=True,scene='BE_'+b.uid)
            tick=time.perf_counter_ns();ds=desks[b.uid];ds.proc.stdin.write(json.dumps(j)+'\n');ds.proc.stdin.flush();ack=ds.proc.stdout.readline();assert ack;meta=json.loads(ds.meta.readline());times['deskew_ms']+=(time.perf_counter_ns()-tick)*1e-6
            source=Path(meta['source_path']);raw=read_cloud(source)
            if a.holdout:fit,hold=partition(raw);fitpath=a.out/f'{b.uid}_fit.bin';write_cloud(fitpath,fit)
            else:fitpath=source;hold=raw
            if held_common is None:held_common=hold
            req=dict(case_id=f'BE_{i}_{b.uid}',header=ref,group=f'frame_{i}',scene='BE',source_path=str(fitpath),target_path=c['target'],target_sha=targetsha,roi=[True,1,*seed[:3,3].tolist(),0,0,0,0,seed[2,3]-10,seed[2,3]+30,120,i+1],seed_common=seed.tolist(),q=50.,max_iterations=a.max_iterations,bd_config=dict(transformation_epsilon=.01,relative_objective_epsilon=.001,line_search=True,mode='gll'))
            tick=time.perf_counter_ns()
            if a.candidates and len(bank.branches)==1:
                f=pool.submit(services['ndt'].call,req);g=pool.submit(services['gicp'].call,req);N,nt=f.result();G,gt=g.result();times['ndt_ms']+=nt;times['gicp_ms']+=gt
                if a.gmf and N['converged'] and G['converged'] and N.get('candidate_valid',False) and G.get('candidate_valid',False) and GaussianBank.distinct(N['final'],G['final']):
                    fork_tick=time.perf_counter_ns();other=bank.fork(b.uid,'gicp','gicp');source_archive[other.uid]=dict(source_archive.get(b.uid,{}));source_stamps[other.uid]=list(source_stamps.get(b.uid,[]));times['gmf_ms']+=(time.perf_counter_ns()-fork_tick)*1e-6;result.append((other,G,prior,Q,imu,seed))
                primary=N
                if not a.gmf:result.append((None,G,prior,Q,imu,seed))
            else:
                primary,wall=services[b.backend].call(req);times[b.backend+'_ms']+=wall
            times['matcher_wall_ms']+=(time.perf_counter_ns()-tick)*1e-6
            result.append((b,primary,prior,Q,imu,seed))
        match_done=time.perf_counter_ns();advance(job_now+match_done-cycle)
        costs={};records=[]
        for b,N,prior,Q,imu,seed in result:
            backend='gicp' if b is None else b.backend;diag={};t=time.perf_counter_ns()
            if a.directional and 'score_hessian' in N:
                tangent=prior.v[:2];tangent=tangent/np.linalg.norm(tangent) if np.linalg.norm(tangent)>0 else np.array([1.,0.])
                diag['ndt']=[ndt_information(N['score_hessian'],np.array(N['parameters_internal']).reshape(6)[3:],tangent,L) for L in (5.,10.,20.)]
            if a.directional and 'H_right_rotation_translation' in N:
                diag['gicp']=[gicp_information(N['H_right_rotation_translation'],np.array(N['final'])[:3,:3],prior.v[:2],L) for L in (5.,10.,20.)]
            if a.directional:diag['slip']=slip(prior,imu['omega'],lever,imu['force'])
            validation=validator.cost(held_common,N['final']) if validator else None
            if validation is not None and not (N['converged'] and N.get('candidate_valid',True)):
                validation['cost']=20.;validation['invalid_candidate_penalty']=True
            times['diagnostic_validation_ms']+=(time.perf_counter_ns()-t)*1e-6
            update=dict(accepted=False,reason='CANDIDATE_SHADOW')
            if b is not None:
                from geometry_ieskf.ndt_pose_observation import engineering_R
                covariance_error=None;old=None
                if b.backend=='ndt' and N['converged'] and N.get('candidate_valid',True):
                    try:old=engineering_R(N['fitness'])
                    except ValueError as exc:covariance_error=str(exc)
                if N['converged'] and N.get('candidate_valid',True) and covariance_error is None:
                    # GICP pose-R is the same explicitly uncalibrated AW research model.
                    Rv=np.diag([old[0,0] if old is not None else .25]*3+[old[2,2] if old is not None else np.deg2rad(1.)**2]*3)
                    projection=None
                    if a.project_directional and b.backend=='ndt':
                        geo=diag['ndt'][1];L=np.asarray(geo['information']);eig,V=np.linalg.eigh(L);keep=eig>max(1e-12,eig[-1]*1e-3)
                        tangent=prior.v[:2];tangent=tangent/np.linalg.norm(tangent);U=np.array([[tangent[0],-tangent[1]],[tangent[1],tangent[0]]]);S=np.zeros((3,6));S[:2,:2]=U.T;S[2,3:]=10*prior.R[2,:]
                        extra=np.zeros((3,6));extra[0,2]=1;extra[1:,3:]=prior.R[:2,:]
                        projection=np.vstack((V[:,keep].T@S,extra));diag['retained_directions']=int(keep.sum())
                    e=dict(kind='ndt',projection=projection,header_ns=ref,message_id=10_000_000+i,observation_id=f'{b.uid}:{i}',correlation_group=f'frame_{i}',observer=obs,pose=np.asarray(N['final']),R=Rv)
                    if (a.reject_every and i%a.reject_every==0) or a.reject_start<=i<a.reject_end:b.replay.reject(e['observation_id'],ref,'INJECTED_REJECTION');update=dict(accepted=False,reason='INJECTED_REJECTION')
                    else:
                        tick=time.perf_counter_ns();reason=b.replay.ingest(e);times['delayed_update_ms']+=(time.perf_counter_ns()-tick)*1e-6;update=e.get('last_update',dict(accepted=False,reason=reason))
                else:
                    update=dict(accepted=False,reason='OBSERVATION_COVARIANCE_UNAVAILABLE' if covariance_error else 'MATCHER_INVALID',detail=covariance_error)
                    b.replay.reject(f'{b.uid}:{i}',ref,update['reason'])
                if validation:costs[b.uid]=validation['cost']
                b.replay.validate();xpost,Ppost,_=b.replay.current()
                records.append(dict(branch=b.uid,backend=backend,prior=dict(p=prior.p.tolist(),v=prior.v.tolist(),R=prior.R.tolist(),bg=prior.bg.tolist(),ba=prior.ba.tolist(),P=Q.tolist()),seed=seed.tolist(),ndt=N,update=update,diagnostics=diag,validation=validation,state=dict(t=xpost.t,p=xpost.p.tolist(),v=xpost.v.tolist(),R=xpost.R.tolist(),bg=xpost.bg.tolist(),ba=xpost.ba.tolist(),P=Ppost.tolist())))
            else:records.append(dict(branch='shadow_gicp',backend=backend,ndt=N,validation=validation))
        if a.gmf and len(costs)==len(bank.branches) and all(r.get('update',{}).get('accepted',False) for r in records if r['branch']!='shadow_gicp'):
            gmf_tick=time.perf_counter_ns();bank.evidence(f'frame_{i}',costs);times['gmf_ms']+=(time.perf_counter_ns()-gmf_tick)*1e-6
        if a.retract_every and i>3 and i%a.retract_every==0:
            targets=[(b,f'{b.uid}:{i-3}') for b in bank.branches]
            accepted=[(b,uid) for b,uid in targets if uid in b.replay.records and b.replay.records[uid].status=='ACCEPTED']
            # Preflight every branch before changing any state or shared evidence.
            if any(b.replay.records[uid].stamp_ns<=b.replay.base[0].t for b,uid in accepted):
                counts['REVOCATION_OUTSIDE_HISTORY']+=1
            else:
                rollback_tick=time.perf_counter_ns()
                for b,uid in accepted:b.replay.retract(uid)
                if accepted:bank.revoke_evidence(f'frame_{i-3}')
                times['rollback_replay_ms']+=(time.perf_counter_ns()-rollback_tick)*1e-6
        advance(job_now+time.perf_counter_ns()-cycle)
        gmf_tick=time.perf_counter_ns();selected=bank.select(ref);times['gmf_ms']+=(time.perf_counter_ns()-gmf_tick)*1e-6;current,PC,_=selected.replay.current();T=pose(current,lever)
        # Continuous odom is a separate motion-only 15D chain, advanced below by raw events.
        # This contract records map revisions without commanding a controller.
        O=pose(continuous.current()[0],lever);packet=frame.correct(T,O);lastref=current.t
        counts.update(r['update']['reason'] for r in records if 'update' in r)
        row=dict(index=i,ref=ref,state_stamp=current.t,available=available,complete=now,selected=selected.uid,weights=bank.weights.tolist(),branches=records,pose=T.tolist(),P=PC.tolist(),odom=O.tolist(),map_odom=packet['map_from_odom'].tolist(),frame_version=packet['version'],times=dict(times),cycle_ms=(time.perf_counter_ns()-cycle)*1e-6)
        log.write(json.dumps(row)+'\n');log.flush();inclusive_ms=(time.perf_counter_ns()-cycle)*1e-6
        with (a.out/'inclusive_timing.jsonl').open('a') as timing:timing.write(json.dumps(dict(index=i,cycle_ms=inclusive_ms))+'\n')
        stats.append(inclusive_ms)
        now=max(now,job_now+round(inclusive_ms*1e6))
        if i%100==0:print(i+1,len(jobs),len(bank.branches),row['cycle_ms'],flush=True)
    for service in services.values():service.close()
    for ds in desks.values():ds.close()
    pool.shutdown()
    for b in bank.branches:
        (a.out/(b.uid+'_lifecycle.json')).write_text(json.dumps(dict(records={k:vars(v) for k,v in b.replay.records.items()},revisions=b.replay.revision_log),indent=2))
    summary=dict(frames=len(stats),requested=len(jobs),window=a.window,startup_ms=startup_ms,cycle_ms={str(p):float(np.percentile(stats,p)) for p in (50,95,99,100)},over_50ms=sum(v>50 for v in stats),switches=bank.switches,decisions=dict(counts),configuration=vars(a).copy(),GNSS_online=False,weights_calibrated=False,weight_revisions=bank.weight_revisions,control_safety_validated=False)
    summary['configuration']={k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()};(a.out/'summary.json').write_text(json.dumps(summary,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--paths',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--window',choices=['lap','multilap'],default='lap');p.add_argument('--limit',type=int);p.add_argument('--optical',choices=['off','lateral','xy'],default='off');p.add_argument('--directional',action='store_true');p.add_argument('--project-directional',action='store_true');p.add_argument('--candidates',action='store_true');p.add_argument('--gmf',action='store_true');p.add_argument('--holdout',action='store_true');p.add_argument('--reject-every',type=int,default=0);p.add_argument('--reject-start',type=int,default=-1);p.add_argument('--reject-end',type=int,default=-1);p.add_argument('--retract-every',type=int,default=0);p.add_argument('--max-iterations',type=int,default=35);p.add_argument('--archive-source-history',action='store_true');a=p.parse_args()
    if a.project_directional and not a.directional:p.error('--project-directional requires --directional')
    if a.gmf and not a.candidates:p.error('--gmf requires --candidates')
    run(a)
