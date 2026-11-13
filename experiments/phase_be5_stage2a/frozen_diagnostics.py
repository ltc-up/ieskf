"""<=60 saved frames. No matching, point clouds, replay, online commit or GNSS fusion."""
import argparse,copy,csv,hashlib,json,time,collections,os
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.linalg import cho_factor,cho_solve
from geometry_ieskf.state import State
from geometry_ieskf.external_pose import ExternalPose
from geometry_ieskf.pose_uncertainty import native_pose_R
from geometry_ieskf.vertical_velocity import fixed_normal_model,body_normal_model
from geometry_ieskf.attitude_reference import source_diagnostic
from geometry_ieskf.shadow_candidate import linear_shadow_candidate
from ndt_global.validation import static_transform
from frozen_contract import qualify_frame,qualify_dataset_certificate

BUDGET=500_000_000
COMP=('x','y','z','rx','ry','rz')

def clean(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    raise TypeError(type(x).__name__)
def dump(path,d):path.write_text(json.dumps(d,default=clean,allow_nan=False,indent=2))
def disk_bytes(path):return sum(p.stat().st_size for p in path.rglob('*') if p.is_file())
def frozen_state(z,t):return State(*(np.asarray(z[k],float) for k in ('p','v','R','bg','ba')),int(t))

def decompose(P,H,r,S):
    inv=cho_solve(cho_factor(S),np.eye(len(r)))
    out={}
    for name,sl in [('position',slice(0,3)),('velocity',slice(3,6)),('theta_right',slice(6,9)),
                    ('gyro_bias',slice(9,12)),('accel_bias',slice(12,15))]:
        out[name]=dict(z=float(P[2,sl]@H[:,sl].T@inv@r),
                       vz=float(P[5,sl]@H[:,sl].T@inv@r))
    return out

def A_shadow(x,P,omega,lever):
    ans={}
    for label in ('A1_fixed_map_up_hypothesis','A2_body_up_hypothesis'):
        if label.startswith('A1'):
            h,H=fixed_normal_model(x,omega,lever,[0.,0.,1.],normal_source='COORDINATE_MAP_UP_HYPOTHESIS_NOT_SURVEYED_ROAD')
        else:h,H=body_normal_model(x,omega,lever)
        q=linear_shadow_candidate(P,H,[-h],[[.5**2]])
        q.update(prediction=h,zero_observation_assumed=True,physical_fusion_eligible=False,
                 normal_source='coordinate map-up, NOT a measured road normal' if label.startswith('A1') else 'state-dependent body up, NOT road normal',
                 variance_provenance='declared 0.5 m/s soft-contact engineering assumption, not calibrated',
                 omitted_risks=['shared gyro noise and state correlation','suspension/heave/contact velocity','road bank/slope versus chosen normal'],
                 gain_contribution_by_state_block=decompose(P,H,np.array([-h]),q['S']))
        ans[label]=q
    return ans

def pose_shadow(x,P,lever,ndt,R):
    obs=ExternalPose(lever,COMP);r,H=obs.residual(x,ndt['final'])
    q=linear_shadow_candidate(P,H,r,R)
    _,_,decision=obs.correct(x.copy(),P.copy(),ndt['final'],np.array(R,copy=True))
    q.update(original_update_would_admit=decision['accepted'],original_update_reason=decision['reason'],
             gain_contribution_by_state_block=decompose(P,H,r,q['S']))
    for k in ('z','vz'):
        row=2 if k=='z' else 5
        assert abs(sum(d[k] for d in q['gain_contribution_by_state_block'].values())-q['delta_proposed'][row])<1e-9
    return q

def run(root,out,contract,only_index=None):
    if out.exists():raise ValueError('REFUSE_OVERWRITE')
    out.mkdir(parents=True)
    if disk_bytes(out)>BUDGET:raise ValueError('BUDGET')
    repo=Path(__file__).resolve().parents[2]
    paths=json.load(open(root/'runtime/paths.json'))
    run=root/'results/controlled2_matched_lap'
    # Select by saved metadata only. No update is evaluated during this scan.
    csvrows={int(r['index']):r for r in csv.DictReader((run/'trajectory_and_errors.csv').open())}
    meta=[]
    with (run/'frames.jsonl').open() as f:
        for line in f:
            r=json.loads(line);b=next(b for b in r['branches'] if b['branch']==r['selected'])
            meta.append(dict(index=r['index'],fitness=b['ndt'].get('fitness'),vz_change=abs(b['update'].get('delta_proposed',[0]*15)[5])))
    selected={};maximum=48
    def add(i,reason):
        if len(selected)<maximum or i in selected:selected.setdefault(i,[]).append(reason)
    for seg in (3,9,5,8):
        ids=[i for i,r in csvrows.items() if int(r['segment'])==seg]
        for i in ids[::max(1,len(ids)//6)][:6]:add(i,'fixed_segment_'+str(seg))
    for m in sorted(meta,key=lambda m:m['fitness'] if isinstance(m['fitness'],(int,float)) and np.isfinite(m['fitness']) else -1,reverse=True)[:8]:add(m['index'],'high_fitness_in_this_baseline')
    for m in sorted(meta,key=lambda m:m['vz_change'],reverse=True)[:6]:add(m['index'],'large_saved_vz_proposal')
    for i,r in csvrows.items():
        if r['reference_valid']=='True' and float(r['xy_proxy_error_m'])>=2:add(i,'offline_proxy_bad_not_truth')
    for i in np.linspace(0,len(meta)-1,48,dtype=int):add(int(i),'coverage_fill')
    if only_index is not None:
        if only_index not in csvrows:raise ValueError('SUPPLEMENT_INDEX_UNAVAILABLE')
        selected={only_index:['supplement_offline_raw_match_proxy_bad_not_truth']}
    assert len(selected)<=48
    dump(out/'selection.json',dict(run=run.name,selection=selected,max_BE_frames=48,
                                  offline_reference_used_only_for_case_selection=True,not_a_classifier=True))
    # Only raw IMU lookup/TF, never replay inputs.
    data=json.load(open(paths['motion']))
    Tbi=static_transform(data['transforms'],'base_link','imu_bosch')
    imu=[e for e in data['events'] if e['kind']=='imu'];stamps=np.array([e['header_ns'] for e in imu])
    lever=Tbi[:3,3];framecert=json.load(open(contract))
    captured=json.load(open(run/'source_version.json'))
    expected_target=json.load(open(repo/'experiments/phase_be_e0_e4/manifest.json'))['input_sha256']['target']
    qualify_dataset_certificate(framecert,run.name,captured,expected_target)
    stats=[];count=0;failures=[];native_contract_failures=[];parities=[]
    with (out/'frames.jsonl').open('x') as log,(run/'frames.jsonl').open() as inp:
        for line in inp:
            row=json.loads(line);i=row['index']
            if i not in selected:continue
            started=time.perf_counter_ns()
            b=next(b for b in row['branches'] if b['branch']==row['selected'])
            x=frozen_state(b['prior'],row['ref']);P=np.asarray(b['prior']['P'],float)
            before={k:np.array(getattr(x,k),copy=True) for k in ('p','v','R','bg','ba')};Pbefore=P.copy()
            ndt=b['ndt'];ii=int(np.searchsorted(stamps,x.t,side='right')-1)
            rec=dict(case_id=f'BE_{i}',run=run.name,index=i,header_ns=x.t,observation_id=f'ndt:{i}',
                     correlation_group=f'frame_{i}',selection_reasons=selected[i],
                     saved_row_sha256=hashlib.sha256(line.encode()).hexdigest(),
                     prior=b['prior'],seed=b['seed'],original_ndt=ndt,
                     original_R=b['update'].get('R'),original_update=b['update'],
                     actual_branch_state=b['state'],actual_branch_state_stamp=b['state']['t'],
                     actual_published_base_pose=row['pose'],actual_published_P=row['P'],
                     actual_published_state_stamp=row['state_stamp'],actual_published_velocity=None,
                     actual_published_velocity_status='NOT_EXPORTED_AT_FINAL_STAMP_IN_ORIGINAL_TRACE',
                     actual_scan_commit_delta=b['update'].get('delta',np.zeros(15).tolist()),
                     all_stage2a_variants_committed=False,C_shadow=dict(status='BLOCKED_NO_QUALIFIED_REFERENCE',source_diagnostic=source_diagnostic()))
            if ii<0 or not 0<=x.t-stamps[ii]<=25_000_000:
                rec['A_shadow']=dict(status='IMU_TIME_CONTRACT_UNAVAILABLE')
            else:
                omega=Tbi[:3,:3]@np.asarray(imu[ii]['omega'])
                rec['IMU_identity']=dict(message_id=imu[ii]['message_id'],header_ns=imu[ii]['header_ns'],age_ns=int(x.t-stamps[ii]))
                rec['A_shadow']=A_shadow(x,P,omega,lever)
            if 'R' in b['update']:
                try:
                    q=pose_shadow(x,P,lever,ndt,b['update']['R'])
                    rec['Legacy_candidate']=q
                    error=float(np.max(abs(q['delta_proposed']-np.asarray(b['update'].get('delta_proposed',q['delta_proposed'])))))
                    rec['Legacy_saved_delta_proposal_max_difference']=error;parities.append(error)
                except (ValueError,np.linalg.LinAlgError) as exc:
                    rec['Legacy_candidate']=dict(status='INVALID',reason=str(exc));failures.append([i,'Legacy',str(exc)])
            else:rec['Legacy_candidate']=dict(status='ORIGINAL_BASELINE_R_UNAVAILABLE',reason=b['update'].get('reason'))
            try:
                A=qualify_frame(ndt,x,b['seed'],x.t,lever,framecert)
                RN=native_pose_R(ndt,COMP,x,T_map_pcm=A)
                q=pose_shadow(x,P,lever,ndt,RN)
                q['frame_contract']='CONFIRMED_SAVED_RESEARCH_NUMERIC_FRAME_ONLY'
                q['R_internal']=ndt['native_internal']
                q['original_matcher_eligible']=bool(ndt['converged'] and ndt.get('candidate_valid',False))
                rec['Native_Raw_candidate']=q
            except (ValueError,np.linalg.LinAlgError,KeyError) as exc:
                rec['Native_Raw_candidate']=dict(status='STOPPED_NATIVE_CONTRACT_OR_COVARIANCE',reason=str(exc))
                native_contract_failures.append([i,str(exc)])
            for k in before:np.testing.assert_array_equal(getattr(x,k),before[k])
            np.testing.assert_array_equal(P,Pbefore)
            rec['input_unchanged']=True;rec['diagnostic_ms']=(time.perf_counter_ns()-started)*1e-6
            log.write(json.dumps(rec,default=clean,allow_nan=False)+'\n');log.flush();stats.append(rec['diagnostic_ms']);count+=1
            if disk_bytes(out)>BUDGET:raise ValueError('BUDGET')
    if only_index is not None:
        summary=dict(real_frames=count,BE_pose_frames=count,historical_vertical_frames=0,historical12632=0,
            native_failures=native_contract_failures,other_failures=failures,
            legacy_delta_proposal_max_parity_error=max(parities,default=None),
            diagnostic_ms={str(q):float(np.percentile(stats,q)) for q in (50,95,99,100)},
            timing_scope='single-frame supplemental frozen algebra, not complete cycle',
            new_matching=0,new_deskew=0,new_replays=0,committed_stage2a_updates=0,C_real_reference_count=0,
            independent_vertical_truth=False,storage_bytes=disk_bytes(out),state_P_unchanged=True)
        dump(out/'summary.json',summary)
        print(json.dumps(summary,indent=2))
        return
    # Five existing old point-factor snapshots: A only, no invented Native pose/R.
    hist=repo/'data/history/phase_z/results'
    old=json.load(open(hist/'frozen_inputs.json'));z1=json.load(open(hist/'z1_summary.json'))
    with (out/'historical_vertical.jsonl').open('x') as f:
        for record,prior_decomposition in zip(old,z1):
            z=record['predicted'];x=State(np.asarray(z['p_imu_map']),np.asarray(z['v_imu_map']),
              Rotation.from_quat(z['q_map_body_xyzw']).as_matrix(),np.asarray(z['bg_body_rad_s']),
              np.asarray(z['ba_body_m_s2']),int(z['state_time_ns']));P=np.asarray(z['P'])
            assert x.t==record['header_ns']
            ii=int(np.searchsorted(stamps,x.t,side='right')-1)
            entry=dict(case=record['case'],prior=z,historical_signed_vz_decomposition=prior_decomposition,
                       actual_historical_return='V_returned is distinct from Y diagnostic proposals',
                       Native_Raw_candidate=dict(status='STOPPED_NO_NATIVE_6DOF_OBSERVATION_CONTRACT'),
                       Legacy_candidate=dict(status='NOT_A_BE_POSE_OBSERVATION_BASELINE'),
                       C_shadow=dict(status='BLOCKED_NO_QUALIFIED_REFERENCE'),all_stage2a_variants_committed=False)
            sameTF=np.allclose(record['extrinsic']['R_body_imu'],Tbi[:3,:3],atol=1e-10) and np.allclose(record['extrinsic']['lever_body_imu_m'],lever,atol=1e-10)
            if ii>=0 and 0<=x.t-stamps[ii]<=25_000_000 and sameTF:
                entry['A_shadow']=A_shadow(x,P,Tbi[:3,:3]@np.asarray(imu[ii]['omega']),lever)
            else:entry['A_shadow']=dict(status='STOPPED_IMU_TIME_OR_TF_CONTRACT')
            f.write(json.dumps(entry,default=clean,allow_nan=False)+'\n');count+=1
    # Historical12632 lacks a paired 15D prior in its matcher-only frozen record.
    historic=json.loads(next((root/'results/e2_historical12632/results.jsonl').open()))
    dump(out/'historical12632.json',dict(original_matcher_record=historic,
            all_candidate_updates=dict(status='STOPPED_NO_PAIRED_15D_PRIOR_AND_P'),committed=False))
    count+=1
    assert count<=60
    summary=dict(real_frames=count,BE_pose_frames=len(selected),historical_vertical_frames=len(old),historical12632=1,
       native_failures=native_contract_failures,other_failures=failures,
       legacy_delta_proposal_max_parity_error=max(parities,default=None),
       diagnostic_ms={str(q):float(np.percentile(stats,q)) for q in (50,95,99,100)},
       timing_scope='frozen algebra only, not complete localization cycle',
       new_matching=0,new_deskew=0,new_replays=0,committed_stage2a_updates=0,
       C_real_reference_count=0,independent_vertical_truth=False,
       storage_bytes=disk_bytes(out),state_P_unchanged=True)
    dump(out/'summary.json',summary);print(json.dumps(summary,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--frame-contract',type=Path,required=True)
    p.add_argument('--only-index',type=int);a=p.parse_args();run(a.root,a.out,a.frame_contract,a.only_index)
