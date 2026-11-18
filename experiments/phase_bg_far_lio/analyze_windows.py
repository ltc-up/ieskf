"""Offline evaluation only. Must not be imported by the replay/estimator."""
import argparse,json,collections,csv
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp


def stats(x):
    x=np.asarray(x,float);x=x[np.isfinite(x)]
    if not len(x):return dict(n=0)
    return dict(n=len(x),rmse=float(np.sqrt(np.mean(x*x))),p50=float(np.percentile(abs(x),50)),p95=float(np.percentile(abs(x),95)),p99=float(np.percentile(abs(x),99)),max=float(np.max(abs(x))))


def interpolate(stamps,p,R,grid):
    t=np.asarray(stamps,dtype=np.int64);order=np.argsort(t);t=t[order];p=np.asarray(p)[order];R=np.asarray(R)[order]
    t,u=np.unique(t,return_index=True);p=p[u];R=R[u]
    good=(grid>=t[0])&(grid<=t[-1]);g=np.clip(grid,t[0],t[-1]);right=np.clip(np.searchsorted(t,g),1,len(t)-1);left=right-1;f=(g-t[left])/(t[right]-t[left])
    xyz=p[left]*(1-f[:,None])+p[right]*f[:,None]
    rot=Slerp((t-t[0])*1e-9,Rotation.from_matrix(R))((g-t[0])*1e-9).as_matrix()
    return xyz,rot,good,t[right]-t[left]


def evaluate(p,R,ix,reference,tangent,valid=None):
    result={};ry=Rotation.from_matrix(reference['R'][ix]).as_euler('xyz')[:,2];yaw=Rotation.from_matrix(R).as_euler('xyz')[:,2]
    for lag in ['0.0','0.075','0.15']:
        good=reference['raw_valid_'+lag][ix].copy()
        if valid is not None:good &= valid
        e=p-reference['G_'+lag][ix];tan=tangent[ix];lat=-tan[:,1]*e[:,0]+tan[:,0]*e[:,1];lon=np.sum(tan*e[:,:2],axis=1);dy=np.angle(np.exp(1j*(yaw-ry)))
        result[lag]=dict(xy=stats(np.linalg.norm(e[good,:2],axis=1)),lateral=stats(lat[good]),longitudinal=stats(lon[good]),yaw_deg=stats(np.rad2deg(dy[good])),z_engineering_proxy=stats(e[good,2]))
    return result


def main(a):
    root=Path(a.root);be=Path(a.be);data=Path(a.data);out=root/'results/analysis'
    if out.exists():raise ValueError('REFUSE_ANALYSIS_OVERWRITE')
    out.mkdir();ref=np.load(be/'runtime/reference_lap.npz');fixed=np.load(data/'phase_as_accuracy_quality/single_lap/evaluation.npz');h=ref['header'];lookup={int(t):i for i,t in enumerate(h)}
    motion=json.loads((data/'phase_ao_ap_validation/loop44.json').read_text());production=[x for x in motion['events'] if x['kind']=='pose' and x['valid']]
    baseline=[json.loads(l) for l in (be/'results/controlled2_matched_lap/frames.jsonl').open()]
    sel=json.loads((root/'audit/frozen_windows.json').read_text());result=dict(scope='Two preregistered segments only; no independent GT; no 6-state runtime identity certification',windows={});cases=[]
    for w in sel['windows']:
        name=w['name'];folder=root/'results'/('normal8_retry1' if name=='normal8' else name);rows=[json.loads(l) for l in (folder/'records.jsonl').open()];inputs=json.loads((folder/'inputs.json').read_text());ids=np.arange(w['score_indices'][0],w['score_indices'][1]+1);grid=h[ids]
        input_by_stamp={x['stamp_ns']:x for x in inputs};lid={r['stamp_ns']:r for r in rows if r['kind']=='lidar'};diag={r['stamp_ns']:r for r in rows if r['kind']=='diagnostic'}
        matched=[i for i in ids if int(h[i]) in lid];ix=np.array(matched);raw=np.array([lid[int(h[i])]['position'] for i in ix]);rawR=Rotation.from_quat([lid[int(h[i])]['quaternion'] for i in ix]).as_matrix()
        original=np.array([baseline[i]['branches'][0]['ndt']['final'] for i in ix]);m=dict(scored_inputs=len(ids),scored_lidar_outputs=len(ix),all_inputs=len(inputs),all_lidar_outputs=len(lid),missing_scored_indices=[int(i) for i in ids if int(h[i]) not in lid],registration_far_all_candidates=evaluate(raw,rawR,ix,ref,fixed['tangent']),registration_ndt_same_returned_indices=evaluate(original[:,:3,3],original[:,:3,:3],ix,ref,fixed['tangent']))
        ekf=[x for x in rows if x['kind']=='ekf'];P,R,ok,gap=interpolate([x['stamp_ns'] for x in ekf],[x['position'] for x in ekf],Rotation.from_quat([x['quaternion'] for x in ekf]).as_matrix(),grid)
        bsel=baseline[w['warmup_indices'][0]:w['score_indices'][1]+2];BP,BR,bok,bgap=interpolate([x['state_stamp'] for x in bsel],[np.array(x['pose'])[:3,3] for x in bsel],[np.array(x['pose'])[:3,:3] for x in bsel],grid)
        PP,PR,pok,pgap=interpolate([x['header_ns'] for x in production],[x['position'] for x in production],Rotation.from_quat([x['quaternion_xyzw'] for x in production]).as_matrix(),grid)
        common=ok&bok&(gap<100_000_000)&(bgap<200_000_000)
        m['delivered_state_common_grid']={'far':evaluate(P,R,ids,ref,fixed['tangent'],common),'15d':evaluate(BP,BR,ids,ref,fixed['tangent'],common),'common_samples':int(common.sum()),'contract':'Offline interpolate actual delivered states to same scan grid; not late scan-posterior versus real-time prior'}
        m['recorded_pose_stream_unverified_filter']=evaluate(PP,PR,ids,ref,fixed['tangent'],pok&(pgap<200_000_000));m['recorded_pose_contract']='kinematic_state output source/deployed binary/GNSS switch not independently identified; NOT a certified 6-state GNSS-off comparison'
        timing={'all':collections.defaultdict(list),'scored':collections.defaultdict(list)}
        for stamp_ns,r in lid.items():
            if stamp_ns not in input_by_stamp:continue
            i=input_by_stamp[stamp_ns];st=lookup.get(stamp_ns);wait=(r['received_wall_ns']-i['publish_wall_ns'])*1e-6;cost=wait+(i['publish_sim_ns']-i['available_ns'])*1e-6;age=cost+(i['available_ns']-stamp_ns)*1e-6
            sub=[x for x in ekf if x['received_wall_ns']>=r['received_wall_ns']];next_ms=(sub[0]['received_wall_ns']-r['received_wall_ns'])*1e-6 if sub else float('nan')
            for key in ['all']+(['scored'] if st in ids else []):
                for k,v in dict(after_input_available_to_lidar_ms=cost,header_to_lidar_delivery_ms=age,publish_to_lidar_delivery_ms=wait,prepare_publish_ms=i['prepare_publish_ms'],read_decode_ms=i['read_decode_ms'],fusion_ms=i['fusion_ms'],first_subsequent_ekf_delivery_proxy_ms=cost+next_ms).items():timing[key][k].append(v)
            dd=diag.get(stamp_ns,{}).get('status',[])
            if st in ids:
                e=np.array(r['position'])-ref['G_0.075'][st];cases.append(dict(window=name,index=int(st),stamp_ns=stamp_ns,xy_proxy=float(np.linalg.norm(e[:2])),diagnostic=' | '.join(x['message'] for x in dd),after_available_ms=cost,first_subsequent_state_ms=cost+next_ms))
        m['timing']={group:{k:stats(v) for k,v in d.items()} for group,d in timing.items()};m['over50_lidar']={group:sum(v>50 for v in d['after_input_available_to_lidar_ms']) for group,d in timing.items()};m['over50_state_proxy']={group:sum(v>50 for v in d['first_subsequent_ekf_delivery_proxy_ms']) for group,d in timing.items()}
        debug=[x['values'] for x in rows if x['kind']=='timing_debug' and x['node']=='LidarOdometry'];state=[x['values'] for x in rows if x['kind']=='timing_debug' and x['node']=='StateEstimation']
        m['native_components_ms']={k:stats([d[k] for d in debug if k in d]) for k in ['node/cloud_callback_time','pipeline/pipeline_time','registration/registration_time','distortion/undistortion_time','covariance/pose_covariance_time','covariance/twist_covariance_time','velocity/velocity_time']}
        m['backend_components_us']={k:stats([d[k] for d in state if k in d]) for k in ['runtime/step','runtime/delay_compensation','runtime/publish','runtime/callback_dt']}
        m['native_debug']=dict(callback_records=len(debug),converged=sum(d.get('registration/converged',0)>0 for d in debug),distortion_valid=sum(d.get('distortion/valid',0)>0 for d in debug),distortion_invalid_points=stats([d.get('distortion/num_invalid_points',0) for d in debug]),iterations=stats([d.get('registration/num_iter',0) for d in debug]),registration_point_count=stats([d.get('registration/num_points_frame',0) for d in debug]),statuses=dict(collections.Counter(s['message'] for r in rows if r['kind']=='diagnostic' for s in r['status'])))
        m['ndt_existing_source_n']=stats([baseline[i]['branches'][0]['ndt']['source_n'] for i in ids]);m['ndt_existing_converged']=sum(bool(baseline[i]['branches'][0]['ndt']['converged']) for i in ids)
        m['continuous_native_state']=dict(samples=len(ekf),stamp_gap_ms=stats(np.diff([x['stamp_ns'] for x in ekf])*1e-6),body_vz_mps=stats([x['velocity'][2] for x in ekf]))
        result['windows'][name]=m
    (out/'comparison.json').write_text(json.dumps(result,indent=2)+'\n');(out/'cases.json').write_text(json.dumps(cases,indent=2)+'\n');print(json.dumps({n:dict(delivered75={k:v['0.075']['xy']['rmse'] for k,v in x['delivered_state_common_grid'].items() if k in ['far','15d']},raw75=x['registration_far_all_candidates']['0.075']['xy'],timing=x['timing']['scored']['after_input_available_to_lidar_ms'],over50=x['over50_lidar']) for n,x in result['windows'].items()},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--be',required=True);p.add_argument('--data',required=True);main(p.parse_args())
