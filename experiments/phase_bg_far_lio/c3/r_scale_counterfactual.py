"""BG-C3 fixed-prior algebra only. Never predict, replay, match or commit state.
All full-state records are private; only non-reconstructive summaries are public.
"""
import argparse, hashlib, json, sys, time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src'))
from geometry_ieskf.state import State, chart_map
from geometry_ieskf.external_pose import ExternalPose

def read(p):
    with Path(p).open() as f:
        for l in f: yield json.loads(l)
def stats(v):
    a=np.asarray(v,float);a=a[np.isfinite(a)]
    if not len(a):return {'n':0}
    return dict(n=len(a),mean=float(a.mean()),p50_abs=float(np.median(abs(a))),p95_abs=float(np.percentile(abs(a),95)),max_abs=float(max(abs(a))),rmse=float(np.sqrt(np.mean(a*a))))
def encode(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    raise TypeError(type(x).__name__)
def main(a):
    out=Path(a.out);pre=json.loads((out/'preregistration.json').read_text());dest=out/'candidate_full_matrices.jsonl'
    if dest.exists():raise FileExistsError(dest)
    be=Path(a.be);ref=np.load(be/'runtime/reference_lap.npz');ev=np.load(Path(a.data)/'phase_as_accuracy_quality/single_lap/evaluation.npz');lookup={int(t):i for i,t in enumerate(ref['header'])};selected=set(sum(pre['C1_cohorts'].values(),[]));cases=[]
    for r in read(Path(a.c1)/'eskf15_recorded_init/matches.jsonl'):
        ix=lookup.get(r['stamp_ns'])
        if ix not in selected or 'ndt' not in r:continue
        cases.append(dict(case_id='C1:'+str(ix),dataset='C1',index=ix,stamp=r['stamp_ns'],available=r['available_ns'],prior=r['prior'],seed=r['seed'],ndt=r['ndt'],update=r['update'],R0=r['update']['R'],groups=[k for k,v in pre['C1_cohorts'].items() if ix in v],raw_identity=r['input_meta']['raw_identities'],source_retention='C1 active source scratch overwritten; raw CDR identities and deskew metadata retained; no source reconstruction',deskew=r['deskew']))
    for s in pre['BE_saved_case_records']:
        r=next(r for r in read(be/'results_be5_stage2a'/s['folder']/'frames.jsonl') if r['case_id']==s['case_id'])
        cases.append(dict(case_id='BE:'+s['case_id'],dataset='BE_saved',index=r['index'],stamp=r['header_ns'],available=None,prior=r['prior'],seed=r['seed'],ndt=r['original_ndt'],update=r['original_update'],R0=r['original_R'],groups=['BE_saved'],raw_identity=r['saved_row_sha256'],source_retention='original BE cleanup mapping; no point cloud read or restoration',deskew=None,old_P_candidate=r['Legacy_candidate']['P_candidate']))
    assert len(cases)==pre['max_unique_cases'];rows=[];checks={'max_R5_saved_K_difference':0.,'max_R5_saved_delta_difference':0.,'max_R5_BE_saved_P_difference':0.,'max_gain_delta_identity_error':0.,'max_Joseph_reset_P_difference':0.,'max_original_R_formula_difference':0.}
    with dest.open('x') as stream:
        for c in cases:
            pr=c['prior'];x=State(*[np.array(pr[k],float) for k in ['p','v','R','bg','ba']],c['stamp']);P=np.array(pr['P']);seed=np.array(c['seed']);T=np.array(c['ndt']['final']);lever=x.R.T@(x.p-seed[:3,3]);ob=ExternalPose(lever,tuple(c['update']['components']));nu,H=ob.residual(x,T);R0=np.array(c['R0']);f=max(float(c['ndt']['fitness']),1e-5);xy,ang=(1.5*f,.6*f) if f>=3 else (.05*f,.00025*f);expected=np.diag([5*xy]*3+[5*ang]*3);checks['max_original_R_formula_difference']=max(checks['max_original_R_formula_difference'],float(np.max(abs(expected-R0))))
            ix=c['index'];tan=ev['tangent'][ix];n=np.r_[-tan[1],tan[0],0.];B=np.zeros((3,15));B[:,:3]=np.eye(3);sk=np.array([[0,-lever[2],lever[1]],[lever[2],0,-lever[0]],[-lever[1],lever[0],0]]);B[:,6:9]=x.R@sk
            item={k:v for k,v in c.items() if k not in ['old_P_candidate']};item.update(H=H,innovation=nu,P_prior=P,variants={})
            for mul in pre['R_multipliers']:
                tick=time.perf_counter_ns();R=R0*(mul/5);S=H@P@H.T+R;K=np.linalg.solve(S,H@P).T;d=K@nu;A=np.eye(15)-K@H;Pc=A@P@A.T+K@R@K.T;D=chart_map(d);post=D@Pc@D.T;post=(post+post.T)/2;y,Q,info=ob.correct(x,P,T,R);elapsed=(time.perf_counter_ns()-tick)/1e6
                if mul==5:
                    for key,v in [('max_R5_saved_K_difference',np.max(abs(K-np.array(c['update']['K'])))),('max_R5_saved_delta_difference',np.max(abs(d-np.array(c['update'].get('delta',c['update']['delta_proposed'])))))]:checks[key]=max(checks[key],float(v))
                    if 'old_P_candidate' in c:checks['max_R5_BE_saved_P_difference']=max(checks['max_R5_BE_saved_P_difference'],float(np.max(abs(Q-np.array(c['old_P_candidate'])))))
                checks['max_gain_delta_identity_error']=max(checks['max_gain_delta_identity_error'],float(np.max(abs(K@nu-d))))
                if info['accepted']:checks['max_Joseph_reset_P_difference']=max(checks['max_Joseph_reset_P_difference'],float(np.max(abs(post-Q))))
                eig=np.linalg.eigvalsh(post);assert eig.min()>0 and np.isfinite(post).all();candidate_position=x.p+d[:3]-x.R@Rotation.from_rotvec(d[6:9]).as_matrix()@lever;candidate_rotation=x.R@Rotation.from_rotvec(d[6:9]).as_matrix();yaw=lambda R:np.arctan2(R[1,0],R[0,0]);yaw_delta=np.arctan2(np.sin(yaw(candidate_rotation)-yaw(x.R)),np.cos(yaw(candidate_rotation)-yaw(x.R)))
                metrics=dict(case_id=c['case_id'],dataset=c['dataset'],index=ix,groups=c['groups'],multiplier=mul,would_admit=bool(info['accepted']),reason=info['reason'],committed=False,delta_lateral=float(n@(candidate_position-seed[:3,3])),K_lateral=float(n@B@K[:,:3]@n),yaw_delta_deg=float(np.rad2deg(yaw_delta)),velocity_delta_norm=float(np.linalg.norm(d[3:6])),rotation_delta_deg=float(np.rad2deg(np.linalg.norm(d[6:9]))),bg_delta_norm=float(np.linalg.norm(d[9:12])),ba_delta_norm=float(np.linalg.norm(d[12:15])),P_min_eigen=float(eig.min()),P_trace=float(np.trace(post)),R_lateral=float(n@R[:3,:3]@n),NIS=float(nu@np.linalg.solve(S,nu)),algebra_wall_ms=elapsed,reference={})
                for shift in ['0.0','0.075','0.15']:
                    G=ref['G_'+shift][ix];valid=bool(ref['raw_valid_'+shift][ix]) and np.isfinite(G).all();v=dict(reference_valid=valid)
                    if valid:
                        prior_error=seed[:3,3]-G;obs_error=T[:3,3]-G;e=candidate_position-G;v.update(prior_lateral=float(n@prior_error),ndt_lateral=float(n@obs_error),candidate_lateral=float(n@e),candidate_longitudinal=float(tan@e[:2]),candidate_xy=float(np.linalg.norm(e[:2])),ndt_xy=float(np.linalg.norm(obs_error[:2])),prior_xy=float(np.linalg.norm(prior_error[:2])))
                    metrics['reference'][shift]=v
                rows.append(metrics);item['variants'][str(mul)]=dict(R=R,S=S,K=K,delta=d,P_before_reset=Pc,P_candidate=post,P_change=post-P,would_admit=info['accepted'],reason=info['reason'],candidate_position_base=candidate_position,candidate_rotation=candidate_rotation,committed=False)
            stream.write(json.dumps(item,default=encode)+'\n');assert dest.stat().st_size<pre['budget_bytes']
    # Abort attribution on identity differences; no tuning or substitute prior permitted.
    assert checks['max_R5_saved_K_difference']<1e-9 and checks['max_R5_saved_delta_difference']<1e-9 and checks['max_R5_BE_saved_P_difference']<1e-9,checks
    result=dict(scope='fixed-prior candidates only; no propagation/feedback/commit; engineering reference not truth',case_count=len(cases),candidate_count=len(rows),checks=checks,groups={})
    for group in list(pre['C1_cohorts'])+['BE_saved']:
        result['groups'][group]={}
        for mul in pre['R_multipliers']:
            q=[z for z in rows if group in z['groups'] and z['multiplier']==mul];z={'count':len(q),'would_admit':sum(v['would_admit'] for v in q)}
            for k in ['delta_lateral','K_lateral','yaw_delta_deg','velocity_delta_norm','rotation_delta_deg','bg_delta_norm','ba_delta_norm','P_min_eigen','P_trace','NIS','algebra_wall_ms']:z[k]=stats([v[k] for v in q])
            z['reference']={}
            for shift in ['0.0','0.075','0.15']:
                v=[v['reference'][shift] for v in q if v['reference'][shift]['reference_valid']];zz={'valid_count':len(v)}
                for k in ['prior_lateral','ndt_lateral','candidate_lateral','candidate_longitudinal','candidate_xy','ndt_xy','prior_xy']:zz[k]=stats([x[k] for x in v])
                zz['candidate_xy_better_than_prior']=sum(x['candidate_xy']<x['prior_xy'] for x in v);zz['proxy_good_ndt_count']=sum(x['ndt_xy']<=.5 for x in v);zz['proxy_bad_ndt_count']=sum(x['ndt_xy']>=2 for x in v);z['reference'][shift]=zz
            result['groups'][group][str(mul)]=z
    (out/'scalar_candidates.json').write_text(json.dumps(rows,indent=2,default=encode));(out/'r_scale_aggregate.json').write_text(json.dumps(result,indent=2,default=encode));print(json.dumps({'cases':len(cases),'candidates':len(rows),'checks':checks,'full_matrix_bytes':dest.stat().st_size}))
if __name__=='__main__':
    p=argparse.ArgumentParser();[p.add_argument('--'+n,required=True) for n in ['out','c1','be','data']];main(p.parse_args())
