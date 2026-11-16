"""Recover aggregates from completed saved candidate matrices; no filter calls."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from r_scale_counterfactual import stats,encode

def main(out,be,data):
 out=Path(out);pre=json.load(open(out/'preregistration.json'));ref=np.load(Path(be)/'runtime/reference_lap.npz');ev=np.load(Path(data)/'phase_as_accuracy_quality/single_lap/evaluation.npz');rows=[];book=[];checks=dict(max_R5_saved_K_difference=0.,max_R5_saved_delta_difference=0.,max_gain_delta_identity_error=0.,min_P_eigen=1.,max_P_asymmetry=0.)
 for line in (out/'candidate_full_matrices.jsonl').open():
  c=json.loads(line);pr=c['prior'];ix=c['index'];seed=np.array(c['seed']);T=np.array(c['ndt']['final']);Rprior=np.array(pr['R']);lever=Rprior.T@(np.array(pr['p'])-seed[:3,3]);n=np.r_[-ev['tangent'][ix,1],ev['tangent'][ix,0],0];tan=ev['tangent'][ix];nu=np.array(c['innovation']);P=np.array(c['P_prior']);H=np.array(c['H']);sk=np.array([[0,-lever[2],lever[1]],[lever[2],0,-lever[0]],[-lever[1],lever[0],0]]);B=np.zeros((3,15));B[:,:3]=np.eye(3);B[:,6:9]=Rprior@sk
  v0=c['variants']['5'];checks['max_R5_saved_K_difference']=max(checks['max_R5_saved_K_difference'],float(np.max(abs(np.array(v0['K'])-np.array(c['update']['K'])))));checks['max_R5_saved_delta_difference']=max(checks['max_R5_saved_delta_difference'],float(np.max(abs(np.array(v0['delta'])-np.array(c['update'].get('delta',c['update']['delta_proposed']))))))
  item={k:c[k] for k in ['case_id','dataset','index','stamp','available','raw_identity','source_retention']};item.update(converged=c['ndt']['converged'],exit=c['ndt']['exit'],fitness=c['ndt']['fitness'],source_n=c['ndt']['source_n'],candidate_pairs=c['ndt']['candidate_pairs'],observation_id=c['ndt']['case_id'],support_inliers=c['ndt']['inliers_at_float_pose'],followup_evidence='raw C1/BE saved subsequent records; no independent ground truth certification',variants={})
  for mul in pre['R_multipliers']:
   v=c['variants'][str(mul)];K=np.array(v['K']);d=np.array(v['delta']);post=np.array(v['P_candidate']);R=np.array(v['R']);S=np.array(v['S']);eig=np.linalg.eigvalsh(post);checks['min_P_eigen']=min(checks['min_P_eigen'],float(eig.min()));checks['max_P_asymmetry']=max(checks['max_P_asymmetry'],float(np.max(abs(post-post.T))));checks['max_gain_delta_identity_error']=max(checks['max_gain_delta_identity_error'],float(np.max(abs(K@nu-d))));pos=np.array(v['candidate_position_base']);rot=np.array(v['candidate_rotation']);yaw=lambda R:np.arctan2(R[1,0],R[0,0]);wrap=lambda x:np.arctan2(np.sin(x),np.cos(x));yd=wrap(yaw(rot)-yaw(Rprior))
   m=dict(case_id=c['case_id'],dataset=c['dataset'],index=ix,groups=c['groups'],multiplier=mul,would_admit=v['would_admit'],reason=v['reason'],committed=False,delta_lateral=float(n@(pos-seed[:3,3])),K_lateral=float(n@B@K[:,:3]@n),yaw_delta_deg=float(np.rad2deg(yd)),velocity_delta_norm=float(np.linalg.norm(d[3:6])),rotation_delta_deg=float(np.rad2deg(np.linalg.norm(d[6:9]))),bg_delta_norm=float(np.linalg.norm(d[9:12])),ba_delta_norm=float(np.linalg.norm(d[12:15])),P_min_eigen=float(eig.min()),P_trace=float(np.trace(post)),R_lateral=float(n@R[:3,:3]@n),NIS=float(nu@np.linalg.solve(S,nu)),reference={})
   for shift in ['0.0','0.075','0.15']:
    G=ref['G_'+shift][ix];valid=bool(ref['raw_valid_'+shift][ix]) and bool(np.isfinite(G).all());z={'reference_valid':valid}
    if valid:
     ep=seed[:3,3]-G;eo=T[:3,3]-G;e=pos-G;z.update(prior_lateral=float(n@ep),ndt_lateral=float(n@eo),candidate_lateral=float(n@e),candidate_longitudinal=float(tan@e[:2]),candidate_xy=float(np.linalg.norm(e[:2])),ndt_xy=float(np.linalg.norm(eo[:2])),prior_xy=float(np.linalg.norm(ep[:2])),candidate_yaw_deg=float(np.rad2deg(wrap(yaw(rot)-yaw(ref['R'][ix])))))
    m['reference'][shift]=z
   rows.append(m);item['variants'][str(mul)]=m
  book.append(item)
 result=dict(scope='fixed-prior candidates, zero feedback/commit; no claims of calibrated R or closed-loop accuracy',cases=len(book),candidates=len(rows),checks=checks,groups={})
 for group in list(pre['C1_cohorts'])+['BE_saved']:
  result['groups'][group]={}
  for mul in pre['R_multipliers']:
   q=[z for z in rows if group in z['groups'] and z['multiplier']==mul];z=dict(count=len(q),would_admit=sum(v['would_admit'] for v in q),reference={})
   for k in ['delta_lateral','K_lateral','yaw_delta_deg','velocity_delta_norm','rotation_delta_deg','bg_delta_norm','ba_delta_norm','P_min_eigen','P_trace','NIS']:z[k]=stats([v[k] for v in q])
   for shift in ['0.0','0.075','0.15']:
    v=[v['reference'][shift] for v in q if v['reference'][shift]['reference_valid']];zz=dict(valid_count=len(v))
    for k in ['prior_lateral','ndt_lateral','candidate_lateral','candidate_longitudinal','candidate_xy','ndt_xy','prior_xy','candidate_yaw_deg']:zz[k]=stats([x[k] for x in v])
    zz['candidate_xy_better_than_prior']=sum(x['candidate_xy']<x['prior_xy'] for x in v);zz['proxy_good_ndt_count']=sum(x['ndt_xy']<=.5 for x in v);zz['proxy_bad_ndt_count']=sum(x['ndt_xy']>=2 for x in v);z['reference'][shift]=zz
   result['groups'][group][str(mul)]=z
 assert len(rows)==891 and checks['min_P_eigen']>0 and checks['max_R5_saved_K_difference']<1e-9 and checks['max_R5_saved_delta_difference']<1e-9
 (out/'scalar_candidates.json').write_text(json.dumps(rows,indent=2,default=encode));(out/'casebook_ledger.json').write_text(json.dumps(book,indent=2,default=encode));(out/'r_scale_aggregate.json').write_text(json.dumps(result,indent=2,default=encode));print(json.dumps(checks))
if __name__=='__main__':main(*sys.argv[1:])
