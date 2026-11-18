"""Frozen cross-backend and next-scan evaluation; engineering proxy labels only."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from runtime import Validator,read_cloud,partition
from evaluate import stats
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--requests',type=Path,required=True);p.add_argument('--results',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
rows=[json.loads(s) for s in a.results.open()];requests=json.load(open(a.requests));ref=np.load(a.root/'runtime/reference_lap.npz');N=np.asarray([r['ndt']['final'] for r in rows]);G=np.asarray([r['gicp']['final'] for r in rows]);diff=np.linalg.norm(N[:,:2,3]-G[:,:2,3],axis=1);yaw=np.array([Rotation.from_matrix(n[:3,:3].T@g[:3,:3]).magnitude() for n,g in zip(N,G)]);distinct=(diff>.1)|(yaw>.01)
summary=dict(frames=len(rows),same_input_contract='identical fit Source/Target/seed/header per paired request; different local objectives/criteria',distinct=int(distinct.sum()),translation_difference=stats(diff),rotation_difference_rad=stats(yaw),NDT_converged=sum(r['ndt']['converged'] for r in rows),GICP_converged=sum(r['gicp']['converged'] for r in rows),sensitivity={},cycle_ms=stats([r['cycle_ms'] for r in rows]),ndt_service_ms=stats([r['ndt_ms'] for r in rows]),gicp_service_ms=stats([r['gicp_ms'] for r in rows]))
summary['gicp_components_ms']={k:stats([r['gicp'][k] for r in rows]) for k in ['source_tree_ms','source_covariance_ms','align_wall_ms','information_refresh_ms','target_build_ms','service_wall_ms']}
summary['ndt_components_ms']={k:stats([r['ndt'][k] for r in rows]) for k in ['align_internal_ms','align_wall_ms','quality_extra_ms','target_build_ms']}
nv=np.array([r['ndt']['converged'] and r['ndt'].get('candidate_valid',False) for r in rows]);gv=np.array([r['gicp']['converged'] and r['gicp'].get('candidate_valid',False) for r in rows]);summary['eligible_candidates']={'ndt':int(nv.sum()),'gicp':int(gv.sum())};summary['raw_choice_contract']='original common-cost diagnostic includes raw nonconverged finals; eligible_choice additionally enforces native convergence/candidate flags'
for delay in ['0.0','0.075','0.15']:
 valid=ref['raw_valid_'+delay];truth=ref['G_'+delay];ne=np.linalg.norm(N[:,:2,3]-truth[:,:2],axis=1);ge=np.linalg.norm(G[:,:2,3]-truth[:,:2],axis=1);ng=ne<=.5;gg=ge<=.5;nb=ne>=2.;gb=ge>=2.;choice=np.array([r['gicp_validation']['cost']<r['ndt_validation']['cost'] for r in rows]);summary['sensitivity'][delay]=dict(proxy_only=True,valid=int(valid.sum()),ndt_error=stats(ne[valid]),gicp_error=stats(ge[valid]),NDT_good_GICP_bad=int((valid&ng&gb).sum()),GICP_good_NDT_bad=int((valid&gg&nb).sum()),both_good=int((valid&ng&gg).sum()),both_bad=int((valid&nb&gb).sum()),validation_chooses_GICP=int((valid&choice).sum()),proxy_error_of_validation_choice=stats(np.where(choice,ge,ne)[valid]))
for delay in summary['sensitivity']:
 valid=ref['raw_valid_'+delay];truth=ref['G_'+delay];ne=np.linalg.norm(N[:,:2,3]-truth[:,:2],axis=1);ge=np.linalg.norm(G[:,:2,3]-truth[:,:2],axis=1);use=gv&(~nv|choice);available=valid&(nv|gv);summary['sensitivity'][delay]['eligible_GICP_choices']=int((available&use).sum());summary['sensitivity'][delay]['eligible_choice_error']=stats(np.where(use,ge,ne)[available]);summary['sensitivity'][delay]['invalid_GICP_lower_cost']=int((valid&choice&~gv).sum())
val=Validator(requests[0]['target_path']);future=[]
for i in range(len(rows)-1):
 _,held=partition(read_cloud(requests[i+1]['source_path']));A=np.asarray(requests[i]['seed_common']);B=np.asarray(requests[i+1]['seed_common']);delta=np.linalg.inv(A)@B
 nc=val.cost(held,N[i]@delta);gc=val.cost(held,G[i]@delta);future.append(dict(index=i,next_scan_ndt=nc,next_scan_gicp=gc,current_ndt=rows[i]['ndt_validation'],current_gicp=rows[i]['gicp_validation']))
(a.out/'next_scan_validation.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in future));summary['next_scan_GICP_lower_cost']=sum(r['next_scan_gicp']['cost']<r['next_scan_ndt']['cost'] for r in future);summary['next_scan_contract']='next held-out scan transported by frozen seed-relative motion; correlated map/odometry, not independent truth'
(a.out/'frozen_summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
