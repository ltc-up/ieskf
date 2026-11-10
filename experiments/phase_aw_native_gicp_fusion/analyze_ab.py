"""Geometry/pose comparison of all actual same-input matches; aggregate outputs only."""
import argparse,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import spearmanr
from ndt_global.matcher_geometry import ndt_native_information,gicp_information,describe,compare
from ndt_global.track_evaluation import paired_errors
from evaluate import reference,stats
p=argparse.ArgumentParser()
for k in ['run','as-evaluation','at-bindings','output']:p.add_argument('--'+k,type=Path,required=True)
p.add_argument('--reference-cache',type=Path);a=p.parse_args();rows=[json.loads(l) for l in (a.run/'matches.jsonl').read_text().splitlines()];f=np.load(a.as_evaluation);corner=f['corner'][:len(rows)];tan=f['tangent'][:len(rows)];ns=np.array([r['ref'] for r in rows],np.int64);refs=reference(json.loads(a.at_bindings.read_text()),ns,a.reference_cache);N=np.array([r['ndt']['final'] for r in rows]);G=np.array([r['gicp']['final'] for r in rows]);seed=np.array([r['seed'] for r in rows]);rot=seed[:,:3,:3];rel=np.einsum('nji,nj->ni',rot,(G-N)[:,:3,3]);rv=Rotation.from_matrix(N[:,:3,:3].transpose(0,2,1)@G[:,:3,:3]).as_rotvec();valid=np.array([r['gicp']['candidate_valid'] and r['gicp']['converged'] for r in rows]);summaries={};geometry=[];desc=[]
for r in rows:
 anchor=Rotation.from_matrix(np.array(r['seed'])[:3,:3]).as_matrix();A=ndt_native_information(r['ndt'],anchor_rotation=anchor);B=gicp_information(r['gicp'],anchor_rotation=anchor);c=compare(A,B);x=describe(A);y=describe(B);geometry.append(dict(shape=c['trace_shape_cosine'],weak=c['weakest_axis_abs_cosine'],subspace_min=float(c['weakest2_subspace_cosines'].min()),ndt_condition=x['condition'],gicp_condition=y['condition'],ndt_lat_yaw_coupling=float(x['normalized_coupling'][1,5]),gicp_lat_yaw_coupling=float(y['normalized_coupling'][1,5])));desc.append((x,y))
for delay,ref in refs.items():
 ok=ref['historical_valid']&valid;result={}
 for method,P in [('NDT',N[:,:3,3]),('GICP',G[:,:3,3])]:
  es,ed=paired_errors(P,ref['G'],tan);dy=Rotation.from_matrix((N if method=='NDT' else G)[:,:3,:3].transpose(0,2,1)@ref['R']).as_rotvec()[:,2];result[method]=dict(coverage=float(ok.mean()),valid=int(ok.sum()),XY=stats(np.linalg.norm((P-ref['G'])[ok,:2],axis=1)),abs_longitudinal=stats(abs(es[ok])),abs_lateral=stats(abs(ed[ok])),abs_yaw_local_deg=stats(abs(np.rad2deg(dy[ok]))),corners={str(int(k)):dict(XY=stats(np.linalg.norm((P-ref['G'])[ok&(corner==k),:2],axis=1)),abs_lateral=stats(abs(ed[ok&(corner==k)])),abs_longitudinal=stats(abs(es[ok&(corner==k)]))) for k in set(corner)})
 for method,P in [('NDT',N[:,:3,3]),('GICP',G[:,:3,3])]:
  own=ref['historical_valid']&(valid if method=='GICP' else np.array([r['ndt']['candidate_valid'] and r['ndt']['converged'] for r in rows]));result[method]['own_valid_samples']=int(own.sum());result[method]['own_valid_XY']=stats(np.linalg.norm((P-ref['G'])[own,:2],axis=1))
 summaries[delay]=result
bycorner={}
for k in sorted(set(corner)):
 ids=np.flatnonzero(corner==k);bycorner[str(int(k))]=dict(samples=len(ids),**{field:stats([geometry[i][field] for i in ids]) for field in geometry[0]},ndt_conditional_energy_mean=np.mean([desc[i][0]['conditional_axis_energy'] for i in ids],axis=0).tolist(),gicp_conditional_energy_mean=np.mean([desc[i][1]['conditional_axis_energy'] for i in ids],axis=0).tolist(),ndt_marginal_translation_mean=np.mean([desc[i][0]['marginal_translation_energy'] for i in ids],axis=0).tolist(),gicp_marginal_translation_mean=np.mean([desc[i][1]['marginal_translation_energy'] for i in ids],axis=0).tolist())
probes=[json.loads(l) for l in (a.run/'probes.jsonl').read_text().splitlines()];probe_summary=[]
for r in probes:
 out=dict(corner=r['corner'],index=r['index'])
 for method in ['ndt','gicp']:
  base=np.array(r['central_'+method]['final']);p0=[x[method] for x in r['probes']];out[method]=dict(converged=sum(x['converged'] for x in p0),candidate_valid=sum(x['candidate_valid'] for x in p0),separation_m=[float(np.linalg.norm(np.array(x['final'])[:3,3]-base[:3,3])) for x in p0],rotation_difference_rad=[float(np.linalg.norm(Rotation.from_matrix(base[:3,:3].T@np.array(x['final'])[:3,:3]).as_rotvec())) for x in p0])
 out['repeat_GICP_transform_max_difference']=float(np.max(abs(np.array(r['repeat_gicp']['final'])-np.array(r['central_gicp']['final']))));curves=r['central_gicp']['frozen_factor_curvature'];out['frozen_factor_curvature_relative_GN_differences']=[float(((v['plus']+v['minus']-2*v['base'])/(v['step']**2)-v['GN_diagonal'])/v['GN_diagonal']) for v in curves];probe_summary.append(out)
correlations={};ref=refs['0.075'];ok=ref['historical_valid']&valid
for j,method,P in [(0,'NDT',N[:,:3,3]),(1,'GICP',G[:,:3,3])]:
 _,ed=paired_errors(P,ref['G'],tan);energy=np.array([x[j]['marginal_translation_energy'][1] for x in desc]);correlations[method]=dict(marginal_lateral_energy_vs_abs_engineering_lateral_spearman=float(spearmanr(energy[ok],abs(ed[ok])).statistic),not_detection_accuracy=True)
out=dict(frames=len(rows),same_source_seed=True,actual_fresh_CUDA=True,actual_small_gicp=True,GICP_valid_converged=int(valid.sum()),NDT_valid_converged=sum(r['ndt']['candidate_valid'] and r['ndt']['converged'] for r in rows),relative_pose=dict(abs_body_longitudinal=stats(abs(rel[:,0])),abs_body_lateral=stats(abs(rel[:,1])),abs_local_yaw_deg=stats(abs(np.rad2deg(rv[:,2])))),reference_sensitivity=summaries,geometry_by_fixed_corner=bycorner,geometry_overall={k:stats([v[k] for v in geometry]) for k in geometry[0]},correlations=correlations,probes=probe_summary,performance={method:{field:stats([r[method][field] for r in rows if field in r[method]]) for field in ['align_wall_ms','source_tree_ms','source_covariance_ms','information_refresh_ms','protocol_wall_ms']} for method in ['ndt','gicp']},curvature_chart='[translation m,10*rotation rad] in the SAME frozen-seed axes at the base reference; trace shape only, not common noise scale',statistical_covariance=False)
a.output.write_text(json.dumps(out,indent=2)+'\n');np.savez_compressed(a.run/'geometry_analysis.npz',header=ns,corner=corner,ndt=N,gicp=G,relative_body=rel,ndt_info=np.array([v[0]['full_information'] for v in desc]),gicp_info=np.array([v[1]['full_information'] for v in desc]))
