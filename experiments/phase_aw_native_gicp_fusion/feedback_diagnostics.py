"""Full-state update/correlation and actual component timing aggregates."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.external_pose import ExternalPose
from geometry_ieskf.state import State
from ndt_global.validation import static_transform
from evaluate import stats
p=argparse.ArgumentParser()
for k in ['run','motion','as-evaluation','output']:p.add_argument('--'+k,type=Path,required=True)
a=p.parse_args();rows=json.loads((a.run/'transactions.json').read_text());states=json.loads((a.run/'states.json').read_text());corner=np.load(a.as_evaluation)['corner'][:len(rows)];lever=static_transform(json.loads(a.motion.read_text())['transforms'],'base_link','imu_bosch')[:3,3];record=[];prev=None;lagleft=[];lagright=[];solver=[]
for r,k in zip(rows,corner):
 x=State(np.array(r['prior']['p']),np.zeros(3),np.array(r['prior']['R']),np.zeros(3),np.zeros(3));res,H=ExternalPose(lever,('x','y','z','rx','ry','rz')).residual(x,np.array(r['ndt']['final']));e=np.r_[x.R[:,1],np.zeros(3)];lat=float(e@res);v=dict(corner=int(k),lateral_innovation=lat)
 if prev is not None and (r['ref']-prev[0])*1e-9<.06:lagleft.append(prev[1]);lagright.append(lat)
 prev=(r['ref'],lat)
 if r['update']['accepted']:
  u=r['update'];d=np.array(u['delta']);K=np.array(u['K']);P=np.array(r['prior']['P']);R=np.array(u['R']);Qrot=x.R@Rotation.from_rotvec(d[6:9]).as_matrix();dp=d[:3]-(Qrot-x.R)@lever;v.update(unit_lateral_gain=float(e@H@K@e),actual_lateral_update=float(x.R[:,1]@dp),prior_lateral_std=float(np.sqrt(e@H@P@H.T@e)),R_lateral_std=float(np.sqrt(e@R@e)),delta_Kr_error=float(np.max(abs(d-K@u['innovation']))))
 record.append(v)
bycorner={str(int(k)):{f:stats([v[f] for v in record if v['corner']==k and f in v]) for f in ['lateral_innovation','unit_lateral_gain','actual_lateral_update','prior_lateral_std','R_lateral_std']} for k in sorted(set(corner))};meta=[json.loads(l) for l in (a.run/'deskew_meta_stream.jsonl').read_text().splitlines() if l.strip()];timing={k:stats([v[k] for v in meta]) for k in ['gpu_deskew_ms','crop_voxel_sample_ms','rebase_with_file_write_ms']};timing.update({k:stats([v['ndt'][k] for v in rows if k in v['ndt']]) for k in ['source_tree_ms','source_covariance_ms','align_wall_ms','information_refresh_ms','service_wall_ms']});age=np.array([(r['complete']-r['ref'])*1e-6 for r in rows]);queue=np.array([max(0,(r['complete']-r['available'])*1e-6) for r in rows]);timing['model_observation_age_ms']=stats(age);timing['complete_minus_arrival_including_compute_ms']=stats(queue);Ps=np.array([s['P'] for s in states]);out=dict(frames=len(rows),fixed_corner_feedback=bycorner,lateral_innovation_lag1_correlation=float(np.corrcoef(lagleft,lagright)[0,1]),adjacent_cloud_errors_independent='NOT_VALIDATED; innovation correlation alone does not identify source-noise correlation',delta_Kr_max_error=max(v.get('delta_Kr_error',0) for v in record),timing=timing,model_age_over500ms=int(sum(age>500)),Pzz=stats(Ps[:,2,2]),P_min_eigenvalue=float(np.linalg.eigvalsh(Ps)[:,0].min()),statistical_consistency=False);a.output.write_text(json.dumps(out,indent=2)+'\n')
