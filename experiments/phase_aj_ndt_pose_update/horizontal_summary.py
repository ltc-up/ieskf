import json,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
import argparse,os
p=argparse.ArgumentParser();p.add_argument('--data-root',default=os.environ.get('IESKF_DATA_ROOT'));p.add_argument('--public-output',required=True);args=p.parse_args();assert args.data_root;root=Path(args.data_root);E=Path(args.public_output);D=root/'phase_aj_ndt_pose_update';paths=dict(A=str(root/'history/phase_v/results/replay_inertial_wheel_control_continuous_extension.json'),B=str(root/'phase_aa_z_validation/new_replay.json'),C=str(D/'C_replay_private.json'));q=json.load(open(E/'key_results.json'));series={}
for k,path in paths.items():
 b=json.load(open(path));xyz=np.array([s['p_imu_map'] for s in b['outputs']]);vv=np.array([s['v_imu_map'] for s in b['outputs']]);yaw=Rotation.from_quat([s['q_map_body_xyzw'] for s in b['outputs']]).as_euler('xyz')[:,2];d=np.diff(yaw);d=np.arctan2(np.sin(d),np.cos(d));series[k]=(xyz,yaw)
 q['branches'][k]['output_XY_increment_norm_m']=dict(zip(['P50','P95','P99','Max'],map(float,np.percentile(np.linalg.norm(np.diff(xyz[:,:2],axis=0),axis=1),[50,95,99,100]))));q['branches'][k]['output_abs_yaw_increment_rad']=dict(zip(['P50','P95','P99','Max'],map(float,np.percentile(abs(d),[50,95,99,100]))));q['branches'][k]['horizontal_speed_minmax_m_s']=[float(np.linalg.norm(vv[:,:2],axis=1).min()),float(np.linalg.norm(vv[:,:2],axis=1).max())]
q['cross_branch_disagreement_not_accuracy']={}
for k in ['A','B']:
 a,y=series[k];c,z=series['C'];d=np.arctan2(np.sin(y-z),np.cos(y-z));q['cross_branch_disagreement_not_accuracy']['C_vs_'+k]=dict(end_XY_distance_m=float(np.linalg.norm(a[-1,:2]-c[-1,:2])),max_XY_distance_m=float(np.linalg.norm(a[:,:2]-c[:,:2],axis=1).max()),end_abs_yaw_difference_rad=float(abs(d[-1])),max_abs_yaw_difference_rad=float(abs(d).max()))
(E/'key_results.json').write_text(json.dumps(q,indent=2)+'\n');(D/'horizontal_summary.json').write_text(json.dumps({k:v for k,v in q.items() if k in ['branches','cross_branch_disagreement_not_accuracy']},indent=2)+'\n');print(q['cross_branch_disagreement_not_accuracy'])
