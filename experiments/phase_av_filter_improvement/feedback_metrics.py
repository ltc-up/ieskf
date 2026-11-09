"""Per-fixed-corner full-state, rear-reference feedback diagnostics; no reference tuning."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.state import State
from geometry_ieskf.external_pose import ExternalPose
from ndt_global.validation import static_transform
from evaluate import stats
p=argparse.ArgumentParser()
for k in ['run','motion','as-evaluation','output']:p.add_argument('--'+k,type=Path,required=True)
a=p.parse_args();tx=json.loads((a.run/'transactions.json').read_text());corner=np.load(a.as_evaluation)['corner'][:len(tx)];lever=static_transform(json.loads(a.motion.read_text())['transforms'],'base_link','imu_bosch')[:3,3];records=[]
for r,c in zip(tx,corner):
 if not r['update']['accepted']:continue
 u=r['update'];x=State(np.array(r['prior']['p']),np.zeros(3),np.array(r['prior']['R']),np.zeros(3),np.zeros(3));res,H=ExternalPose(lever,('x','y','z','rx','ry','rz')).residual(x,np.array(r['ndt']['final']));lat=x.R[:,1];e=np.r_[lat,[0.,0.,0.]];W=np.asarray(r['ndt'].get('av_projection',{}).get('matrix',np.eye(6)));K=np.asarray(u['K']);d=np.asarray(u['delta']);Rpost=x.R@Rotation.from_rotvec(d[6:9]).as_matrix();base_delta=d[:3]-(Rpost-x.R)@lever;v=dict(corner=int(c),gain=float(e@H@K@W@e),innovation=float(lat@res[:3]),actual_update=float(lat@base_delta),speed_change=float(np.linalg.norm(d[3:6])),yaw_local_delta=float(d[8]),gyro_bias_change=float(np.linalg.norm(d[9:12])),accel_bias_change=float(np.linalg.norm(d[12:15])),rank=len(W),prior_lateral_std=float(np.sqrt(e@H@np.array(r['prior']['P'])@H.T@e)))
 if len(W)==6:v['packet_lateral_std']=float(np.sqrt(e@np.asarray(u['R'])@e))
 if 'expected_precision' in u:v['precision']=u['expected_precision'];v['nonlinear_derivative']=float(e@H@np.asarray(u['local_residual_jacobian'])@W@e)
 records.append(v)
out={}
for c in sorted(set(corner)):
 rows=[v for v in records if v['corner']==c];out[str(int(c))]=dict(accepted=len(rows),**{k:stats([v[k] for v in rows if k in v]) for k in records[0] if k!='corner'})
a.output.write_text(json.dumps(dict(fixed_AS_corners=out,conditional_model_not_calibration=True),indent=2)+'\n')
