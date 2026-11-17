"""Uncalibrated conditional curvature algebra only; no state commit or scale fitting."""
import sys,json
from pathlib import Path
import numpy as np
REPO=Path(__file__).resolve().parents[2];sys.path.insert(0,str(REPO/'src'))
from geometry_ieskf.state import State
from geometry_ieskf.external_pose import ExternalPose
from geometry_ieskf.common import skew
root=Path(sys.argv[1]);cfg=json.load(open(root/'inputs/series.json'));records=cfg['series'][0]['records'];Tbi=np.array(cfg['T_base_imu']);Rb=Tbi[:3,:3];r=Tbi[:3,3];diag={x['id']:x for x in map(json.loads,open(root/'results/raw_geometry.jsonl'))};out=[]
for i,z in enumerate(records):
 id=f'causal_{i}_map1013'
 if id not in diag:continue
 if 'R'not in z['legacy_update']:
  out.append(dict(id=id,status='NO_PAIRED_UPDATE_R',reason=z['legacy_update'].get('reason','UNKNOWN'),no_commit=True));continue
 pr=z['prior'];x=State(np.array(pr['p']),np.array(pr['v']),np.array(pr['R']),np.array(pr['bg']),np.array(pr['ba']),z['t']);P=np.array(pr['P']);Hgeom=np.array(diag[id]['native_augmented_H'])[:6,:6];M=np.zeros((6,6));M[:3,:3]=x.R@skew(r)@Rb;M[:3,3:]=x.R@Rb;M[3:,:3]=Rb;obs=ExternalPose(r,('x','y','z','rx','ry','rz'));Rlegacy=np.array(z['legacy_update']['R']);Rcurv=M@np.linalg.solve(Hgeom,M.T);Rcurv=(Rcurv+Rcurv.T)/2
 row=dict(id=id,calibrated=False,identity_t=z['t'],geometry_at='same frozen causal prior',models={})
 for name,R in [('Legacy',Rlegacy),('conditional_inverse_curvature_NOT_measurement_covariance',Rcurv)]:
  yy,Q,info=obs.correct(x,P,np.array(z['legacy']['final']),R);row['models'][name]=dict(R=R.tolist(),R_eigen=np.linalg.eigvalsh(R).tolist(),P_posterior_candidate=Q.tolist(),H=obs.residual(x,np.array(z['legacy']['final']))[1].tolist(),update=info,no_commit=True)
 out.append(row)
(root/'results/conditional_R_candidates.json').write_text(json.dumps(out,indent=2));print(len(out),'conditional curvature algebra rows; no calibrated R, no online update')
