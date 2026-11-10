"""Physical-input and frozen-segment diagnostics; never feeds a runtime policy."""
import argparse,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.state import State
from geometry_ieskf.optical_velocity import OpticalVelocity
from geometry_ieskf.optical_calibration import CalibratedOptical,lift
from ndt_global.validation import static_transform
from ndt_global.track_evaluation import paired_errors
sys.path.insert(0,str(Path(__file__).parents[1]/'phase_aw_native_gicp_fusion'));from evaluate import stats
p=argparse.ArgumentParser();p.add_argument('--private-root',type=Path,required=True);p.add_argument('--motion',type=Path,required=True);p.add_argument('--baseline-priors',type=Path,required=True);p.add_argument('--as-evaluation',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();motion=json.loads(a.motion.read_text());Rb=static_transform(motion['transforms'],'base_link','imu_bosch')[:3,:3];lever=static_transform(motion['transforms'],'base_link','imu_bosch')[:3,3];Ts=static_transform(motion['transforms'],'base_link','kistler');obs=OpticalVelocity(lever,Ts,(0,1));calobs=CalibratedOptical(lever,Ts,(0,1));raw=json.loads((a.private_root/'optical.json').read_text());used={e['message_id'] for e in raw['events']};held=[e for e in raw['raw_topics']['/sensor/kistler/measurement'] if e['message_id'] not in used];ht=np.array([e['header_ns'] for e in held]);im=[e for e in motion['events'] if e['kind']=='imu'];it=np.array([e['header_ns'] for e in im]);old=json.loads(a.baseline_priors.read_text());fixed=np.load(a.as_evaluation);corners=fixed['corner'];tan=fixed['tangent'];out={};series={}
for name in ['L_lap','XY_lap','C_lap']:
 tx=json.loads((a.private_root/name/'transactions.json').read_text());ev=np.load(a.private_root/name/'evaluation/evaluation.npz');ns=ev['header'];res=[];base_res=[];dt=[];vy=[];basevy=[];beta=[];gain=[]
 for i,r in enumerate(tx):
  u=r['prior'];x=State(*[np.array(u[k]) for k in ['p','v','R','bg','ba']],r['ref']);z=np.array(old[i]['prior']);base=State(z[:3],z[3:6],z[6:15].reshape(3,3),z[15:18],z[18:21],r['ref']);j=np.searchsorted(ht,r['ref'],side='right')-1;k=np.searchsorted(it,r['ref'],side='right')-1;omega=Rb@np.array(im[k]['omega']);vx=x.R.T@x.v-np.cross(omega-x.bg,lever);vb=base.R.T@base.v-np.cross(omega-base.bg,lever);vy.append(vx[1]);basevy.append(vb[1]);b=u.get('calibration_yaw_rad',0.);beta.append(b)
  if j<0 or r['ref']-ht[j]>6_000_000:res.append([np.nan,np.nan]);base_res.append([np.nan,np.nan]);dt.append(np.nan)
  else:
   h=(calobs.model(lift(x,b),omega) if name=='C_lap' else obs.model(x,omega))[0];res.append(np.array(held[j]['raw'])-h);base_res.append(np.array(held[j]['raw'])-obs.model(base,omega)[0]);dt.append((r['ref']-ht[j])*1e-6)
 res=np.array(res);base_res=np.array(base_res);valid=ev['historical_valid_0.075'];G=ev['G_0.075'];N=np.array([r['ndt']['final'] for r in tx])[:,:3,3];prior=ev.get('prior',np.array([np.array(r['seed'])[:3,3] for r in tx]));post=ev['candidate'];C=corners[:len(tx)];result={}
 for c in range(10):
  mask=C==c;v=mask&valid
  result[str(c)]=dict(heldout_raw_optical_x_rmse=stats(res[mask,0]),heldout_raw_optical_y_rmse=stats(res[mask,1]),baseline_raw_optical_y_rmse=stats(base_res[mask,1]),body_vy=stats(np.array(vy)[mask]),baseline_body_vy=stats(np.array(basevy)[mask]),calibration_yaw_deg=stats(np.rad2deg(np.array(beta)[mask])),reference={k:stats(abs(paired_errors(P,G,tan)[1][v])) for k,P in [('prior',prior),('NDT',N),('posterior',post)]})
 out[name]=dict(segments=result,heldout_packet_age_ms=stats(dt),raw_optical_y=stats(res[:,1]),baseline_raw_optical_y=stats(base_res[:,1]),independence='unused packets from the same optical sensor; temporally correlated, not independent truth',calibration_yaw_deg=stats(np.rad2deg(beta)),calibration_sigma_last_deg=float(np.rad2deg(np.sqrt(tx[-1]['prior']['P'][15][15]))) if len(tx[-1]['prior']['P'])==16 else None)
 series[name+'_raw_optical_residual']=res;series[name+'_base_residual']=base_res;series[name+'_body_vy']=vy;series[name+'_beta']=beta
 for key,values in [('native',ns),('corner',C)]:series[key]=values
np.savez_compressed(a.private_root/'mechanism_series.npz',**series);a.output.write_text(json.dumps(out,indent=2)+'\n')
