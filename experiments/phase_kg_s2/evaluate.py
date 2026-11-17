"""Offline-only engineering reference. Must never be imported by optimizer."""
import sys,json,subprocess,hashlib
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation,Slerp
REPO=Path(__file__).resolve().parents[2];sys.path.insert(0,str(REPO/'src'))
from ndt_global.validation import static_transform
from ndt_global.track_evaluation import interpolate_reference
from ndt_global.footprint_audit import antenna_to_base

def stats(a):
 a=np.asarray(a,float);return dict(n=len(a),p50=float(np.median(a)),p95=float(np.percentile(a,95)),p99=float(np.percentile(a,99)),max=float(a.max()))if len(a)else dict(n=0)
def main(root,old,folder,config_name="series.json"):
 root=Path(root);old=Path(old);directory=root/folder;d=json.load(open(root/'inputs'/config_name));origin=d['origin'];series={s['id']:s for s in d['series']};raw=json.load(open(old/'phase_bd_ndt_health_bridge/inputs/research_data/phase_at_gnss_footprint_corners/reference_bound.json'));cfg=yaml.safe_load((old/'phase_be_e0_e4/runtime/offline_reference_config.yaml').read_text());cfg=cfg.get('/**',{}).get('ros__parameters',cfg);ar=raw['/vectornav/raw/attitude'];at=np.array([r['header']-origin*1e-9 for r in ar]);quats=np.array([r['q']for r in ar]);keep=np.r_[True,np.diff(at)>0];at=at[keep];quats=quats[keep];T=static_transform(raw['/tf_static'],'base_link','vectornav_ant1');rotation=Rotation.from_euler('xyz',cfg['pose_orientation_bias'])*Rotation.from_quat([2**-.5,2**-.5,0,0])*Rotation.from_quat(quats)*Rotation.from_quat([1,0,0,0])*Rotation.from_matrix(T[:3,:3]).inv();sl=Slerp(at,rotation);gps=[r for r in raw['/vectornav/raw/gps']if r['fix']==8];gt=np.array([r['header']-origin*1e-9 for r in gps]);payload=''.join(' '.join(map(str,r['lla']))+'\n'for r in gps);z=subprocess.run(['CartConvert','-l',*map(str,cfg['reference_point']),'-p','10'],input=payload,text=True,capture_output=True,check=True);enu=np.loadtxt(z.stdout.splitlines())
 def ref(t,lag):
  qt=np.array([(t-origin)*1e-9]);
  if not at[0]<=qt[0]<=at[-1]:return None
  rot=sl(qt).as_matrix();ant,valid=interpolate_reference(gt-lag,enu,np.ones(len(gt),bool),qt,.600001)
  if not valid[0]:return None
  # Guard unusually large attitude gaps; no filling missing engineering reference.
  k=np.searchsorted(at,qt[0]);
  if 0<k<len(at)and at[k]-at[k-1]>.1:return None
  ans=np.eye(4);ans[:3,:3]=rot[0];ans[:3,3]=antenna_to_base(ant,rot,T[:3,3])[0];return ans
 def error(state,t,lag):
  B=ref(t,lag)
  if B is None:return dict(valid=False)
  A=np.eye(4);A[:3,:3]=np.asarray(state['T_map_base_R']);A[:3,3]=np.asarray(state['p_base']).ravel();D=np.linalg.inv(B)@A;return dict(valid=True,xy=float(np.linalg.norm(D[:2,3])),longitudinal=float(D[0,3]),lateral=float(D[1,3]),z=float(D[2,3]),z_map=float(A[2,3]-B[2,3]),yaw_deg=float(np.rad2deg(np.arctan2(D[1,0],D[0,0]))))
 summaries=[];details=[]
 for p in sorted(directory.glob('*_E[0-4]')):
  if not(p/'steps.jsonl').exists():continue
  name,E=p.name.rsplit('_E',1);rows=list(map(json.loads,open(p/'steps.jsonl')));rec=series[name]['records'];recordby={r['t']:r for r in rec};s=dict(series=name,E=int(E),processed=len(rows),timing=stats([r['total_step_ms']for r in rows]),update_timing=stats([r['update_ms']for r in rows]),over50=sum(r['total_step_ms']>50 for r in rows),by_lag={});allerr={};bias=[];Pmin=[]
  for lag in [0.,.075,.15]:
   es=[error(r['final'],r['t'],lag)for r in rows];ee=[e for e in es if e['valid']];s['by_lag'][str(lag)]=dict(n=len(ee),XY_RMSE=float(np.sqrt(np.mean([e['xy']**2 for e in ee])))if ee else None,lateral_abs=stats([abs(e['lateral'])for e in ee]),longitudinal_abs=stats([abs(e['longitudinal'])for e in ee]),yaw_RMSE_deg=float(np.sqrt(np.mean([e['yaw_deg']**2 for e in ee])))if ee else None,z_local_reference_abs=stats([abs(e['z'])for e in ee]),z_map_abs=stats([abs(e['z_map'])for e in ee]));allerr[str(lag)]=es
  for i,r in enumerate(rows):
   st=r['final'];bias.append(np.linalg.norm(np.array(st['bias_accel_imu']).ravel()));diag=r.get('diagnostic_only',{});
   if 'P_X_V_B_explicit'in diag:Pmin.append(float(np.linalg.eigvalsh(diag['P_X_V_B_explicit']).min()))
   details.append(dict(series=name,E=int(E),t=r['t'],error={k:v[i]for k,v in allerr.items()},correction_map=(np.array(st['p_base'])-np.array(r['initial']['p_base'])).ravel().tolist(),state=st))
  s['Vz_map_IMU_estimate']=stats([np.array(r['final']['velocity_map_imu']).ravel()[2]for r in rows]);s['bias_accel_norm']=stats(bias);s['P_min_eigen']=min(Pmin)if Pmin else None;summaries.append(s)

 if folder=='native_verified':
  q={x['id']:x for x in json.load(open(root/'inputs/geometric_diagnostic_plan.json'))};rawdiag=list(map(json.loads,open(root/'results/native_raw_geometry.jsonl')));geometry_errors=[]
  for x in rawdiag:
   if not x['id'].startswith('old_'):continue
   z=q[x['id']];poses={'frozen_input_pose':np.array(z['seed'])}
   if 'independent_geometry_diagnostic_only'in x:poses['native_single_factor_candidate']=np.array(x['independent_geometry_diagnostic_only']['base_pose'])
   geometry_errors.append(dict(id=x['id'],by_pose={name:{str(lag):error(dict(T_map_base_R=A[:3,:3],p_base=A[:3,3]),z['t'],lag)for lag in [0.,.075,.15]}for name,A in poses.items()}))
  (root/'results/native_raw_offline_errors.json').write_text(json.dumps(geometry_errors,indent=2))
 (root/f'{folder}_evaluation_detail.json').write_text(json.dumps(details,indent=2));(root/f'{folder}_evaluation_summary.json').write_text(json.dumps(summaries,indent=2));print(json.dumps([{k:r[k]for k in ['series','E','by_lag','over50']}for r in summaries],indent=2))
if __name__=='__main__':main(*sys.argv[1:])
