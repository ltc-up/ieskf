"""Stationary-candidate and wheel/Kistler comparisons; not a calibration fit."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from ndt_global.validation import static_transform

def main(a):
 d=json.loads(a.input.read_text());by={k:sorted([e for e in d['events'] if e['kind']==k],key=lambda e:e['header_ns']) for k in ['imu','wheel','kistler','pose']};im=by['imu'];wh=by['wheel'];kt=by['kistler'];po=by['pose'];wt=np.array([e['header_ns'] for e in wh]);pt=np.array([e['header_ns'] for e in po]);Rb=static_transform(d['transforms'],'base_link','imu_bosch')[:3,:3];f=[];az=[];gyro=[];ktimes=np.array([e['header_ns'] for e in kt])
 for e in im:
  t=e['header_ns'];j=np.searchsorted(wt,t,side='right')-1;k=np.searchsorted(pt,t,side='right')-1
  if j<0 or k<0 or t-wt[j]>100_000_000 or t-pt[k]>30_000_000:continue
  ki=np.searchsorted(ktimes,t,side='right')-1
  independent_motion_quiet=ki>=0 and t-ktimes[ki]<=20_000_000 and abs(kt[ki]['speed'])<.2
  if abs(wh[j]['speed'])<.2 and np.linalg.norm(e['omega'])<.02 and independent_motion_quiet:
   F=Rb@e['force'];R=Rotation.from_quat(po[k]['quaternion_xyzw']).as_matrix();f.append(np.linalg.norm(F));az.append((R@F)[2]-9.80665);gyro.append(np.linalg.norm(e['omega']))
 groups={};times=np.array([e['header_ns'] for e in kt])
 for v in wh:
  j=np.searchsorted(times,v['header_ns'],side='right')-1
  if j<0 or v['header_ns']-times[j]>20_000_000:continue
  label='low' if abs(v['speed'])<7 else 'medium' if abs(v['speed'])<30 else 'high';groups.setdefault(label,[]).append(v['speed']-kt[j]['speed'])
 out=dict(wheel_messages=len(wh),wheel_exact_zero=sum(e['speed']==0 for e in wh),Kistler_abs_speed_p50_p95=np.percentile([abs(e['speed']) for e in kt],[50,95]).tolist(),static_candidate_rule='wheel and fresh Kistler-x both abs<0.2m/s and gyro norm<0.02rad/s; still not a certified stationary detector',static_candidates=len(f),force_norm_m_s2_p5_p50_p95=np.percentile(f,[5,50,95]).tolist(),vertical_acceleration_with_historical_attitude_p5_p50_p95=np.percentile(az,[5,50,95]).tolist(),stationary_gnss_independent=False,noise_calibrated=False,gravity_removed_input_evidence=False,interpretation='force norm near gravity and publisher g-to-SI conversion support specific-force interpretation; actual fix_autonoma/runtime calibration remains unverified',wheel_minus_raw_Kistler_x={k:dict(n=len(v),p5_p50_p95=np.percentile(v,[5,50,95]).tolist()) for k,v in groups.items()},Kistler_comparison='raw sensor-x diagnostic only; not base velocity truth or a qualified joint observation')
 a.output.write_text(json.dumps(out,indent=2)+'\n')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True);main(p.parse_args())
