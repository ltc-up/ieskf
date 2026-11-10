"""Offline independent-IMU dynamics check; never supplies estimator inputs."""
import argparse,json
from pathlib import Path
import numpy as np
from ndt_global.validation import static_transform
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--motion',type=Path,required=True);a=p.parse_args();m=json.loads(a.motion.read_text());inv=json.loads((a.root/'modalities.json').read_text());d=np.load(a.root/'modalities.npz');main='/sensor/bosch/imu';Rb=static_transform(m['transforms'],'base_link','imu_bosch');tb=d[main+'_header'];w=d[main+'_gyro']@Rb[:3,:3].T;f=d[main+'_accel']@Rb[:3,:3].T;origin=tb[0];t=(tb-origin)*1e-9;wp=np.column_stack([np.interp(t-.1,t,w[:,i]) for i in range(3)]);alpha=(w-wp)/.1;out={}
for name,meta in inv.items():
 if name==main or meta['point_fields'] or name+'_header' not in d:continue
 try:T=static_transform(m['transforms'],'base_link',meta['frames'][0])
 except (KeyError,ValueError) as e:out[name]=dict(qualification='FRAME_UNRESOLVED');continue
 ts=(d[name+'_header']-origin)*1e-9;gyro=d[name+'_gyro']@T[:3,:3].T;acc=d[name+'_accel']@T[:3,:3].T;valid=(t>=max(.1,ts[0]))&(t<=ts[-1]);wi=np.column_stack([np.interp(t,ts,gyro[:,i]) for i in range(3)]);fi=np.column_stack([np.interp(t,ts,acc[:,i]) for i in range(3)]);r=T[:3,3]-Rb[:3,3];fcorrected=fi-np.cross(alpha,r)-np.cross(w,np.cross(w,r));dg=wi[valid]-w[valid];df=fcorrected[valid]-f[valid];out[name]=dict(paired=len(dg),gyro_difference_RMS=np.sqrt(np.mean(dg*dg,axis=0)).tolist(),gyro_difference_abs_P95=np.percentile(abs(dg),95,axis=0).tolist(),specific_force_without_lever_difference_RMS=np.sqrt(np.mean((fi[valid]-f[valid])**2,axis=0)).tolist(),specific_force_difference_abs_P95=np.percentile(abs(df),95,axis=0).tolist(),gyro_yaw_correlation=float(np.corrcoef(w[valid,2],wi[valid,2])[0,1]),specific_force_difference_RMS=np.sqrt(np.mean(df*df,axis=0)).tolist(),interpretation='conditional static-TF, nominal-time and rigid-body lever compensation; filtered/DBC/scale/clock physical contracts not independently certified; no averaging/fusion')
(a.root/'auxiliary_check.json').write_text(json.dumps(out,indent=2)+'\n')
