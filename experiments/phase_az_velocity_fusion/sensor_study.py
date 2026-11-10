"""Sensor-only diagnostics: no GNSS or geographical segment input."""
import argparse,json
from pathlib import Path
import numpy as np
from ndt_global.validation import static_transform
from geometry_ieskf.state import State
from geometry_ieskf.optical_velocity import OpticalVelocity
from geometry_ieskf.wheel import WheelObservation
p=argparse.ArgumentParser();p.add_argument('--optical',type=Path,required=True);p.add_argument('--motion',type=Path,required=True);p.add_argument('--states',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();o=json.loads(a.optical.read_text());m=json.loads(a.motion.read_text());states=json.loads(a.states.read_text());events=o['events'];wheel=[e for e in m['events'] if e['kind']=='wheel'];imu=[e for e in m['events'] if e['kind']=='imu'];wt=np.array([e['header_ns'] for e in wheel]);it=np.array([e['header_ns'] for e in imu]);st=np.array([e['time_ns'] for e in states]);lever=static_transform(m['transforms'],'base_link','imu_bosch')[:3,3];Rb=static_transform(m['transforms'],'base_link','imu_bosch')[:3,:3];T=static_transform(m['transforms'],'base_link','kistler');opt=OpticalVelocity(lever,T,(0,1));wh=WheelObservation(lever);rows=[]
for e in events:
 t=e['header_ns'];j=np.searchsorted(wt,t,side='right')-1;k=np.searchsorted(it,t,side='right')-1;q=np.searchsorted(st,t,side='right')-1
 if min(j,k,q)<0 or t-wt[j]>20_000_000 or t-st[q]>100_000_000:continue
 u=states[q];x=State(*[np.array(u[n]) for n in ['p','v','R','bg','ba']],u['time_ns']);w=Rb@np.array(imu[k]['omega']);force=Rb@np.array(imu[k]['force']);hp,_=opt.model(x,w);hw,_=wh.model(x,w);z=np.array(e['velocity']);ax=(force-x.ba+x.R.T@np.array([0,0,-9.80665]))[0];rw=wheel[j]['speed']-hw;ro=z-hp
 rows.append([t,z[0],z[1],wheel[j]['speed'],rw-ro[0],ro[0],ro[1],ax,w[2]-x.bg[2],(t-wt[j])*1e-6,(e['recorder_ns']-t)*1e-6,e['status_good']])
r=np.array(rows);out={}
def stats(v):return dict(n=len(v),mean=float(np.mean(v)),rmse=float(np.sqrt(np.mean(v*v))),p05=float(np.percentile(v,5)),p50=float(np.percentile(v,50)),p95=float(np.percentile(v,95)),abs_max=float(np.max(abs(v))))
for label,mask in dict(all=np.ones(len(r),bool),accelerating=r[:,7]>1,braking=r[:,7]<-1,steady=abs(r[:,7])<=1,turning=abs(r[:,8])>.1,straight=abs(r[:,8])<=.1,fast=r[:,1]>50,medium=(r[:,1]>=20)&(r[:,1]<=50),slower=r[:,1]<20).items():
 if not np.any(mask):continue
 out[label]={name:stats(r[mask,col]) for name,col in [('wheel_minus_optical_raw_X',3),('reference_compensated_disagreement',4),('opticalX_residual',5),('opticalY_residual',6),('wheel_pair_age_ms',9),('optical_record_age_ms',10)]};out[label]['wheel_minus_optical_raw_X']=stats((r[:,3]-r[:,1])[mask]);out[label]['over5_disagreement_risk']=int(sum(abs(r[mask,4])>5))
# Temporal dependence and delay scan are descriptive; no lag is fed to the filter.
wtimes=(r[:,0]-r[0,0])*1e-9;ox=r[:,1];wx=r[:,3];dt=.2;valid=wtimes>dt;dx=ox[valid]-np.interp(wtimes[valid]-dt,wtimes,ox);dw=wx[valid]-np.interp(wtimes[valid]-dt,wtimes,wx);scans={}
for shift in [-.1,-.05,0,.05,.1]:
 ts=wtimes[valid]+shift;ok=(ts>=dt)&(ts<=wtimes[-1]);pred=np.interp(ts[ok],wtimes,wx)-np.interp(ts[ok]-dt,wtimes,wx);scans[str(shift)]=stats(dx[ok]-pred)
out['sensor_increment_lag_sensitivity']=scans;out['sample_dependence']=dict(optical_X_increment_lag1_correlation=float(np.corrcoef(np.diff(ox)[:-1],np.diff(ox)[1:])[0,1]));out['limits']='No GNSS. Common reference uses preceding frozen baseline state up to100ms old; compensation and dynamics are conditional diagnostics, not independent truth or certified slip/delay. Individual raw wheel contract requires separate qualification.'
a.output.write_text(json.dumps(out,indent=2)+'\n');np.savez_compressed(a.output.with_suffix('.npz'),sensor_only_rows=r)
