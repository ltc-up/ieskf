"""Exact AO recursion instrumentation; complete pre-update records stay private."""
import argparse,json,importlib.util,time,collections
from pathlib import Path
import numpy as np
from geometry_ieskf.motion_audit import gain_decomposition,derivative_audit
from geometry_ieskf.common import sha,C

def main(a):
 spec=importlib.util.spec_from_file_location('ao_local',Path(__file__).parents[1]/'phase_ao_full_lap_validation/run_local.py');ao=importlib.util.module_from_spec(spec);spec.loader.exec_module(ao);data=json.loads(a.input.read_text());original=ao.WheelObservation.correct;predict=ao.InertialPredictor.predict;records={};maxdz=-1;wheel_sum=0.;prop_sum=0.;firstp=None;groups=collections.defaultdict(list);identity=[];start=None
 def capture(self,x,P,w,z,var,age=0):
  nonlocal maxdz,wheel_sum,firstp,start
  y,Q,info=original(self,x,P,w,z,var,age)
  if start is None:start=x.t;firstp=x.p[2]
  if info['accepted']:
   r=gain_decomposition(x,P,w,z,var,self.lever);err=abs(r['delta'][2]-info['delta'][2]);identity.append(err);wheel_sum+=info['delta'][2];dz=abs(info['delta'][2]);bucket='low' if abs(z)<7 else 'medium' if abs(z)<30 else 'high';groups[bucket].append([abs(r['innovation']),dz,float(np.linalg.norm(w)),abs(z)])
   labels=[]
   if dz>=1 and 'first_wheel_abs_dZ_ge_1m' not in records:labels+=['first_wheel_abs_dZ_ge_1m']
   if abs(x.p[2]-firstp)>=1 and 'first_abs_Z_change_ge_1m' not in records:labels+=['first_abs_Z_change_ge_1m']
   if dz>maxdz:maxdz=dz;labels+=['maximum']
   for label in labels:records[label]=dict(elapsed_s=(x.t-start)*1e-9,**r,pre_state=dict(p=x.p.tolist(),v=x.v.tolist(),R=x.R.tolist(),bg=x.bg.tolist(),ba=x.ba.tolist()),P=P.tolist(),delta_Z=float(info['delta'][2]),delta_vz=float(info['delta'][5]),derivatives=derivative_audit(x,w,np.array([0,0,9.8]),.005,self.lever))
  return y,Q,info
 def propagate(self,x,P,w,f,dt):
  nonlocal prop_sum
  y,Q=predict(self,x,P,w,f,dt);prop_sum+=y.p[2]-x.p[2];return y,Q
 ao.WheelObservation.correct=capture;ao.InertialPredictor.predict=propagate;a.private.mkdir(parents=True,exist_ok=True);result=ao.run(data,a.private);(a.private/'gain_records.json').write_text(json.dumps(records));public={}
 for k,r in records.items():public[k]={n:r[n] for n in ['elapsed_s','predicted','innovation','S','Kz_by_block','Pzz','correlation_z_velocity','delta_Z','delta_vz','derivatives']};public[k]['Kz']=r['K'][2];public[k]['delta_attitude_norm_rad']=float(np.linalg.norm(r['delta'][6:9]));public[k]['delta_bg_norm']=float(np.linalg.norm(r['delta'][9:12]));public[k]['delta_ba_norm']=float(np.linalg.norm(r['delta'][12:15]))
 summary=dict(records=public,max_gain_identity_error=max(identity,default=0),signed_Z_increment_sum=dict(propagation_m=prop_sum,wheel_m=wheel_sum),speed_groups={k:dict(count=len(v),innovation_abs_p50_p95_max=np.percentile(np.array(v)[:,0],[50,95,100]).tolist(),wheel_abs_dZ_p50_p95_max=np.percentile(np.array(v)[:,1],[50,95,100]).tolist(),angular_rate_p50_p95_max=np.percentile(np.array(v)[:,2],[50,95,100]).tolist()) for k,v in groups.items()},AO_result=result,private_gain_records_sha256=sha(a.private/'gain_records.json'),input_sha256=sha(a.input),calibrated=False)
 a.output.write_text(json.dumps(summary,indent=2)+'\n')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--private',type=Path,required=True);p.add_argument('--output',type=Path,required=True);main(p.parse_args())
