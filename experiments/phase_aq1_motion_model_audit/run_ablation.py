"""Same causal initialization; 20s predetermined normal raw interval, no maps."""
import argparse,json,importlib.util
from pathlib import Path
import numpy as np
from geometry_ieskf.common import sha
from geometry_ieskf.motion_audit import sensor_velocity_model
from ndt_global.validation import static_transform

def main(a):
 data=json.loads(a.input.read_text());bindings=json.loads(a.bindings.read_text());normal=[j for j in bindings if j['scene']=='normal'];lo=min(min(c['scan_start_ns'] for c in j['clouds']) for j in normal)-100_000_000;hi=lo+20_000_000_000;events=data['events'];pose=max((e for e in events if e['kind']=='pose' and e['header_ns']<=lo),key=lambda e:e['header_ns']);start=pose['header_ns'];cut=[e for e in events if e['kind'] in ('imu','wheel','pose') and start-100_000_000<=e['header_ns']<=hi];sub=dict(data,events=cut);spec=importlib.util.spec_from_file_location('local',Path(__file__).parents[1]/'phase_ao_full_lap_validation/run_local.py');ao=importlib.util.module_from_spec(spec);spec.loader.exec_module(ao);original=ao.WheelObservation.correct;out={}
 for mode in ('IMU_ONLY','WHEEL'):
  if mode=='IMU_ONLY':ao.WheelObservation.correct=lambda self,x,P,w,z,var,age=0:(x.copy(),P.copy(),dict(accepted=False,reason='ABLATION_NO_VELOCITY_UPDATE'))
  else:ao.WheelObservation.correct=original
  private=a.private/mode;private.mkdir(parents=True,exist_ok=True);out[mode]=ao.run(sub,private)
 ao.WheelObservation.correct=original
 # Verified raw frame, sensor-axis model and actual lever; no invented 3D speed.
 k=json.loads(a.kistler.read_text());Ts=static_transform(data['transforms'],'base_link','kistler');model=ao.WheelObservation.model
 ao.WheelObservation.model=lambda self,x,omega:sensor_velocity_model(x,omega,self.lever,Ts[:3,3],Ts[:3,0])
 kev=[e for e in k['events'] if start<e['header_ns']<=hi];before=max((e for e in cut if e['kind']=='wheel' and e['header_ns']<=start),key=lambda e:e['header_ns']);subk=dict(sub,events=[e for e in cut if e['kind']!='wheel']+[before]+kev);private=a.private/'KISTLER';private.mkdir(parents=True,exist_ok=True);out['KISTLER']=ao.run(subk,private);out['KISTLER']['qualification']='CONDITIONAL raw sensor-axis x, status/noise/correlation not calibrated';ao.WheelObservation.model=model
 out['JOINT']=dict(status='NOT_QUALIFIED',reason='shared vehicle signals and correlation not calibrated; no duplicated scalar speed fusion')
 a.output.write_text(json.dumps(dict(provenance='real normal raw 20s, fixed input-only choice, no NDT',duration_s=(hi-start)*1e-9,branches=out),indent=2)+'\n')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--bindings',type=Path,required=True);p.add_argument('--private',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--kistler',type=Path,required=True);main(p.parse_args())
