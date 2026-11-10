"""Frozen NDT100+motion loop, opt-in actual raw optical velocity constraint."""
import argparse,json,gc,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).parents[1]/'phase_aw_native_gicp_fusion'))
from runtime_journal import Journal,journal_json
from geometry_ieskf.optical_velocity import OpticalVelocity
from ndt_global.optical_replay import OpticalReplay
p=argparse.ArgumentParser()
for k in ['input','bindings','target','deskew','deskew-service','matcher','private','output','optical']:p.add_argument('--'+k,type=Path,required=True)
p.add_argument('--profile',action='store_true');p.add_argument('--calibrate-yaw',action='store_true');p.add_argument('--policy',choices=['baseline','optical_lateral','optical_xy'],required=True);p.add_argument('--window',default='lap');p.add_argument('--limit',type=int);a=p.parse_args();assert not a.private.exists();a.private.mkdir(parents=True)
s=(Path(__file__).parents[1]/'phase_ar_loop44_map0105/run_closed_loop.py').read_text();s=s.replace('from ndt_global.ar_replay import AuditedPoseReplay as PoseDelayReplay,observation_health','from ndt_global.ar_replay import observation_health\nfrom ndt_global.optical_replay import OpticalReplay as PoseDelayReplay')
old='replay=PoseDelayReplay(x,P,held,lever);components='
new="replay=PoseDelayReplay(x,P,held,lever);replay.policy=a.policy;replay.optical=OpticalVelocity(lever,static_transform(data['transforms'],'base_link','kistler'),(0,1) if a.policy=='optical_xy' else (1,));replay.optical_log=Journal(a.private/'optical_updates.jsonl',True);components="
assert old in s;s=s.replace(old,new)
needle=" for i,j in enumerate(jobs):sensor.append"
s=s.replace(needle," if a.policy!='baseline':sensor.extend(e for e in json.loads(a.optical.read_text())['events'] if e['header_ns']>t0)\n"+needle)
s=s.replace('ledger=[];outputs=[];wall=[]',"ledger=Journal(a.private/'transactions.jsonl');outputs=Journal(a.private/'states.jsonl');wall=[]").replace('wheel_records=[];health_records=[]',"wheel_records=Journal(a.private/'wheel_internal.jsonl',True);health_records=[]")
for key in ['ledger','outputs','wheel_records']:s=s.replace('json.dumps('+key+')','journal_json('+key+')')
s=s.replace(' for i,job in enumerate(jobs):',' gc.collect();gc.freeze()\n for i,job in enumerate(jobs):')
s=s.replace('prior=dict(p=prior.p.tolist(),R=prior.R.tolist(),P=Q.tolist())','prior=dict(p=prior.p.tolist(),v=prior.v.tolist(),R=prior.R.tolist(),bg=prior.bg.tolist(),ba=prior.ba.tolist(),P=Q.tolist())')
s=s.replace('prior=dict(p=prior.p.tolist(),',"prior=dict(calibration_yaw_rad=float(getattr(prior,'beta',0.)),p=prior.p.tolist(),")
if a.profile:
 from runtime_probe import RuntimeProbe
 probe=RuntimeProbe();s=s.replace(' for i,job in enumerate(jobs):', ' for i,job in enumerate(jobs):\n  probe.frame=i;cycle_start=time.perf_counter_ns()');s=s.replace("  if a.stop_loss is not None","  probe.record(i,start,cycle_start)\n  if a.stop_loss is not None")
ns=dict(__name__='ay_actual_loop',Journal=Journal,journal_json=journal_json,gc=gc,OpticalVelocity=OpticalVelocity)
if a.profile:ns['probe']=probe
exec(compile(s,'AY_reuse_AR','exec'),ns)
if a.calibrate_yaw:
 from geometry_ieskf.optical_calibration import CalibratedPredictor,CalibratedWheel,CalibratedOptical,CalibratedPose,lift
 class Replay(OpticalReplay):
  def __init__(self,x,P,imu,lever):
   Pi=np.zeros((16,16));Pi[:15,:15]=P;Pi[15,15]=np.deg2rad(1.)**2;super().__init__(lift(x),Pi,imu,lever);self.predictor=CalibratedPredictor();self.wheel=CalibratedWheel(lever)
 ns['PoseDelayReplay']=Replay;ns['OpticalVelocity']=CalibratedOptical;ns['ExternalPose']=CalibratedPose
# Save the extra physical state privately; original interface fields are retained.
# The unchanged compiled AR wrapper sees beta through this explicitly selected class.
if a.profile:probe.install(ns)
a.mode='F';a.stop_loss=2.;ns['main'](a);r=json.loads(a.output.read_text());r.update(policy=a.policy,GNSS_feedback=False,actual_RAW_OPTICAL=a.policy!='baseline',noise_qualified=False,source='raw optical axes, no internal corrected velocity');a.output.write_text(json.dumps(r,indent=2)+'\n')

if a.profile:probe.save(a.private/'runtime_probe.json')
