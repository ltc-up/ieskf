"""G2-V offline velocity-source A/B around the unchanged G2 matcher and filter."""
import argparse
import gc
import json
from pathlib import Path

import numpy as np

from geometry_ieskf.optical_velocity import OpticalVelocity
from velocity_replay import VelocityReplay
from runtime_journal import Journal, journal_json
from g2e_health import PoseHealth
from g2e2_health import SensorHealth


parser = argparse.ArgumentParser()
for name in ('input', 'bindings', 'target', 'deskew', 'deskew-service',
             'matcher', 'private', 'output'):
    parser.add_argument('--' + name, type=Path, required=True)
parser.add_argument('--optical', type=Path, required=True)
parser.add_argument('--policy', choices=('A', 'B', 'C'), required=True)
parser.add_argument('--window', default='lap')
parser.add_argument('--limit', type=int)
parser.add_argument('--prune-geometry', action='store_true',
                    help='Remove per-frame Deskew geometry after its hash and GICP result are recorded')
parser.add_argument('--g2e-gate', action='store_true')
parser.add_argument('--g2e-yaw-variance-scale', type=float, default=1.)
parser.add_argument('--g2e-recovery', action='store_true')
parser.add_argument('--g2e-continue-loss', action='store_true')
parser.add_argument('--g2e-inject-frame', type=int)
parser.add_argument('--g2e-drop-start', type=int)
parser.add_argument('--g2e-drop-count', type=int, default=0)
parser.add_argument('--g2e2-health', action='store_true')
parser.add_argument('--g2e2-recovery-baseline', action='store_true')
parser.add_argument('--g2e2-speed-drop-start', type=int)
parser.add_argument('--g2e2-speed-drop-count', type=int, default=0)
args = parser.parse_args()
assert args.g2e_yaw_variance_scale > 0
if args.g2e2_recovery_baseline:
    args.g2e_recovery = True
assert not args.private.exists()
args.private.mkdir(parents=True)

source = Path(__file__).parents[1] / 'g2_deterministic' / 'run_closed_loop_det.py'
program = source.read_text()
program = program.replace(
    'from ndt_global.ar_replay import AuditedPoseReplay as PoseDelayReplay,observation_health',
    'from ndt_global.ar_replay import observation_health\nPoseDelayReplay=VelocityReplay',
)
old = 'replay=PoseDelayReplay(x,P,held,lever);components='
new = (
    "replay=PoseDelayReplay(x,P,held,lever);replay.policy='g2v';"
    "replay.velocity_policy=a.policy;"
    "replay.optical_y=OpticalVelocity(lever,static_transform(data['transforms'],'base_link','kistler'),(1,));"
    "replay.optical_xy=OpticalVelocity(lever,static_transform(data['transforms'],'base_link','kistler'),(0,1));"
    "replay.optical=replay.optical_y;replay.optical_log=Journal(a.private/'optical_updates.jsonl',True);"
    'components='
)
assert program.count(old) == 1
program = program.replace(old, new)
needle = ' for i,j in enumerate(jobs):sensor.append'
assert program.count(needle) == 1
program = program.replace(needle, (
    " if a.policy!='A':sensor.extend(dict(e) for e in json.loads(a.optical.read_text())['events'] "
    "if e['header_ns']>t0)\n" + needle
))
needle = ' sensor.sort(key=PoseDelayReplay.key);cursor=0;now=t0;'
assert program.count(needle) == 1
program = program.replace(needle, """ if a.g2e2_speed_drop_start is not None and a.g2e2_speed_drop_count:
  lo_i=a.g2e2_speed_drop_start;hi_i=min(len(jobs),lo_i+a.g2e2_speed_drop_count)
  assert 0<=lo_i<hi_i
  drop_lo=jobs[lo_i]['native_reference_ns'];drop_hi=jobs[hi_i]['native_reference_ns'] if hi_i<len(jobs) else jobs[-1]['native_reference_ns']+1
  sensor=[e for e in sensor if not (e['kind'] in ('wheel','optical') and drop_lo<=e['header_ns']<drop_hi)]
""" + needle)
program = program.replace('ledger=[];outputs=[];wall=[]',
                          "ledger=Journal(a.private/'transactions.jsonl');outputs=Journal(a.private/'states.jsonl');wall=[]")
program = program.replace('wheel_records=[];health_records=[]',
                          "wheel_records=Journal(a.private/'wheel_internal.jsonl',True);velocity_log=Journal(a.private/'velocity_source.jsonl',True);health_records=[]")
for name in ('ledger', 'outputs', 'wheel_records'):
    program = program.replace('json.dumps(' + name + ')', 'journal_json(' + name + ')')
program = program.replace(' for i,job in enumerate(jobs):',
                          ' gc.collect();gc.freeze()\n for i,job in enumerate(jobs):')
program = program.replace('prior=dict(p=prior.p.tolist(),',
                          'prior=dict(v=prior.v.tolist(),bg=prior.bg.tolist(),ba=prior.ba.tolist(),p=prior.p.tolist(),')
program = program.replace('components=(',
                          "health=PoseHealth(lever,a.g2e_gate,a.g2e_recovery,a.g2e_yaw_variance_scale,a.g2e2_recovery_baseline);health_log=Journal(a.private/'g2e_health.jsonl',True);sensor_health=SensorHealth();sensor_health_log=Journal(a.private/'g2e2_sensor_health.jsonl',True);components=(", 1)
needle = "ndt=json.loads(line);T=np.array(ndt['final']);latency="
assert program.count(needle) == 1
program = program.replace(needle, """ndt=json.loads(line);T=np.array(ndt['final'])
   if a.g2e_inject_frame==i:
    T=T.copy();T[0,3]+=3.;T[:3,:3]=T[:3,:3]@Rotation.from_euler('z',4,degrees=True).as_matrix();ndt['final']=T.tolist()
   if a.g2e_drop_start is not None and a.g2e_drop_start<=i<a.g2e_drop_start+a.g2e_drop_count:ndt['converged']=False
   if ndt['converged'] and ndt['candidate_valid']:
    proposed_R=pose_covariance(ndt,components,prior);g2e_ok,g2e_R,g2e_diag=health.check(prior,Q,T,proposed_R,ref)
    health_log.append(dict(index=i,header_ns=ref,**g2e_diag,gate_accepted=g2e_ok))
   else:health_log.append(dict(index=i,header_ns=ref,reason='MATCH_INVALID',state=health.state(ref),gate_accepted=False))
   latency=""")
program = program.replace("if a.mode!='E' and ndt['converged'] and ndt['candidate_valid']:",
                          "if a.mode!='E' and ndt['converged'] and ndt['candidate_valid'] and g2e_ok:")
program = program.replace("else:update=dict(accepted=False,reason='NDT_NOT_CONVERGED_OR_INVALID')",
                          "else:update=dict(accepted=False,reason='POSE_HEALTH_REJECTED' if ndt['converged'] and ndt['candidate_valid'] else 'NDT_NOT_CONVERGED_OR_INVALID')")
program = program.replace('Rv=pose_covariance(ndt,components,prior);projection=None',
                          'Rv=g2e_R;projection=None')
program = program.replace("  if update['accepted']:\n", "  health.record(ref,update['accepted'])\n  if update['accepted']:\n", 1)
program = program.replace("if a.stop_loss is not None and a.mode", "if a.stop_loss is not None and not a.g2e_continue_loss and a.mode")
needle = "print(a.mode,i+1,update['reason'],flush=True)"
assert program.count(needle) == 1
program = program.replace(needle, needle + "\n  if a.prune_geometry:\n   poses.unlink(missing_ok=True);path.unlink(missing_ok=True)\n   for key in ('native_xyzi','common_xyzi'):\n    if key in ds:Path(ds[key]).unlink(missing_ok=True)")
program = program.replace('replay.ingest(e);after=replay.current()[0]',
                          "replay.ingest(e);after=replay.current()[0]\n   if a.g2e2_health:sensor_health.observe(e)\n   if e['kind'] in ('wheel','optical'):velocity_log.append(dict(kind=e['kind'],header_ns=e['header_ns'],source=e.get('selected_vx_source'),update=e.get('last_update',{}),observed_velocity=e.get('velocity'),wheel_speed=e.get('speed')))")
needle = 'current,PC,_=replay.current();outputs.append'
assert program.count(needle) == 1
program = program.replace(needle, "current,PC,_=replay.current();hs,hP,_=checkpoint(ref) if a.g2e2_health else (None,None,None);sensor_health_log.append(dict(index=i,header_ns=ref,**sensor_health.snapshot(ref,hs,hP,health.state(ref)))) if a.g2e2_health else None;outputs.append")
namespace = dict(__name__='g2v_runner', gc=gc, Journal=Journal,
                 journal_json=journal_json, VelocityReplay=VelocityReplay,
                 OpticalVelocity=OpticalVelocity, PoseHealth=PoseHealth,
                 SensorHealth=SensorHealth, np=np)
exec(compile(program, str(source), 'exec'), namespace)
namespace['pose_covariance'] = lambda ndt, components, prior: np.diag([
    .25 if component in ('x', 'y', 'z') else np.deg2rad(1.)**2
    for component in components
])
args.mode = 'F'
args.stop_loss = 2.
args.position_only = False
namespace['main'](args)
summary = json.loads(args.output.read_text())
summary.update(velocity_policy=args.policy, actual_raw_optical=args.policy != 'A',
               lidar_R_unchanged=(not args.g2e_recovery or args.g2e2_recovery_baseline)
                                  and args.g2e_yaw_variance_scale==1.,
               full_6d_pose=True, g2e_gate=args.g2e_gate,
               g2e_yaw_variance_scale=args.g2e_yaw_variance_scale,
               g2e_recovery=args.g2e_recovery)
summary.update(g2e2_health=args.g2e2_health,
               g2e2_recovery_baseline=args.g2e2_recovery_baseline,
               g2e2_speed_drop_start=args.g2e2_speed_drop_start,
               g2e2_speed_drop_count=args.g2e2_speed_drop_count)
args.output.write_text(json.dumps(summary, indent=2) + '\n')
