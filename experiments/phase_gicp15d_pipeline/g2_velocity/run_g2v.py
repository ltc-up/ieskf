"""G2-V offline velocity-source A/B around the unchanged G2 matcher and filter."""
import argparse
import gc
import json
from pathlib import Path

import numpy as np

from geometry_ieskf.optical_velocity import OpticalVelocity
from velocity_replay import VelocityReplay
from runtime_journal import Journal, journal_json


parser = argparse.ArgumentParser()
for name in ('input', 'bindings', 'target', 'deskew', 'deskew-service',
             'matcher', 'private', 'output'):
    parser.add_argument('--' + name, type=Path, required=True)
parser.add_argument('--optical', type=Path, required=True)
parser.add_argument('--policy', choices=('A', 'B', 'C'), required=True)
parser.add_argument('--window', default='lap')
parser.add_argument('--limit', type=int)
args = parser.parse_args()
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
program = program.replace('replay.ingest(e);after=replay.current()[0]',
                          "replay.ingest(e);after=replay.current()[0]\n   if e['kind'] in ('wheel','optical'):velocity_log.append(dict(kind=e['kind'],header_ns=e['header_ns'],source=e.get('selected_vx_source'),update=e.get('last_update',{}),observed_velocity=e.get('velocity'),wheel_speed=e.get('speed')))")
namespace = dict(__name__='g2v_runner', gc=gc, Journal=Journal,
                 journal_json=journal_json, VelocityReplay=VelocityReplay,
                 OpticalVelocity=OpticalVelocity, np=np)
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
               lidar_R_unchanged=True, full_6d_pose=True)
args.output.write_text(json.dumps(summary, indent=2) + '\n')
