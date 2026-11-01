"""Single admitted diagnostic replay; no production output or parameter modification."""
import argparse,json,time
from pathlib import Path
import numpy as np
from geometry_ieskf.common import dump,sha
from geometry_ieskf.replay import EventReplay
from geometry_ieskf.qualification import QualifiedUpdate,TrackedCorrespondence
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output)
admission=json.load(open(O/'continuous_admission.json'));assert admission['eligible_diagnostic_only']
assert not (O/'candidate_continuous.json').exists(),'Do not accidentally repeat the admitted experiment'
contract=json.load(open(D/'history/phase_v/results/input_contract.json'));events=json.load(open(D/'history/phase_v/results/input_continuous_extension.json'));assert sha(D/'inputs/map_target.bin')==contract['target_sha256']
def load(e):
 path=D/'history/phase_v'/e['source_file'];assert sha(path)==e['source_sha256'];return np.load(path)
query=TrackedCorrespondence(np.fromfile(D/'inputs/map_target.bin','<f4',offset=8).reshape(-1,3).astype(float));replay=EventReplay(contract,source_loader=load);replay.solver=QualifiedUpdate(replay.factor);original=replay.solver.correct
def traced(*args):
 x,P,info=original(*args);info['solver_trace']=replay.solver.trace;return x,P,info
replay.solver.correct=traced
r=replay.run(events,query);dump(O/'candidate_continuous.json',r);print(r['counts'],r['aborted'],r['cpu_replay_wall_s'],flush=True)
