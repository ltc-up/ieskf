"""Frozen candidate implementation parity, not a continuous replay."""
import argparse,json,numpy as np
from pathlib import Path
from geometry_ieskf.common import dump
from geometry_ieskf.objective import state
from geometry_ieskf.geometry import GeometryFactor
from geometry_ieskf.qualification import QualifiedUpdate,TrackedCorrespondence
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);out=Path(a.output)
old=json.load(open(D/'phase_aa_z_validation/new_replay.json'));analysis=json.load(open(D/'phase_ab_geometry_qualification/frame_diagnostics.json'));byordinal={e['ordinal']:e for e in analysis};events=json.load(open(D/'history/phase_v/results/input_continuous_extension.json'));frames={e['header_ns']:e for e in events['frames']};contract=json.load(open(D/'history/phase_v/results/input_contract.json'));factor=GeometryFactor(contract['extrinsic']['lever_body_imu_m']);query=TrackedCorrespondence(np.fromfile(D/'inputs/map_target.bin','<f4',offset=8).reshape(-1,3).astype(float));tests=[]
for ordinal in [0,1,3,193,215]:
 e=old['lidar_updates'][ordinal];prior=state(e['predicted']);P=np.array(e['predicted']['P']);source=np.load(D/'history/phase_v'/frames[e['header_ns']]['source_file']);solver=QualifiedUpdate(factor);x,Q,info=solver.correct(prior,P,source,query)
 expected=byordinal[ordinal]['decision']['accepted'] if e['accepted'] else False
 assert info['accepted']==expected
 if not expected:assert np.array_equal(x.p,prior.p) and np.array_equal(x.v,prior.v) and np.array_equal(Q,P)
 else:assert np.max(abs(Q-np.array(e['corrected']['P'])))<1e-9 and np.max(abs(x.v-np.array(e['corrected']['v_imu_map'])))<1e-9
 tests.append({'ordinal':ordinal,'pass':True,'old_accepted':e['accepted'],'new_accepted':info['accepted'],'reason':info['reason'],'qualification':info['qualification'],'wall_ms':info['wall_ms']})
dump(out,tests);print(tests)
