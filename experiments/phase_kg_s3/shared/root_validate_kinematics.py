"""Coordinator's independent matrix-exponential and availability checks.

Uses scipy's general 4x4 exponential, not the predictor's closed-form Jacobian,
and hashes the reviewed source. No GPU, navigation reference or bag is used.
"""
import argparse, hashlib, importlib, json, sys
from pathlib import Path
import numpy as np
from scipy.linalg import expm
from scipy.spatial.transform import Rotation


def validate(module_path):
    sys.path.insert(0,str(module_path));m=importlib.import_module('kinematic_predictor')
    rng=np.random.default_rng(713);maximum=0.
    for _ in range(300):
        w=rng.normal(size=3)*2;s=rng.uniform(-30,100);dt=rng.uniform(0,.1)
        T=np.eye(4);T[:3,:3]=Rotation.random(random_state=rng).as_matrix();T[:3,3]=rng.normal(size=3)*20
        A=np.zeros((4,4));A[:3,:3]=[[0,-w[2],w[1]],[w[2],0,-w[0]],[-w[1],w[0],0]];A[0,3]=s
        maximum=max(maximum,float(abs(m.twist_step(T,w,s,dt)-T@expm(A*dt)).max()))
    if maximum>1e-11:raise AssertionError('GENERAL_SE3_EXPONENTIAL_MISMATCH')
    p=m.IntervalKinematicPredictor(0)
    for t in range(0,50_000_001,5_000_000):
        p.ingest(dict(kind='imu',header_ns=t,available_ns=t,message_id=t,omega_base=[.1,.2,.3]))
        if t%10_000_000==0:p.ingest(dict(kind='wheel',header_ns=t,available_ns=t,message_id=t,speed_base=20.,variance=.01))
    a=p.query(0,50_000_000,50_000_000,50_000_000)
    p.ingest(dict(kind='wheel',header_ns=25_000_000,available_ns=100_000_000,message_id=999,speed_base=99.,variance=.01))
    b=p.query(0,50_000_000,50_000_000,50_000_000)
    if a['status']!='VALID' or not np.array_equal(a['delta_T'],b['delta_T']):raise AssertionError('FUTURE_AVAILABLE_CHANGED_OLD_QUERY')
    return dict(kind='root independent CPU numerical review',random_seed=713,general_expm_cases=300,
        max_absolute_matrix_error=maximum,future_available_query_unchanged=True,
        source_sha256=hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest(),
        GPU_used=False,reference_used=False,continuous_replay_validated=False)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('module_path',type=Path);p.add_argument('--output',type=Path);a=p.parse_args()
    s=json.dumps(validate(a.module_path),indent=2)
    if a.output:a.output.write_text(s+'\n')
    print(s)
