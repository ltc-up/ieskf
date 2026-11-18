"""Identity-only map serialization and one-time preexisting-prior initialization."""
import argparse,hashlib,json,struct
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def main(a):
    root=Path(a.root);data=Path(a.data);out=root/'runtime/input_contract';out.mkdir(exist_ok=False)
    source=data/'phase_ar_loop44_map0105/map0105_target.bin';raw=source.read_bytes()
    if hashlib.sha256(raw).hexdigest()!='0895ad26af296aeba68b55350f7b9b70cd6b397be551fc78af75794410192968':raise ValueError('TARGET_IDENTITY')
    n=struct.unpack('<Q',raw[:8])[0];xyz=np.frombuffer(raw,dtype='<f4',offset=8).reshape(n,3)
    if len(raw)!=8+n*12 or not np.isfinite(xyz).all():raise ValueError('TARGET_LAYOUT')
    header=('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
            f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n').encode()
    target=out/'existing_target_xyz.pcd';target.write_bytes(header+raw[8:])
    restored=np.frombuffer(target.read_bytes(),dtype='<f4',offset=len(header)).reshape(n,3)
    if not np.array_equal(xyz,restored):raise ValueError('MAP_VALUES_CHANGED')
    motion=json.loads((data/'phase_ao_ap_validation/loop44.json').read_text())
    # Independently recorded T from prior audit, cross-checked against raw bag and bindings.
    Tbi=np.array(json.loads((root.parent/'phase_be_e0_e4/audit/be5_stage1/source_and_contract_evidence.json').read_text())['T_base_imu'])
    selection=json.loads((root/'audit/frozen_windows.json').read_text());wanted={w['warmup_indices'][0]:w['name'] for w in selection['windows']};init={}
    with Path(a.prior_records).open() as f:
        for line in f:
            row=json.loads(line)
            if row['index'] not in wanted:continue
            br=row['branches'][0];prior=br['prior'];R=np.array(prior['R']);p=np.array(prior['p'])-R@Tbi[:3,3]
            seed=np.array(br['seed']);T=np.eye(4);T[:3,:3]=R;T[:3,3]=p
            if not np.allclose(seed,T,atol=1e-9):raise ValueError('PRIOR_BASE_SEED_MISMATCH')
            init[wanted[row['index']]]=dict(index=row['index'],stamp_ns=row['ref'],position=p.tolist(),quaternion_xyzw=Rotation.from_matrix(R).as_quat().tolist(),source='Saved pre-NDT prior at the first warmup scan only; not per-frame OXTS/GNSS seed',bias_not_transferred=True)
    if len(init)!=2:raise ValueError('INITIAL_PRIOR_MISSING')
    (out/'initialization.json').write_text(json.dumps(init,indent=2)+'\n')
    result=dict(source_sha256=hashlib.sha256(raw).hexdigest(),pcd_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),points=n,bytes=target.stat().st_size,xyz_exactly_equal=True,format_conversion_only=True,occupied_4m_voxels=int(len(np.unique(np.floor(xyz/4).astype(np.int64),axis=0))),initialization_windows=list(init),map_origin_transform_applied=False)
    (out/'map_identity.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--data',required=True);p.add_argument('--prior-records',required=True);main(p.parse_args())
