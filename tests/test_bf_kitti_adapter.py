import importlib.util
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
p=Path(__file__).parents[1]/'experiments/phase_bf_public_dataset_validation/kitti_adapter.py'
spec=importlib.util.spec_from_file_location('bf_kitti_adapter',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def test_fixed_split_disjoint():
    assert m.MAP_IDS==tuple(range(20)) and m.TEST_IDS==tuple(range(20,77))
    assert not set(m.MAP_IDS)&set(m.TEST_IDS)

def test_ns_precision(tmp_path):
    p=tmp_path/'t';p.write_text('2011-09-26 13:02:25.123456789\n2011-09-26 13:02:25.133456790\n')
    assert np.diff(m.timestamps(p))[0]==10000001
    p.write_text('2011-09-26 13:02:25\n2011-09-26 13:02:25\n')
    with pytest.raises(ValueError,match='NON_MONOTONIC'):m.timestamps(p)

def test_reference_chart_not_ndt_intrinsic():
    v=np.zeros((2,30));v[:,0]=49;v[:,1]=8;v[:,2]=100;v[:,3:6]=[.12,-.08,.9]
    T=m.reference_poses(v)
    np.testing.assert_allclose(T[0,:3,:3],Rotation.from_euler('xyz',[.12,-.08,.9]).as_matrix())
    assert not np.allclose(T[0,:3,:3],Rotation.from_euler('XYZ',[.12,-.08,.9]).as_matrix())
    np.testing.assert_equal(T[0,:3,3],[0,0,0])

def test_extrinsic_invert_and_points(tmp_path):
    R=Rotation.from_rotvec([.03,.02,-.1]).as_matrix();t=np.array([.3,-.7,.5]);p=tmp_path/'cal'
    p.write_text('R: '+' '.join(map(str,R.ravel()))+'\nT: '+' '.join(map(str,t))+'\n')
    T=m.calibration(p);pts=np.array([[1.,2.,3.],[-2,4,1]])
    np.testing.assert_allclose(m.transform(m.transform(pts,T),np.linalg.inv(T)),pts,atol=1e-14)

def test_no_fake_point_time_and_stride(tmp_path):
    p=tmp_path/'p';p.write_bytes(b'x')
    with pytest.raises(ValueError,match='STRIDE'):m.read_velodyne(p)
    np.array([[1,2,3,.5]],'<f4').tofile(p)
    assert m.read_velodyne(p).shape==(1,4)

def test_online_schema_excludes_reference_fields():
    source=p.read_text();online=source.split('online=dict(',1)[1].split('np.savez_compressed',1)[0]
    assert 'nav_packets' not in online and 'offline_reference' not in online


def test_reference_timestamp_alignment_and_no_test_leak():
    t=np.arange(100,dtype=np.int64)*10000000+1317042164000000000
    v=np.zeros((100,30));v[:,0]=49;v[:,1]=8+np.arange(100)*1e-6;v[:,2]=100
    q=t[[0,10,20]]+1234567
    a=m.reference_at(t,v,q)
    changed=v.copy();changed[30:,1]+=1
    np.testing.assert_allclose(a,m.reference_at(t,changed,q))
    assert a[1,0,3]>0
    with pytest.raises(ValueError,match='NO_BRACKET'):m.reference_at(t,v,[t[0]-1])

def test_runner_has_no_offline_reference_loader_or_gnss_update():
    s=p.with_name('run_kitti.py').read_text()
    assert "np.load(root/'online.npz')" in s
    assert 'np.load(root/\'offline_reference.npz\')' not in s
    assert 'range(20,77)' in s
    assert "C=False,A='shadow only'" in s
    assert 'legacy_R' in s and 'native_pose_R' in s


def test_legacy_six_matrix_keeps_be_baseline():
    from geometry_ieskf.ndt_pose_observation import engineering_R
    spec=importlib.util.spec_from_file_location('bf_runner',p.with_name('run_kitti.py'));r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)
    for fitness in (.01,.3,3.,10.):
        old=engineering_R(fitness)
        np.testing.assert_array_equal(r.legacy_R(fitness),np.diag([old[0,0]]*3+[old[2,2]]*3))
