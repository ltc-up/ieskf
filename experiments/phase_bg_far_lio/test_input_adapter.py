import importlib.util
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
s=importlib.util.spec_from_file_location('bg_adapter',Path(__file__).with_name('input_adapter.py'))
a=importlib.util.module_from_spec(s)
import sys
sys.modules[s.name]=a;s.loader.exec_module(a)


def make(sensor=0):
    T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',[.2,-.1,.7]).as_matrix();T[:3,3]=[1.4,.2,.04]
    return a.Cloud(np.array([[1.,2.,3.],[-2.,4.,1.]]),np.array([2.,3.]),np.array([-.025,.02],dtype='f4'),
                   1700000000000000000+sensor*10001,1700000000100000000,sensor,10+sensor,T)


def test_absolute_time_identity_and_static_geometry():
    clouds=[make(i) for i in range(3)];out,m=a.fuse_three(clouds)
    assert len(out)==6 and m['max_point_time_roundtrip_error_ns']<=2
    assert m['header_ns']!=clouds[0].header_ns+50_000_000
    assert not m['deskew_applied'] and m['available_ns']==max(c.available_ns for c in clouds)
    assert out['timestamp'].max()==0
    for i,c in enumerate(clouds):
        got=np.column_stack([out[k][2*i:2*i+2] for k in ['x','y','z']])
        np.testing.assert_allclose(got,c.xyz@c.T_base_sensor[:3,:3].T+c.T_base_sensor[:3,3],atol=2e-6)
        np.testing.assert_array_equal(out['intensity'][2*i:2*i+2],c.intensity)


@pytest.mark.parametrize('bad',['missing','duplicate_sensor','duplicate_id','future'])
def test_refuse_fusion_contract_failures(bad):
    c=[make(i) for i in range(3)]
    if bad=='missing':c=c[:2]
    if bad=='duplicate_sensor':c[2].sensor_id=1
    if bad=='duplicate_id':c[2].identity=c[0].identity
    if bad=='future':
        for x in c:x.available_ns=x.header_ns
    with pytest.raises(ValueError):a.fuse_three(c)


@pytest.mark.parametrize('endian',['<','>'])
def test_decode_actual_fields(endian):
    p=np.array([[1,2,3,-.025,4],[4,5,6,.02,7]],dtype=endian+'f4')
    fields={k:dict(offset=i*4,datatype=7,count=1) for i,k in enumerate(['x','y','z','timestamp','intensity'])}
    c=a.decode_cloud(p.tobytes(),fields,2,20,1000000000,1100000000,0,1,np.eye(4),endian=='>')
    np.testing.assert_allclose(c.xyz,p[:,:3]);np.testing.assert_allclose(c.offset_s,p[:,3])
    del fields['timestamp']
    with pytest.raises(ValueError):a.decode_cloud(p.tobytes(),fields,2,20,1,2,0,1,np.eye(4))


@pytest.mark.parametrize('angles',[[0,0,0],[0,0,np.pi],[.2,-.4,.7]])
def test_imu_reference_point_and_gravity(angles):
    T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',angles).as_matrix();T[:3,3]=[1.465,.172,.0403]
    w=np.array([.1,-.2,.9]);alpha=np.array([.3,.4,-.1]);force_base=np.array([2.,7.,9.5]);dt=.005
    force_imu=force_base+np.cross(alpha,T[:3,3])+np.cross(w,np.cross(w,T[:3,3]))
    prev=dict(stamp_ns=1_000_000_000,omega_base=w-alpha*dt,gyro_cov=np.eye(3)*1e-5)
    z=a.adapt_imu(T[:3,:3].T@w,T[:3,:3].T@force_imu,T,1_005_000_000,prev,np.eye(3)*1e-5,np.eye(3)*.01)
    np.testing.assert_allclose(z['force_base'],force_base,atol=1e-12)
    assert not z['gravity_removed'] and np.linalg.eigvalsh(z['force_cov']).min()>0
    with pytest.raises(ValueError):a.adapt_imu(w,force_base,T,1,None)
    with pytest.raises(ValueError):a.adapt_imu(w,force_base,T,prev['stamp_ns'],prev)


def test_imu_stationary_does_not_remove_gravity():
    T=np.eye(4);T[:3,3]=[1,.2,.1];prev=dict(stamp_ns=1000000,omega_base=np.zeros(3))
    z=a.adapt_imu([0,0,0],[0,0,9.81],T,6000000,prev)
    np.testing.assert_array_equal(z['force_base'],[0,0,9.81])


def test_missing_covariance_is_not_a_rotated_negative_matrix():
    T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',.7).as_matrix()
    marker=np.diag([-1.,0.,0.]);z=a.adapt_imu([0,0,0],[0,0,9.81],T,1,None,marker,marker)
    assert not z['gyro_cov_available'] and not z['force_cov_available']
    assert 'force_cov' not in z and 'gyro_cov' not in z
    assert a.covariance_or_none(np.zeros((3,3))) is None
    with pytest.raises(ValueError):a.covariance_or_none(np.diag([1.,-2.,1.]))


def test_lever_gyro_jacobian_against_finite_difference():
    T=np.eye(4);r=T[:3,3]=[1.4,.2,.04];w=np.array([.1,.2,.8]);v=np.array([.09,.21,.799]);dt=.005
    def f(x,y):
        z=a.adapt_imu(x,[1,2,9.8],T,5000000,dict(stamp_ns=0,omega_base=y))
        return z['force_base']
    Jc=a.skew(r)/dt+a.skew(np.cross(w,r))+a.skew(w)@a.skew(r);Jp=-a.skew(r)/dt
    eps=1e-6
    for j in range(3):
        e=np.eye(3)[j]*eps
        np.testing.assert_allclose((f(w+e,v)-f(w-e,v))/(2*eps),Jc[:,j],atol=1e-7)
        np.testing.assert_allclose((f(w,v+e)-f(w,v-e))/(2*eps),Jp[:,j],atol=1e-7)


def test_diagnostic_byte_serialization():
    from run_native_window import json_scalar
    import json
    assert json.loads(json.dumps({'level':b'\x02'},default=json_scalar))['level']==2
    assert json.loads(json.dumps({'value':np.int64(3)},default=json_scalar))['value']==3
    with pytest.raises(TypeError):json_scalar(object())
