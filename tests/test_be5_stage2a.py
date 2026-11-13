"""Synthetic models only. No sensor references, services, replay or state commits."""
import json
import os
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from geometry_ieskf.state import State, inject
from geometry_ieskf.vertical_velocity import fixed_normal_model, body_normal_model
from geometry_ieskf.attitude_reference import tilt_model, source_diagnostic
from geometry_ieskf.shadow_candidate import linear_shadow_candidate
from geometry_ieskf.pose_uncertainty import native_pose_R
from geometry_ieskf.ndt_pose_observation import engineering_R

METRICS = {'A1_max_absolute_FD_error': 0., 'A2_max_absolute_FD_error': 0.,
           'C_max_absolute_FD_error': 0., 'Native_max_absolute_covariance_FD_error': 0.,
           'Native_max_absolute_chart_Jacobian_FD_error': 0.,
           'synthetic_A_states': 0, 'synthetic_C_states': 0, 'all_15_columns': True}

@pytest.fixture(scope='session', autouse=True)
def save_metrics():
    yield
    p = os.environ.get('BE5_UNIT_METRICS')
    if p:
        Path(p).write_text(json.dumps(METRICS, indent=2))

def state(seed=0):
    rng = np.random.default_rng(seed)
    return State(rng.normal(size=3), np.array([53., 9., 2.])+rng.normal(size=3),
                 Rotation.from_euler('xyz', rng.uniform([-.45, -.5, -2.5], [.45, .5, 2.5])).as_matrix(),
                 rng.normal(size=3)*.02, rng.normal(size=3)*.1, 1234)

@pytest.mark.parametrize('seed', range(32))
def test_A_all_15_columns_and_three_steps(seed):
    rng = np.random.default_rng(seed+120)
    x = state(seed)
    lever = np.array([1.2, -.4, .3])+rng.normal(size=3)*.1
    omega = np.array([.15, -.12, .8])+rng.normal(size=3)*.1
    n = Rotation.from_euler('xy', rng.uniform(-.35, .35, 2)).apply([0., 0., 1.])
    cases = [('A1', lambda z: fixed_normal_model(z, omega, lever, n, normal_source='synthetic_fixed_slope')),
             ('A2', lambda z: body_normal_model(z, omega, lever))]
    for name, model in cases:
        h, H = model(x)
        for eps in (1e-4, 1e-5, 1e-6):
            fd = np.array([(model(inject(x, np.eye(15)[j]*eps))[0] -
                            model(inject(x, -np.eye(15)[j]*eps))[0])/(2*eps) for j in range(15)])
            error = float(np.max(abs(fd-H[0])))
            METRICS[name+'_max_absolute_FD_error'] = max(METRICS[name+'_max_absolute_FD_error'], error)
            np.testing.assert_allclose(fd, H[0], atol=1e-6, rtol=1e-6)
        np.testing.assert_array_equal(H[:, :3], 0)
        np.testing.assert_array_equal(H[:, 12:], 0)
    METRICS['synthetic_A_states'] += 1

def test_A2_is_not_A1_jacobian_with_frozen_normal():
    x = state(3);omega = np.array([.2, -.1, .8]);lever = np.array([1., .3, .2])
    h1,H1 = fixed_normal_model(x,omega,lever,x.R[:,2],normal_source='frozen_predicted_body_up')
    h2,H2 = body_normal_model(x,omega,lever)
    assert abs(h1-h2)<1e-12
    assert np.linalg.norm(H1[:,6:9]-H2[:,6:9])>10

def test_slope_not_forced_map_vz_zero():
    x = state();x.R = Rotation.from_euler('y', -.12).as_matrix();x.v=x.R@np.array([50.,0.,0.]);x.bg[:]=0
    assert abs(body_normal_model(x,np.zeros(3),[1.,.2,.1])[0])<1e-12
    assert abs(fixed_normal_model(x,np.zeros(3),[1.,.2,.1],[0.,0.,1.],normal_source='map_z_hypothesis')[0])>5

@pytest.mark.parametrize('n,source', [([0,0,2],'n'),([0,0,1],''),([0,0,np.nan],'n')])
def test_A_invalid_normal(n,source):
    with pytest.raises(ValueError):fixed_normal_model(state(),np.zeros(3),[1,0,0],n,normal_source=source)

@pytest.mark.parametrize('seed',range(16))
def test_C_residual_all_15_columns(seed):
    x=state(seed);Rref=x.R@Rotation.from_rotvec([.03,-.02,.01]).as_matrix()
    r,H,E=tilt_model(x,Rref)
    for eps in (1e-4,1e-5,1e-6):
        fd=np.column_stack([(tilt_model(inject(x,np.eye(15)[j]*eps),Rref)[0]-
                            tilt_model(inject(x,-np.eye(15)[j]*eps),Rref)[0])/(2*eps) for j in range(15)])
        error=float(np.max(abs(fd+H)))
        METRICS['C_max_absolute_FD_error']=max(METRICS['C_max_absolute_FD_error'],error)
        np.testing.assert_allclose(fd,-H,atol=1e-7,rtol=1e-6)
    np.testing.assert_allclose(H[:,6:9]@(x.R.T@[0,0,1]),0,atol=1e-14)
    METRICS['synthetic_C_states']+=1

def test_C_yaw_invariance_and_true_tilt_not_zeroed():
    x=state(9);Rref=x.R@Rotation.from_euler('xyz',[.03,-.02,.4]).as_matrix()
    r,H,_=tilt_model(x,Rref)
    z=x.copy();Z=Rotation.from_euler('z',1.3).as_matrix();z.R=Z@x.R
    rr,HH,_=tilt_model(z,Z@Rref)
    np.testing.assert_allclose(r,rr,atol=1e-14)
    np.testing.assert_allclose(H,HH,atol=1e-14)
    np.testing.assert_allclose(tilt_model(x,x.R)[0],0,atol=1e-14)

def test_C_antipodal_reference_rejected():
    x=state();x.R=Rotation.from_euler('x',np.pi).as_matrix()
    with pytest.raises(ValueError,match='HEMISPHERE'):tilt_model(x,np.eye(3))

@pytest.mark.parametrize('p',[None,{'uses_bosch':True},{'uses_gnss_ins':True},{'uses_self_deskew':True},
 {'source_id':'synthetic','timestamp_contract':'synthetic','covariance_provenance':'synthetic',
  'uses_bosch':False,'uses_gnss_ins':False,'uses_self_deskew':False}])
def test_C_source_never_enables_fusion(p):
    assert not source_diagnostic(p)['fusion_allowed']

def test_shadow_candidate_preserves_prior_and_full_P():
    rng=np.random.default_rng(4);A=rng.normal(size=(15,15));P=A@A.T+np.eye(15);before=P.copy()
    x=state(5);old=x.copy();h,H=body_normal_model(x,[.2,.1,.8],[1.,.2,.1]);R=np.array([[.25]])
    q=linear_shadow_candidate(P,H,[-h],R)
    np.testing.assert_array_equal(P,before)
    for k in ('p','v','R','bg','ba'):np.testing.assert_array_equal(getattr(x,k),getattr(old,k))
    assert not q['committed']
    np.testing.assert_array_equal(q['actual_applied_delta'],0)
    assert np.linalg.norm(q['P_change'])>0
    K=P@H.T@np.linalg.inv(H@P@H.T+R);B=q['reset'];J=np.eye(15)-K@H
    np.testing.assert_allclose(q['P_candidate'],B@(J@P@J.T+K@R@K.T)@B.T,atol=1e-12)
    assert abs(sum(q['vz_signed_innovation_contributions'])-q['delta_proposed'][5])<1e-12
    assert np.linalg.eigvalsh(q['P_candidate']).min()>0

def test_shadow_retains_indirect_vz_and_z_coupling():
    P=np.eye(15);P[2,5]=P[5,2]=.6;H=np.zeros((1,15));H[0,2]=1
    q=linear_shadow_candidate(P,H,[.2],[[.3]])
    assert q['delta_proposed'][5]>0
    assert abs(q['P_change'][2,5])>0

@pytest.mark.parametrize('R',[[[0.]],[[-1.]],[[np.nan]]])
def test_shadow_invalid_R_no_fallback(R):
    with pytest.raises((ValueError,np.linalg.LinAlgError)):
        linear_shadow_candidate(np.eye(15),np.eye(15)[:1],[1.],R)

def native():
    p=np.array([2.,3.,4.,.2,-.3,.4]);T=np.eye(4);T[:3,3]=p[:3];T[:3,:3]=Rotation.from_euler('XYZ',p[3:]).as_matrix()
    rng=np.random.default_rng(13);Q=rng.normal(size=(6,6));C=(Q@Q.T+np.eye(6))*.01
    return dict(parameters_internal=p,final=T,native_internal=C)

@pytest.mark.parametrize('fitness',[None,np.nan,100.,-1.])
def test_native_decoupled_from_all_legacy_fitness(fitness):
    n=native()
    if fitness is not None:n['fitness']=fitness
    C=native_pose_R(n,('x','y','z','rx','ry','rz'),state())
    np.linalg.cholesky(C)

@pytest.mark.parametrize('frame_rotation',[(0,0,0),(.2,-.15,.7)])
def test_native_full_correlated_covariance_map_and_log_FD(frame_rotation):
    n=native();x=state(4);A=np.eye(4);A[:3,:3]=Rotation.from_euler('xyz',frame_rotation).as_matrix();A[:3,3]=[10.,-4.,2.]
    p=n['parameters_internal']
    def obs(q):
        R=Rotation.from_euler('XYZ',q[3:]).as_matrix()
        return np.r_[A[:3,:3]@q[:3]+A[:3,3],Rotation.from_matrix(x.R.T@A[:3,:3]@R).as_rotvec()]
    eps=1e-6;M=np.column_stack([(obs(p+np.eye(6)[j]*eps)-obs(p-np.eye(6)[j]*eps))/(2*eps) for j in range(6)])
    from geometry_ieskf.pose_uncertainty import euler_xyz_to_body
    from geometry_ieskf.common import jr
    analytic=np.zeros((6,6));analytic[:3,:3]=A[:3,:3]
    analytic[3:,3:]=np.linalg.solve(jr(obs(p)[3:]),euler_xyz_to_body(p[3:]))
    jac_error=float(np.max(abs(M-analytic)))
    METRICS['Native_max_absolute_chart_Jacobian_FD_error']=max(METRICS['Native_max_absolute_chart_Jacobian_FD_error'],jac_error)
    np.testing.assert_allclose(M,analytic,atol=1e-8,rtol=1e-7)
    expected=M@n['native_internal']@M.T
    C=native_pose_R(n,('x','y','z','rx','ry','rz'),x,T_map_pcm=A)
    error=float(np.max(abs(expected-C)));METRICS['Native_max_absolute_covariance_FD_error']=max(METRICS['Native_max_absolute_covariance_FD_error'],error)
    np.testing.assert_allclose(C,expected,atol=2e-8,rtol=2e-7)
    assert np.max(abs(C-np.diag(np.diag(C))))>.001
    np.linalg.cholesky(C)

def test_native_yaw_chart_FD():
    n=native();x=state();p=n['parameters_internal'];eps=1e-6
    def obs(q):
        R=Rotation.from_euler('XYZ',q[3:]).as_matrix()
        return np.r_[q[:3],np.arctan2(R[1,0],R[0,0])]
    M=np.column_stack([(obs(p+np.eye(6)[j]*eps)-obs(p-np.eye(6)[j]*eps))/(2*eps) for j in range(6)])
    np.testing.assert_allclose(native_pose_R(n,('x','y','z','yaw'),x),M@n['native_internal']@M.T,atol=1e-8)

@pytest.mark.parametrize('mode',['asymmetric','indefinite','singular','nonfinite','pose_mismatch','bad_frame'])
def test_native_contract_failures_not_diagonalized(mode):
    n=native();A=np.eye(4)
    if mode=='asymmetric':n['native_internal'][0,1]+=.1
    if mode=='indefinite':
        n['native_internal']=np.eye(6);n['native_internal'][0,1]=n['native_internal'][1,0]=2.
    if mode=='singular':n['native_internal']=np.ones((6,6))
    if mode=='nonfinite':n['native_internal'][0,0]=np.nan
    if mode=='pose_mismatch':n['final'][0,3]+=1
    if mode=='bad_frame':A[0,0]=2
    with pytest.raises((ValueError,np.linalg.LinAlgError)):native_pose_R(n,('x','y','z','rx','ry','rz'),state(),A)

def test_legacy_original_golden_behavior_unchanged():
    np.testing.assert_allclose(engineering_R(.2),np.diag([.05,.05,.00025]),atol=1e-17,rtol=0)
    np.testing.assert_array_equal(engineering_R(4.),np.diag([30.,30.,12.]))
    with pytest.raises(ValueError):engineering_R(100.)

def test_real_frame_contract_fails_closed():
    import importlib.util
    p=Path(__file__).resolve().parents[1]/'experiments/phase_be5_stage2a/frozen_contract.py'
    spec=importlib.util.spec_from_file_location('be5_contract_test',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    x=state();n=native();lever=np.array([1.,.2,.1]);seed=np.eye(4);seed[:3,:3]=x.R;seed[:3,3]=x.p-x.R@lever
    n.update(header=x.t,seed_float=seed.astype(np.float32))
    with pytest.raises(ValueError,match='UNCONFIRMED'):m.qualify_frame(n,x,seed,x.t,lever,{'status':'UNKNOWN'})
    c=dict(status='CONFIRMED_SAVED_RESEARCH_NUMERIC_FRAME',direct_matching_map=True,base_source=True,
           native_parameters_and_covariance_same_chart=True,no_production_PCM_to_ENU_adapter=True)
    np.testing.assert_array_equal(m.qualify_frame(n,x,seed,x.t,lever,c),np.eye(4))
    with pytest.raises(ValueError,match='TIMESTAMP'):m.qualify_frame(n,x,seed,x.t+1,lever,c)
    with pytest.raises(ValueError,match='BASE_IMU'):m.qualify_frame(n,x,seed+np.diag([0,0,0,1]),x.t,lever,c)

@pytest.mark.parametrize('fault',['none','run','target','source'])
def test_dataset_certificate_is_bound_to_saved_run(fault):
    import importlib.util
    p=Path(__file__).resolve().parents[1]/'experiments/phase_be5_stage2a/frozen_contract.py'
    spec=importlib.util.spec_from_file_location('be5_dataset_binding',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    c=dict(status='CONFIRMED_SAVED_RESEARCH_NUMERIC_FRAME',allowed_run='saved',target_sha256='target',source_hashes={'source':'sha'})
    if fault=='run':c['allowed_run']='other'
    if fault=='target':c['target_sha256']='other'
    if fault=='source':c['source_hashes']={'source':'other'}
    if fault=='none':m.qualify_dataset_certificate(c,'saved',{'source':'sha'},'target')
    else:
        with pytest.raises(ValueError,match='DATASET_BINDING'):
            m.qualify_dataset_certificate(c,'saved',{'source':'sha'},'target')

def test_canonical_branch_and_published_times_are_not_conflated():
    import importlib.util
    p=Path(__file__).resolve().parents[1]/'experiments/phase_be5_stage2a/verify_cases.py'
    spec=importlib.util.spec_from_file_location('be5_time_labels',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    branch=dict(branch='ndt',prior={'p':[0,0,0]},ndt={'header':10},update={'R':[[1.]]},state={'t':12,'v':[1,2,3]})
    original=dict(ref=10,selected='ndt',branches=[branch],pose='synthetic published base pose',P='synthetic P',state_stamp=15)
    record=dict(header_ns=10,prior=branch['prior'],original_ndt=branch['ndt'],original_update=branch['update'],
                original_R=[[1.]],actual_published_state=branch['state'],actual_state_stamp=15)
    q=m.canonical_case(record,original)
    assert q['actual_branch_state_stamp']==12 and q['actual_published_state_stamp']==15
    assert q['actual_published_velocity'] is None
    assert 'actual_published_state' not in q
    assert record['actual_published_state']['t']==12
    with pytest.raises(ValueError,match='CHANGED'):m.canonical_case(dict(record,original_R=[[2.]]),original)
