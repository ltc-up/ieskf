import numpy as np
import pytest
from geometry_ieskf.state import State,difference
from geometry_ieskf.external_pose import ExternalPose
from ndt_global.managed_replay import ManagedReplay
from ndt_global.multihypothesis import GaussianBank,ContinuousMapOdom

def replay():
    x=State(np.zeros(3),np.array([2.,0.,0.]),np.eye(3),np.zeros(3),np.zeros(3),0)
    return ManagedReplay(x,np.eye(15)*.1,dict(header_ns=0,omega=np.zeros(3),force=np.array([0.,0.,9.80665])),np.zeros(3))
def imu(t):return dict(kind='imu',header_ns=t,message_id=t,omega=np.zeros(3),force=np.array([0.,0.,9.80665]))
def obs(t,uid='pose',dx=.1):
    T=np.eye(4);T[0,3]=2*t*1e-9+dx
    return dict(kind='ndt',header_ns=t,message_id=999,observation_id=uid,correlation_group=uid,observer=ExternalPose(np.zeros(3)),pose=T,R=np.eye(4)*.25)
def equal(a,b):
    x,P,_=a.current();y,Q,_=b.current();np.testing.assert_allclose(difference(x,y),0,atol=1e-12);np.testing.assert_allclose(P,Q,atol=1e-12)
def test_weight_normalization_and_group_once():
    g=GaussianBank(replay());g.fork('ndt','gicp','gicp');assert g.evidence('scan',{'ndt':.2,'gicp':1.})
    w=g.weights.copy();assert not g.evidence('scan',{'ndt':4.,'gicp':0.});np.testing.assert_equal(w,g.weights);assert g.weights.sum()==pytest.approx(1.)
def test_branch_full_state_history_independence():
    g=GaussianBank(replay());b=g.fork('ndt','gicp','gicp');g.branches[0].replay.ingest(imu(10_000_000));assert b.replay.current()[0].t==0
    b.replay.current()[1][:]=5;assert not np.all(g.branches[0].replay.current()[1]==5)
def test_rejected_id_cannot_resurrect():
    r=replay();r.reject('bad',10_000_000);r.ingest(imu(10_000_000));before=r.fork();assert r.ingest(obs(10_000_000,'bad'))=='DUPLICATE_TERMINAL';equal(r,before)
def test_duplicate_accepted_once():
    r=replay();r.ingest(imu(10_000_000));e=obs(10_000_000);r.ingest(e);before=r.fork();assert r.ingest(e)=='DUPLICATE_TERMINAL';equal(r,before)
def test_retraction_replays_all_inputs_exact():
    a,b=replay(),replay()
    for t in [10_000_000,20_000_000,30_000_000]:
        a.ingest(imu(t));b.ingest(imu(t))
        if t==10_000_000:a.ingest(obs(t,'bad'))
        if t==20_000_000:a.ingest(obs(t,'good'));b.ingest(obs(t,'good'))
    a.retract('bad');equal(a,b);assert a.records['bad'].status=='REJECTED';assert a.validate()
def test_delayed_event_same_timestamp_solution():
    a,b=replay(),replay()
    for t in [10_000_000,20_000_000,30_000_000]:
        a.ingest(imu(t));b.ingest(imu(t))
        if t==10_000_000:a.ingest(obs(t))
    b.ingest(obs(10_000_000));equal(a,b);assert b.records['pose'].stamp_ns==10_000_000

def test_expired_no_fake_timestamp():
    r=replay()
    for i in range(1,70):r.ingest(imu(i*10_000_000))
    assert r.ingest(obs(10_000_000))=='OUTSIDE_CHECKPOINT_HISTORY';assert r.records['pose'].status=='EXPIRED'
def test_retraction_outside_window_no_state_change():
    r=replay();r.ingest(imu(10_000_000));r.ingest(obs(10_000_000))
    for i in range(2,70):r.ingest(imu(i*10_000_000))
    before=r.fork()
    with pytest.raises(ValueError,match='OUTSIDE_HISTORY'):r.retract('pose')
    equal(r,before)
def test_invalid_covariance_terminal_rejection():
    r=replay();r.ingest(imu(10_000_000));e=obs(10_000_000);e['R']=-e['R'];before=r.fork();r.ingest(e);equal(r,before);assert r.records['pose'].status=='REJECTED'
def test_no_mean_between_modes_and_hysteresis():
    g=GaussianBank(replay());b=g.fork('ndt','gicp','gicp');b.replay.base[0].p[0]=10.
    g.evidence('1',{'ndt':4.,'gicp':0.})
    assert g.select(1).uid=='ndt';g.select(2);chosen=g.select(3);assert chosen.uid=='gicp';assert chosen.replay.current()[0].p[0]==10.
    with pytest.raises(ValueError,match='NO_MERGE'):g.merge_same_mode('gicp','ndt')
def test_frame_switch_keeps_odom_and_path_pose_relative():
    f=ContinuousMapOdom();O=np.eye(4);O[0,3]=20;M=O.copy();M[1,3]=3
    packet=f.correct(M,O);np.testing.assert_equal(packet['odom_from_base'],O)
    local=f.path_in_odom([[20,4,0]],packet['version']);np.testing.assert_allclose(local[0]-O[:3,3],[0,1,0])
    with pytest.raises(ValueError,match='VERSION'):f.path_in_odom([[0,0,0]],0)
def test_gnss_off_all_unauthorized_sources_blocked():
    r=replay()
    for kind in ['gnss','gps','pose','heading_gnss']:
        with pytest.raises(ValueError,match='UNAUTHORIZED'):r.ingest(dict(kind=kind,header_ns=10,message_id=1))
    with pytest.raises(ValueError,match='GNSS_DISABLED'):r.ingest(dict(imu(10),source='GNSS_INS'))
def test_information_group_not_second_observation():
    r=replay();r.ingest(imu(10_000_000));e=obs(10_000_000);r.ingest(e);before=r.fork();z=obs(10_000_000,'new');z['correlation_group']='pose';r.ingest(z);equal(r,before);assert r.records['new'].status=='REJECTED'

def test_terminal_ledger_does_not_become_pending_scan():
    r=replay()
    for i in range(10000):r.reject(str(i),i)
    r.ingest(imu(10_000_000));assert not r.pending_ids;assert len(r.records)==10000

def test_cloud_ack_before_buffer_close(tmp_path):
    import importlib.util, pathlib,threading,time,struct
    file=pathlib.Path(__file__).parents[1]/'experiments/phase_be_e0_e4/runtime.py'
    spec=importlib.util.spec_from_file_location('be_runtime',file);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    path=tmp_path/'delayed.bin'
    def writer():
        with path.open('wb') as f:
            f.write(struct.pack('<Q',2));f.write(np.array([1.,2.,3.],'<f4').tobytes());f.flush();time.sleep(.01);f.write(np.array([4.,5.,6.],'<f4').tobytes())
    thread=threading.Thread(target=writer);thread.start()
    while not path.exists():time.sleep(.0001)
    np.testing.assert_allclose(mod.read_cloud(path),[[1,2,3],[4,5,6]]);thread.join()

def test_revoked_weight_evidence_replay_equals_never_admitted():
    a,b=GaussianBank(replay()),GaussianBank(replay())
    for g in (a,b):g.fork('ndt','gicp','gicp')
    a.evidence('bad',{'ndt':3.,'gicp':0.})
    for g in (a,b):g.evidence('good',{'ndt':.5,'gicp':1.})
    assert a.revoke_evidence('bad');np.testing.assert_allclose(a.weights,b.weights,atol=1e-15)
    assert not a.evidence('bad',{'ndt':0.,'gicp':4.})

def test_same_pose_with_different_bias_is_not_same_mode():
    bank=GaussianBank(replay());branch=bank.fork('ndt','gicp','gicp')
    branch.replay.base[0].bg[1]=.1
    with pytest.raises(ValueError,match='DISTINCT_MODES'):bank.merge_same_mode('ndt','gicp')
    assert len(bank.branches)==2

def test_merge_with_evidence_requires_explicit_new_epoch_atomically():
    bank=GaussianBank(replay());bank.fork('ndt','gicp','gicp');bank.evidence('scan',{'ndt':1.,'gicp':1.})
    before=bank.weights.copy()
    with pytest.raises(ValueError,match='NEW_EPOCH'):bank.merge_same_mode('ndt','gicp')
    np.testing.assert_array_equal(bank.weights,before);assert len(bank.branches)==2

def test_legacy_comparison_replay_matches_original_all_state_and_covariance():
    import copy
    from ndt_global.optical_replay import OpticalReplay
    from ndt_global.managed_replay import LegacyComparisonReplay
    r=replay();x,P,imu0=r.current();a=OpticalReplay(x,P,imu0,np.zeros(3));b=LegacyComparisonReplay(x,P,imu0,np.zeros(3))
    events=[imu(10_000_000),obs(10_000_000,'one'),imu(20_000_000),imu(30_000_000),obs(20_000_000,'late')]
    for e in events:a.ingest(copy.deepcopy(e));b.ingest(copy.deepcopy(e));equal(a,b)
    assert not b.records
    with pytest.raises(ValueError,match='NO_RETRACTION'):b.retract('one')

def test_retraction_revokes_later_update_that_leaves_its_valid_domain():
    class DomainLimitedPose(ExternalPose):
        def correct(self,x,P,T,R,projection=None):
            if x.p[0]<.05:return x.copy(),P.copy(),dict(accepted=False,reason='TEST_DOMAIN')
            return super().correct(x,P,T,R,projection)
    a,b=replay(),replay()
    for t in [10_000_000,20_000_000,30_000_000]:
        a.ingest(imu(t));b.ingest(imu(t))
        if t==10_000_000:a.ingest(obs(t,'first'))
        if t==20_000_000:
            e=obs(t,'later');e['observer']=DomainLimitedPose(np.zeros(3));a.ingest(e);assert a.records['later'].status=='ACCEPTED'
    a.retract('first');equal(a,b)
    assert a.records['later'].status=='REJECTED'
    assert a.records['later'].reason=='REPLAY_INVALID_AFTER_REVOCATION'
    assert {r['uid'] for r in a.revision_log}=={'first','later'}
