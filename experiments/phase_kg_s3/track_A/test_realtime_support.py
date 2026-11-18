import sys,unittest,copy
from pathlib import Path
import numpy as np
sys.path[:0]=[str(Path(__file__).resolve().parents[3]/'src'),str(Path(__file__).parent)]
from geometry_ieskf.state import State
from score_map import prior_identity
from realtime_support import PreparedGeometry,HistoricalGeometryRequiresAsyncRebuild,ExpiredHistory,replay_prior,AttemptCost
from ndt_global.managed_replay import ManagedReplay

class PreparedTest(unittest.TestCase):
 def setUp(self):
  self.x=State(np.zeros(3),np.array([1.,0,0]),np.eye(3),np.zeros(3),np.zeros(3),10_000_000)
  self.P=np.eye(15)*.04
 def test_all_retries_and_wait_time_count_in_complete_cycle(self):
  cost=AttemptCost()
  cost.add(dict(worker_begin_ns=100_000_000,worker_wall_ms=10,align_ms=4,geometry_ms=5,deskew_ms=1))
  cost.add(dict(worker_begin_ns=150_000_000,worker_wall_ms=20,align_ms=7,geometry_ms=11,deskew_ms=2))
  self.assertEqual(cost.elapsed_ms(200_000_000),100.)
  self.assertEqual(cost.totals,dict(worker_wall_ms=30.,align_ms=11.,geometry_ms=16.,deskew_ms=3.,attempts=2))
 def test_expired_reference_has_explicit_history_reason(self):
  imu=dict(header_ns=self.x.t,omega=np.zeros(3),force=np.array([0,0,9.81]))
  replay=ManagedReplay(self.x,self.P,imu,np.zeros(3))
  with self.assertRaisesRegex(ExpiredHistory,'EXPIRED_HISTORY'):replay_prior(replay,self.x.t,-1)
 def test_exact_prior_commit_and_nanosecond_change_reject(self):
  y=self.x.copy();y.p[0]=.2;Q=self.P*.9;info=dict(accepted=True,reason='SYNTHETIC_TRANSACTION',covariance_updates=1)
  obs=PreparedGeometry('scan1',prior_identity(self.x,self.P),y,Q,info)
  z,R,record=obs.correct(self.x,self.P,None);np.testing.assert_array_equal(z.p,y.p);np.testing.assert_array_equal(R,Q)
  x2=self.x.copy();x2.t+=1
  with self.assertRaises(HistoricalGeometryRequiresAsyncRebuild):obs.correct(x2,self.P,None)
 def test_deepcopy_has_no_mutable_posterior_leak(self):
  obs=PreparedGeometry('scan1',prior_identity(self.x,self.P),self.x,self.P,dict(accepted=True,covariance_updates=1))
  a=copy.deepcopy(obs);y,Q,info=a.correct(self.x,self.P,None);Q[:]=0;y.p[:]=10;info['accepted']=False
  z,R,again=obs.correct(self.x,self.P,None);np.testing.assert_array_equal(R,self.P);np.testing.assert_array_equal(z.p,self.x.p);self.assertTrue(again['accepted'])
 def test_dedup_and_post_commit_changed_prior_fail_explicitly(self):
  x=self.x.copy();x.t=0;imu=dict(header_ns=0,omega=np.zeros(3),force=np.array([0,0,9.81]));replay=ManagedReplay(x,self.P,imu,np.zeros(3))
  replay.ingest(dict(kind='imu',header_ns=10_000_000,message_id=1,omega=np.zeros(3),force=np.array([0,0,9.81])))
  prior,P,_=replay.current();y=prior.copy();y.p[0]+=.2
  obs=PreparedGeometry('scan1',prior_identity(prior,P),y,P,dict(accepted=True,reason='SYNTHETIC',covariance_updates=1))
  e=dict(kind='ndt',header_ns=10_000_000,message_id=2,observation_id='scan1',correlation_group='scan1',observer=obs,pose=np.eye(4),R=None)
  replay.ingest(e);self.assertEqual(replay.ingest(e),'DUPLICATE_TERMINAL')
  with self.assertRaises(HistoricalGeometryRequiresAsyncRebuild):replay.ingest(dict(kind='imu',header_ns=5_000_000,message_id=3,omega=np.array([0,0,.01]),force=np.array([0,0,9.81])))

if __name__=='__main__':unittest.main()
