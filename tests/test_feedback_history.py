import unittest,importlib.util
from pathlib import Path
import numpy as np
from geometry_ieskf.state import State
from geometry_ieskf.external_pose import ExternalPose
from ndt_global.ar_replay import AuditedPoseReplay
spec=importlib.util.spec_from_file_location('au_recover',Path(__file__).parents[1]/'experiments/phase_au_prediction_feedback/recover_history.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class FeedbackHistoryTests(unittest.TestCase):
 def setUp(self):
  self.x=State(np.zeros(3),np.array([5.,0,0]),np.eye(3),np.zeros(3),np.zeros(3),1_000_000_000);self.P=np.eye(15)*.01;self.imu=dict(header_ns=self.x.t,omega=np.zeros(3),force=np.array([0.,0,9.80665]));self.lever=np.array([1.,.2,.1])
 def test_state_pack_roundtrip(self):
  y=m.unpack(m.pack(self.x),self.x.t);np.testing.assert_array_equal(m.pack(y),m.pack(self.x));self.assertEqual(y.t,self.x.t)
 def test_trace_does_not_change_delayed_mean_covariance(self):
  rec=m.Recorder();r=m.TracedReplay(self.x,self.P,self.imu,self.lever,rec=rec);plain=AuditedPoseReplay(self.x,self.P,self.imu,self.lever)
  events=[dict(kind='imu',header_ns=1_010_000_000,message_id=1,omega=[0,0,.1],force=[0,0,9.8]),dict(kind='wheel',header_ns=1_012_000_000,message_id=2,speed=5.01,variance=.01),dict(kind='clock',header_ns=1_015_000_000,message_id=-1),dict(kind='imu',header_ns=1_020_000_000,message_id=3,omega=[0,0,.1],force=[0,0,9.8])]
  for e in events:r.ingest(dict(e));plain.ingest(dict(e))
  T=np.eye(4);T[:3,3]=[-.91,-.2,-.1];e=dict(kind='ndt',header_ns=1_015_000_000,message_id=9,pose=T,R=np.eye(6)*.1,correlation_group='one-cloud')
  r.ingest(e|dict(observer=m.TracedObserver(self.lever,rec)));plain.ingest(e|dict(observer=ExternalPose(self.lever,('x','y','z','rx','ry','rz'))))
  x,P,_=r.current();y,Q,_=plain.current();np.testing.assert_array_equal(m.pack(x),m.pack(y));np.testing.assert_array_equal(P,Q);self.assertGreater(len(rec.rows),0)
 def test_trace_records_full_covariance_and_inputs(self):
  rec=m.Recorder();r=m.TracedReplay(self.x,self.P,self.imu,self.lever,rec=rec);r.ingest(dict(kind='imu',header_ns=1_010_000_000,message_id=1,omega=[0,0,0],force=[0,0,9.8]));z=rec.rows[0];self.assertEqual(z['P0'].shape,(15,15));self.assertEqual(z['before'].shape,(21,));self.assertAlmostEqual(z['dt'],.01);np.testing.assert_array_equal(z['force'],self.imu['force'])
