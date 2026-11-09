import unittest,json
from pathlib import Path
import numpy as np
from geometry_ieskf.state import State,inject
from geometry_ieskf.external_pose import ExternalPose
from geometry_ieskf.ndt_packet_observation import packet_covariance,assess_integrity,InnovationGuardedPose
from geometry_ieskf.ndt_pose_observation import engineering_R
from ndt_global.pose_delay import PoseDelayReplay
class NdtPacketTests(unittest.TestCase):
 def setUp(self):
  self.config=json.loads((Path(__file__).parents[1]/'experiments/phase_av_filter_improvement/candidate.json').read_text());self.x=State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3),1_000_000_000);self.P=np.eye(15)*.01;self.l=np.array([1.,.2,.1]);self.T=np.eye(4);self.T[:3,3]=-self.l+[.01,0,0];self.components=('x','y','z','rx','ry','rz')
 def h(self,T=None,converged=True):return dict(final=(self.T if T is None else T).tolist(),candidate_valid=True,converged=converged,fitness=.01)
 def test_static_information_budget_repeated_vs_single(self):
  rng=np.random.default_rng(10);A=rng.normal(size=(15,15));P=A@A.T+np.eye(15);H=rng.normal(size=(6,15));R=np.diag(np.arange(1,7));r=rng.normal(size=6)
  def update(x,P,R):
   K=np.linalg.solve(H@P@H.T+R,H@P).T;B=np.eye(15)-K@H;return x+K@(r-H@x),B@P@B.T+K@R@K.T
  one,Q=update(np.zeros(15),P,R);x=np.zeros(15);S=P.copy()
  for _ in range(5):x,S=update(x,S,5*R)
  np.testing.assert_allclose(x,one,atol=1e-13);np.testing.assert_allclose(S,Q,atol=1e-13);_,wrong=update(np.zeros(15),P,5*R);self.assertGreater(np.trace(wrong),np.trace(Q))
 def test_legacy_default_unchanged(self):
  f=1.2;old=engineering_R(f);np.testing.assert_allclose(old,np.diag([.3,.3,.0015]));np.testing.assert_allclose(packet_covariance(f,self.components)*5,np.diag([old[0,0]]*3+[old[2,2]]*3))
 def test_guard_matches_original_when_consistent(self):
  R=packet_covariance(1.,self.components);g=InnovationGuardedPose(self.l,self.components);o=ExternalPose(self.l,self.components);y,Q,info=g.correct(self.x,self.P,self.T,R);z,S,_=o.correct(self.x,self.P,self.T,R);self.assertTrue(info['accepted']);np.testing.assert_array_equal(y.p,z.p);np.testing.assert_array_equal(Q,S);self.assertGreater(np.linalg.eigvalsh(Q)[0],0)
 def test_low_fitness_far_outlier_rejected_without_clipping(self):
  T=self.T.copy();T[0,3]+=30;g=InnovationGuardedPose(self.l,self.components);y,Q,info=g.correct(self.x,self.P,T,packet_covariance(.01,self.components));self.assertFalse(info['accepted']);self.assertEqual(info['reason'],'AV_INNOVATION_INCONSISTENT');np.testing.assert_array_equal(Q,self.P);np.testing.assert_array_equal(y.p,self.x.p)
 def test_ambiguity_not_averaged(self):
  T=self.T.copy();T[1,3]+=3;v=assess_integrity(self.h(),[self.h(),self.h(T),self.h()],dict(rank=6,nearby_fraction=1),self.config);self.assertFalse(v['eligible']);self.assertEqual(v['reason'],'AMBIGUOUS')
 def test_partial_geometry_withheld(self):
  v=assess_integrity(self.h(),[self.h()]*3,dict(rank=3,nearby_fraction=1),self.config);self.assertFalse(v['eligible']);self.assertEqual(v['reason'],'PARTIAL_OR_WEAK_GEOMETRY')
 def test_map_support_is_separate(self):
  v=assess_integrity(self.h(),[self.h()]*3,dict(rank=6,nearby_fraction=.1),self.config);self.assertEqual(v['reason'],'MAP_SUPPORT_INSUFFICIENT')
 def test_incomplete_probes_not_called_ambiguity(self):
  v=assess_integrity(self.h(),[self.h(),self.h(converged=False),self.h()],dict(rank=6,nearby_fraction=1),self.config);self.assertEqual(v['reason'],'INSUFFICIENT_HYPOTHESES')
 def test_consensus_does_not_certify_global_peak(self):
  v=assess_integrity(self.h(),[self.h()]*3,dict(rank=6,nearby_fraction=1),self.config);self.assertTrue(v['eligible']);self.assertEqual(v['global_peak_correctness'],'UNKNOWN')
 def test_unchanged_full_pose_jacobian(self):
  g=InnovationGuardedPose(self.l,self.components);_,H=g.model(self.x,self.x.R);eps=1e-7;fd=np.column_stack([(g.model(inject(self.x,eps*np.eye(15)[i]),self.x.R)[0]-g.model(self.x,self.x.R)[0])/eps for i in range(15)]);np.testing.assert_allclose(H,fd,atol=1e-7)
 def test_delayed_unique_group_not_fused_twice(self):
  im=dict(header_ns=self.x.t,omega=np.zeros(3),force=np.array([0.,0,9.80665]));r=PoseDelayReplay(self.x,self.P,im,self.l)
  r.ingest(dict(kind='imu',header_ns=self.x.t+10_000_000,message_id=1,omega=im['omega'],force=im['force']));r.ingest(dict(kind='clock',header_ns=self.x.t+15_000_000,message_id=2));r.ingest(dict(kind='imu',header_ns=self.x.t+20_000_000,message_id=3,omega=im['omega'],force=im['force']));e=dict(kind='ndt',header_ns=self.x.t+15_000_000,message_id=4,pose=self.T,R=packet_covariance(1.,self.components),observer=InnovationGuardedPose(self.l,self.components),correlation_group='unique');r.ingest(e);x,P,_=r.current();self.assertEqual(r.ingest(e|dict(message_id=5)),'DUPLICATE_INFORMATION_GROUP');y,Q,_=r.current();np.testing.assert_array_equal(x.p,y.p);np.testing.assert_array_equal(P,Q)
 def test_invalid_noise_keeps_prior(self):
  y,Q,info=InnovationGuardedPose(self.l,self.components).correct(self.x,self.P,self.T,np.zeros((6,6)));self.assertFalse(info['accepted']);np.testing.assert_array_equal(Q,self.P)
