import unittest
import numpy as np
from kinematic_bridge import *
from test_kinematic_predictor import sample
from kinematic_predictor import HeaderCoverageAudit, twist_step


class FakeGroup:
 def __init__(self,exe,scratch,modes):self.modes=modes;self.inputs=[]
 def on_frame(self,**r):
  self.inputs.append(r)
  return dict(variants={m:dict(status='INITIALIZED' if len(self.inputs)==1 else 'ACCEPTED',O_T_base=r['U_T_base'].tolist()) for m in self.modes})
 def close(self):pass


def frame(i,t,available=None,initial=None):
 r=dict(frame_id=i,ref_ns=t,available_ns=t if available is None else available,dense=np.array([[0.,0,1],[1,0,1]]),source=np.array([[0.,0,1]]),deskew_history='synthetic immutable source')
 if initial is not None:r['legacy_initial_O0']=initial
 return r


class Tests(unittest.TestCase):
 def group(self,**kw):return KinematicShadowGroup(0,'unused','unused',targets=('NONE','PREVIOUS'),group_factory=FakeGroup,**kw)
 def test_fixed_first_O_alignment_and_no_later_legacy_read(self):
  g=self.group();g.on_motion(sample('imu',0,value=[.3,.1,0]));g.on_motion(sample('wheel',0,value=2))
  O0=np.eye(4);c,s=np.cos(.2),np.sin(.2);O0[:3,:3]=[[c,0,s],[0,1,0],[-s,0,c]]
  r=g.on_frame(**frame(1,20_000_000,initial=O0))[0]
  np.testing.assert_allclose(g.group.inputs[0]['U_T_base'],O0,atol=2e-16)
  H=g.H_init.copy();g.on_motion(sample('imu',20_000_000))
  r=g.on_frame(**frame(2,40_000_000))[0];np.testing.assert_array_equal(g.H_init,H)
  self.assertTrue(all(x['predictor']=='KINEMATIC_BK' for x in r['variants'].values()))
  with self.assertRaises(ValueError):g.on_frame(**frame(3,50_000_000,initial=O0))
 def test_delay_keeps_original_source_and_retries_actual_available(self):
  audit=HeaderCoverageAudit([0,20_000_000,40_000_000],[0,100_000_000],0,100_000_000,'synthetic')
  g=self.group(audit=audit);g.on_motion(sample('imu',0));g.on_motion(sample('wheel',0))
  f=frame(1,40_000_000,50_000_000,np.eye(4));r=g.on_frame(**f)[0]
  self.assertEqual(r['status'],'DELAY');self.assertEqual(len(g.group.inputs),0)
  f['dense'][0,0]=999;f['source'][0,0]=999
  r=g.on_motion(sample('imu',20_000_000,a=60_000_000))[0]
  self.assertEqual(r['status'],'COMMITTED');self.assertEqual(r['query_available_ns'],60_000_000)
  self.assertEqual(r['original_cloud_available_ns'],50_000_000)
  self.assertEqual(g.group.inputs[0]['source'][0,0],0);self.assertEqual(g.group.inputs[0]['available_ns'],60_000_000)
 def test_later_frames_do_not_skip_pending_head(self):
  audit=HeaderCoverageAudit([0,20_000_000,40_000_000,60_000_000],[0,100_000_000],0,100_000_000,'synthetic')
  g=self.group(audit=audit);g.on_motion(sample('imu',0));g.on_motion(sample('wheel',0))
  g.on_frame(**frame(1,40_000_000,50_000_000,np.eye(4)))
  g.on_frame(**frame(2,50_000_000,55_000_000))
  g.on_motion(sample('imu',20_000_000,a=60_000_000))
  self.assertEqual([x['frame_id'] for x in g.group.inputs],['1'])
  g.on_motion(sample('imu',40_000_000,a=65_000_000))
  self.assertEqual([x['frame_id'] for x in g.group.inputs],['1','2'])
 def test_real_audited_gap_terminates_with_remaining_range(self):
  a=HeaderCoverageAudit([0,50_000_000],[0,100_000_000],0,100_000_000,'synthetic')
  g=self.group(audit=a,requested_end_ns=100_000_000);g.on_motion(sample('imu',0));g.on_motion(sample('wheel',0))
  r=g.on_frame(**frame(1,40_000_000,50_000_000,np.eye(4)))[0]
  self.assertEqual(r['status'],'IRRECOVERABLE_INPUT_GAP');self.assertEqual(r['remaining_requested_range_ns'],[0,100_000_000])
  self.assertEqual(len(g.group.inputs),0)
 def test_queue_resource_failure_is_not_gap(self):
  g=self.group(queue_bytes=1);g.on_motion(sample('imu',0));g.on_motion(sample('wheel',0))
  r=g.on_frame(**frame(1,20_000_000,initial=np.eye(4)))[0]
  self.assertEqual(r['status'],'RESOURCE_FAILURE');self.assertNotIn('GAP',r['reason'])
 def test_no_timeout_or_fake_zero_at_caller_end(self):
  g=self.group();g.on_motion(sample('imu',0));g.on_motion(sample('wheel',0));g.on_frame(**frame(1,40_000_000,50_000_000,np.eye(4)))
  r=g.finish(60_000_000);self.assertEqual(r[-1]['status'],'DELAY_UNRESOLVED_AT_CALLER_END');self.assertIsNone(g.terminal)
 def test_original_initial_axis_matches_legacy_definition(self):
  U=np.eye(4);U[:3,3]=[3,4,5];U[:3,:3]=twist_step(np.eye(4),[.2,.3,.4],0,.1)[:3,:3]
  O=original_legacy_initial_O0(U);np.testing.assert_allclose(O[:3,3],0,atol=1e-15)
  self.assertAlmostEqual(np.arctan2(O[1,0],O[0,0]),0)


class ResourceAndGaugeTests(unittest.TestCase):
 def test_pending_limit_separate_from_motion_sample_limit(self):
  g=KinematicShadowGroup(0,'unused','unused',group_factory=FakeGroup,maximum_pending_frames=1,maximum_motion_samples=20)
  g.on_motion(sample('imu',0));g.on_motion(sample('wheel',0));g.on_frame(**frame(1,40_000_000,50_000_000,np.eye(4)))
  r=g.on_frame(**frame(2,50_000_000,55_000_000))[0]
  self.assertEqual(r['status'],'RESOURCE_FAILURE');self.assertEqual(len(g.predictor.samples['imu']),1)
 def test_explicit_evaluation_gauge_does_not_change_motion(self):
  g=KinematicShadowGroup(0,'unused','unused',targets=('PREVIOUS',),group_factory=FakeGroup)
  g.on_motion(sample('imu',0));g.on_motion(sample('wheel',0))
  gauge=np.eye(4);gauge[:3,3]=[100,200,5]
  f=frame(1,20_000_000,initial=np.eye(4));f['legacy_initial_T_map_from_O']=gauge
  r=g.on_frame(**f)[0]
  np.testing.assert_array_equal(r['variants']['KINEMATIC_BK__PREVIOUS']['T_map_from_O'],gauge)
  np.testing.assert_allclose(g.group.inputs[0]['U_T_base'],np.eye(4),atol=1e-15)

class PrefixTests(unittest.TestCase):
 def test_complete_preinit_stream_keeps_only_required_held_prefix(self):
  g=KinematicShadowGroup(1_000_000_000,'unused','unused',targets=('NONE',),group_factory=FakeGroup,maximum_motion_samples=10)
  for h in range(0,1_000_000_000,10_000_000):
   self.assertEqual(g.on_motion(sample('imu',h)),[]);self.assertEqual(g.on_motion(sample('wheel',h)),[])
  self.assertIsNone(g.terminal);self.assertEqual(sum(map(len,g.predictor.samples.values())),2)


class FutureHeaderPrefixTests(unittest.TestCase):
 def test_preinit_pruning_preserves_already_arrived_future_header(self):
  g=KinematicShadowGroup(100,'unused','unused',targets=('NONE',),group_factory=FakeGroup,maximum_motion_samples=10)
  g.on_motion(sample('imu',120,a=10));g.on_motion(sample('imu',80,a=20));g.on_motion(sample('imu',90,a=30))
  self.assertEqual(sorted(x.header_ns for x in g.predictor.samples['imu']),[90,120])

if __name__=='__main__':unittest.main()
