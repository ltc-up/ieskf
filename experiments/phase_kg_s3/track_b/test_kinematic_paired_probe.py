import unittest
import numpy as np
from kinematic_paired_probe import FixedHistoryPairProbe
from local_odometry import Scan,cloud_hash
from kinematic_predictor import IntervalKinematicPredictor

class PairTests(unittest.TestCase):
 def test_same_target_double_seed_and_isolated_roi_columns(self):
  from types import SimpleNamespace
  p=IntervalKinematicPredictor(0)
  for k in ('imu','wheel'):
   p.ingest(dict(kind=k,header_ns=0,available_ns=0,message_id=1,**({'omega_base':[0,0,0]} if k=='imu' else {'speed_base':2.,'variance':0})))
  a=np.array([[1.,2.,3.]]);calls=[];logs=[]
  def matcher(r):
   calls.append(r);return dict(O_T_base=r['initial_guess'].tolist(),candidate_valid=True,converged=True,target_valid_voxels=20,neighbor_points=100)
  chain=SimpleNamespace(previous=SimpleNamespace(scan=SimpleNamespace(ref_ns=0)),O=np.eye(4),_target=lambda scan,seed:(a+1,[],[{'fixed_anchor':True}]))
  probe=FixedHistoryPairProbe(matcher,chain,p,logs.append)
  scan=Scan.create('2',20_000_000,20_000_000,np.ones((100,3)),np.ones((100,3)),'h')
  req=dict(source=scan.source,source_identity=scan.source_identity,target=a,target_identity=cloud_hash(a),initial_guess=np.eye(4),frame_id='2')
  probe.arm(scan,['case']);result=probe(req);probe.end_frame()
  self.assertEqual(len(calls),4);self.assertEqual(len(logs[0]['cells']),4)
  self.assertIs(calls[0]['source'],calls[1]['source']);self.assertIs(calls[0]['target'],calls[1]['target'])
  self.assertIs(calls[2]['target'],calls[3]['target']);self.assertEqual(calls[1]['initial_guess'][0,3],.04)
  np.testing.assert_array_equal(chain.O,np.eye(4));self.assertNotIn('validation',result)
 def test_delay_never_emits_seed_or_additional_match(self):
  from types import SimpleNamespace
  p=IntervalKinematicPredictor(0);calls=[];logs=[]
  c=SimpleNamespace(previous=SimpleNamespace(scan=SimpleNamespace(ref_ns=0)),O=np.eye(4))
  def matcher(r):calls.append(r);return {'untouched':True}
  f=FixedHistoryPairProbe(matcher,c,p,logs.append);s=Scan.create('1',20,20,np.ones((1,3)),np.ones((1,3)),'h')
  f.arm(s,['case']);r=f({});self.assertEqual(len(calls),1);self.assertEqual(logs[0]['cells'],[]);self.assertTrue(r['untouched'])

if __name__=='__main__':unittest.main()
