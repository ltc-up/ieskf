import unittest,json,hashlib
from pathlib import Path
import numpy as np
from ndt_global.shadow import Contract,PoseHistory,ShadowCorrector,Candidate
from ndt_global.global_manager import GlobalManager,PositionObservation

class FrozenIntegrityTests(unittest.TestCase):
 def fixture(self):
  c=Contract('test','test','test');h=PoseHistory(c);h.add(1_000_000_000,np.eye(4),c);h.add(1_100_000_000,np.eye(4),c);return c,h,GlobalManager(h)
 def test_unknown_candidate_never_becomes_anchor(self):
  c,h,m=self.fixture();r=m.ndt([np.eye(4)],'UNKNOWN',1_000_000_000,'g',1_100_000_000,c,converged=True);self.assertFalse(r['accepted']);self.assertFalse(m.publish(1_100_000_000,1_100_000_000)['published'])
 def test_ambiguity_not_removed_by_unqualified_fix(self):
  c,h,m=self.fixture();T=np.eye(4);U=T.copy();U[0,3]=10;m.ndt([T,U],'AMBIGUOUS',1_000_000_000,'g',1_100_000_000,c);m.position(PositionObservation(1_100_000_000,'gps','q',None,c),1_100_000_000);self.assertEqual(m.status(1_100_000_000)['state'],'AMBIGUOUS');self.assertEqual(len(m.pending),2)
 def test_wrong_shadow_output_is_untrusted_and_history_unchanged(self):
  c,h,m=self.fixture();old=[v.copy() for v in h.poses];s=ShadowCorrector(h);T=np.eye(4);T[:3,3]=[100,0,50];self.assertTrue(s.consume(Candidate(1_000_000_000,'injected','injected',T,c,True,0.,'OK'),1_100_000_000)['accepted']);p=s.publish(1_100_000_000);self.assertFalse(p['trusted']);self.assertGreater(np.linalg.norm(p['pose'][:3,3]),100)
  for a,b in zip(old,h.poses):np.testing.assert_array_equal(a,b)
 def test_stale_cannot_anchor(self):
  c,h,m=self.fixture();r=m.ndt([np.eye(4)],'UNKNOWN',1_000_000_000,'g',1_600_000_000,c);self.assertEqual(r['reason'],'STALE');self.assertIsNone(m.anchor_ns)
 def test_exact_AO_code_and_config_frozen(self):
  root=Path(__file__).resolve().parents[1];d=json.loads((root/'experiments/phase_ap_robustness_validation/frozen_identity.json').read_text())
  for p,digest in d['core_sha256'].items():self.assertEqual(hashlib.sha256((root/p).read_bytes()).hexdigest(),digest,p)
if __name__=='__main__':unittest.main()
