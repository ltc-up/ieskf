import unittest
import numpy as np
from ndt_global.track_evaluation import Polyline,paired_errors,interpolate_reference,offline_quality
class TrackTests(unittest.TestCase):
 def test_signed_fixed_tangent(self):
  p=Polyline([[0,0],[10,0],[10,10],[0,10]]);z=p.project([[5,2]]);self.assertAlmostEqual(z['s'][0],5);self.assertAlmostEqual(z['d'][0],2);es,ed=paired_errors([[8,3]],[[5,2]],z['tangent']);np.testing.assert_allclose([es[0],ed[0]],[3,1])
 def test_invalid_gap_never_bridged(self):
  _,ok=interpolate_reference([0,.05,.1,.5],[[0,0],[1,0],[2,0],[8,0]],[True,False,True,True],[.025,.075,.3],.1);self.assertFalse(ok.any())
 def test_closed_progress(self):
  p=Polyline([[0,0],[10,0],[10,10],[0,10],[0,0]]);self.assertEqual(p.length,40);self.assertAlmostEqual(p.project([[0,1]])['s'][0],39)
 def test_quality_not_fitness_only(self):
  self.assertEqual(offline_quality(converged=True,valid=True,support=.95,rank=6,multimode=True),'AMBIGUOUS');self.assertEqual(offline_quality(converged=True,valid=True,support=.5,rank=6),'DEGRADED')
