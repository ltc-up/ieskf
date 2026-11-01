"""New seed-adapter contract checks, not a rerun of historical math suites."""
import unittest,numpy as np
from geometry_ieskf.state import State,inject
from geometry_ieskf.common import exp,C
from geometry_ieskf.geometry import GeometryFactor,FixedCorrespondence
from geometry_ieskf.seeded_geometry import ndt_seed,SeededUpdate,POSE,OTHER
from geometry_ieskf.instrumented import InstrumentedUpdate
from geometry_ieskf.qualification import atomic_select
class SeedInterfaceTests(unittest.TestCase):
 def setUp(self):
  self.x=State(np.array([1.,2.,3.]),np.zeros(3),exp(np.array([.1,-.1,.2])),np.zeros(3),np.zeros(3),12);self.P=np.eye(15);self.factor=GeometryFactor([.2,-.1,.3]);rng=np.random.default_rng(3);self.p=rng.normal(size=(150,3));self.q=self.p+self.x.p-self.x.R@self.factor.lever;self.n=np.tile([0.,0.,1.],(150,1));self.corr=FixedCorrespondence(self.p,self.q,self.n,np.ones(150))
 def test_pose_round_trip(self):
  d=np.zeros(15);d[:3]=[.1,.2,-.1];d[6:9]=[.01,.02,-.01];target=inject(self.x,d);z=ndt_seed(self.x,self.P,self.factor.transform(target),self.factor.lever);np.testing.assert_allclose(self.factor.transform(inject(self.x,z)),self.factor.transform(target),atol=1e-14)
 def test_conditional_completion_preserves_cross_prior(self):
  self.P[5,2]=self.P[2,5]=.4;d=np.zeros(15);d[2]=.1;z=ndt_seed(self.x,self.P,self.factor.transform(inject(self.x,d)),self.factor.lever);self.assertAlmostEqual(z[5],.04);self.assertEqual(self.x.t,12)
 def test_infeasible_seed_not_clipped(self):
  z=np.zeros(15);z[9]=.031;s=SeededUpdate(self.factor);_,p,i=s.correct(self.x,self.P,self.p,self.corr,z);self.assertEqual(i['reason'],'INFEASIBLE_INITIAL_SEED');np.testing.assert_array_equal(s.last_delta,z);self.assertIs(p,self.P)
 def test_zero_start_matches_historical_decisions(self):
  old=InstrumentedUpdate(self.factor);new=SeededUpdate(self.factor);_,_,a=old.correct(self.x,self.P,self.p,self.corr);_,_,b=new.correct(self.x,self.P,self.p,self.corr);self.assertEqual(a['reason'],b['reason']);np.testing.assert_allclose(old.trace['proposals'][0]['absolute_frozen_delta'],new.trace['proposals'][0]['absolute_frozen_delta'])
 def test_nonzero_seed_frozen_prior_not_reset(self):
  z=np.zeros(15);z[3]=.2;s=SeededUpdate(self.factor);P=self.P.copy();before=self.x.p.copy();_,p,i=s.correct(self.x,self.P,self.p,self.corr,z);self.assertIs(p,self.P);np.testing.assert_array_equal(self.P,P);np.testing.assert_array_equal(self.x.p,before);self.assertEqual(i.get('covariance_updates',0),0);self.assertNotEqual(s.trace['proposals'][0]['current_delta'][3],0.);self.assertGreaterEqual(i['iterations'][0]['cost_before'],.5*.2**2)
 def test_partial_candidate_atomic_rejection(self):
  d=np.zeros(15);d[2]=.2;candidate=inject(self.x,d);x,P,decision=atomic_select(self.x,self.P,candidate,self.P*.2,{'accepted':False,'status':'REJECTED_KEEP_PRIOR'})
  np.testing.assert_array_equal(x.p,self.x.p);np.testing.assert_array_equal(P,self.P);self.assertFalse(decision['accepted'])
if __name__=='__main__':unittest.main()
