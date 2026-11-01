"""New AG optimizer interface checks only; AF foundation suite is not rerun."""
import unittest,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments/phase_af_surface_observation'))
from synthetic_cases import prior,observation
from geometry_ieskf.surface_capture import *
from geometry_ieskf.surface_observation import Plane
class SurfaceCaptureTests(unittest.TestCase):
 def make(self,height):
  x,P=prior(correlated=True);obs=observation('horizontal',3,True);p=obs.planes['plane0'];obs.planes['plane0']=Plane(p.key,p.normal,p.anchor+height*p.normal,p.tangent,p.covariance);return FrozenSurfaceObjective(x,P,obs)
 def test_new_constant_R_objective_gradient(self):
  o=self.make(.8);z=np.array([.02,-.01,.03,.01,-.03,.02,.01,-.01,.005,.001,-.002,.001,.01,-.02,.01]);g=o.gradient(z)
  fd=np.array([(o.cost(z+np.eye(15)[j]*1e-6)-o.cost(z-np.eye(15)[j]*1e-6))/2e-6 for j in range(15)]);np.testing.assert_allclose(fd,g,rtol=1e-6,atol=1e-6)
 def test_large_single_step_has_legal_multistep_solution(self):
  o=self.make(.8);raw=o.L@o.local(np.zeros(15))[2];self.assertGreater(abs(raw[2]),.5);p=o.P.copy();r=constrained_surface_solve(o);self.assertTrue(r['qualified'],r['status']);self.assertGreater(len(r['history']),1);self.assertLess(r['z'][2],2);np.testing.assert_array_equal(o.P,p);self.assertEqual(r['covariance_updates'],0)
  for h in r['history']:
   if 'step_margins' in h:self.assertGreaterEqual(min(h['step_margins']),-1e-9);self.assertGreaterEqual(min(h['cumulative_margins']),-1e-9)
 def test_boundary_solution_not_infeasible(self):
  o=self.make(3);r=constrained_surface_solve(o);self.assertTrue(r['qualified'],r['status']);self.assertEqual(r['status'],'CONVERGED_BOUNDARY');self.assertAlmostEqual(np.linalg.norm(r['z'][:3]),2,places=5)
 def test_effective_covariance_really_frozen(self):
  o=self.make(.8);before=o.noise.dense().copy();o.cost(np.r_[np.zeros(6),.03,.02,.01,np.zeros(6)]);np.testing.assert_array_equal(o.noise.dense(),before)
 def test_budget_not_relabelled_converged(self):
  o=self.make(.8);r=constrained_surface_solve(o,max_inner=1);self.assertFalse(r['qualified']);self.assertEqual(r['status'],'INNER_BUDGET_EXHAUSTED');self.assertEqual(r['covariance_updates'],0)
if __name__=='__main__':unittest.main()
