import unittest
import numpy as np
from prototype import update, hierarchy, geometry, gaussian, health_scenario, Health

class ArchitectureTests(unittest.TestCase):
    def test_isolation_and_wrong_global(self):
        r=hierarchy()
        self.assertEqual(r['local_difference_max'],0.)
        self.assertGreater(r['global_position_errors'][1],7.9)
        self.assertGreater(r['global_position_errors'][2],11.)
        self.assertLess(r['global_position_errors'][-1],1e-12)
        self.assertGreater(r['local_sigma_z_final'],1.)
        self.assertAlmostEqual(r['global_rotation_errors'][-1],.4)
        self.assertGreater(max(v['velocity'] for v in r['direct_corrections']),.1)
        self.assertGreater(max(v['bias'] for v in r['direct_corrections']),.01)

    def test_gaussian_reference_and_information_loss(self):
        r=gaussian()
        self.assertLess(r['full_reference_error'],1e-12)
        self.assertLess(r['schmidt_reference_error'],1e-12)
        self.assertEqual(r['schmidt_delta'][2],0.)
        self.assertEqual(r['schmidt_variance'],1.)
        self.assertFalse(np.allclose(r['schmidt_cross'],[.4,.3]))
        self.assertTrue(all(v>0 for v in r['min_eigenvalues']))
        self.assertLess(max(r['symmetry_errors']),1e-12)
        self.assertGreater(r['schmidt_trace'],r['full_trace'])
        self.assertGreater(r['projected_trace'],r['full_trace'])

    def test_scalar_schmidt_analytic(self):
        P=np.array([[2.,.6],[.6,1.]])
        d,Q,K=update(P,np.array([[1.,1.]]),np.array([[.2]]),np.array([1.]),[0])
        # S=4.4, C_a=2.6, C_s=1.6. P_ss remains 1.
        np.testing.assert_allclose(d,[2.6/4.4,0.],atol=1e-14)
        np.testing.assert_allclose(Q,[[2-2.6**2/4.4,.6-2.6*1.6/4.4],[.6-2.6*1.6/4.4,1.]],atol=1e-14)

    def test_projection_discards_null_channel(self):
        H=np.eye(2); U=np.array([[1.,0.]])
        d,Q,_=update(np.eye(2),U@H,np.eye(1),np.array([1.]))
        np.testing.assert_allclose(d,[.5,0.]); np.testing.assert_allclose(Q,np.diag([.5,1.]))

    def test_geometry_and_peak_counterexample(self):
        r=geometry()
        self.assertEqual(r['ranks'],dict(single=3,orthogonal=5,multi=6,missing_block=3))
        self.assertGreater(abs(np.diff(r['repeated_peaks'])[0]),4.9)
        self.assertLess(abs(np.diff(r['repeated_costs'])[0]),1e-10)
        np.testing.assert_allclose(r['repeated_curvature'],[25.,25.],rtol=1e-5)

    def test_health_semantics(self):
        r=health_scenario()
        self.assertEqual([v['reason'] for v in r],['ACCEPTED','ACCEPTED_PARTIAL','AMBIGUOUS','ACCEPTED','STALE','MISSING','REJECTED_INCONSISTENT','NOT_CONVERGED','REJECTED_INCONSISTENT','ACCEPTED','DUPLICATE'])
        self.assertTrue(r[2]['received']); self.assertFalse(r[2]['accepted'])
        self.assertFalse(r[5]['received'])
        h=Health(); self.assertEqual(h.assess('ndt','zero',0,0,rank=0)['reason'],'UNSUPPORTED')
        self.assertEqual(h.assess('ndt','a',0,0,peaks=2)['reason'],'AMBIGUOUS')
        self.assertTrue(h.assess('ndt','b',1,1)['accepted'])

    def test_duplicate_not_independent_information(self):
        h=Health(); P=np.eye(2); H=np.array([[1.,0.]])
        for _ in range(2):
            if h.assess('ndt','same_scan',0,0)['accepted']:
                _,P,_=update(P,H,np.eye(1),np.zeros(1))
        self.assertEqual(P[0,0],.5)

if __name__=='__main__': unittest.main()
