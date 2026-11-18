import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.state import State
from experiments.phase_gicp15d_pipeline.g2_velocity.g2e_health import PoseHealth


class PoseHealthTest(unittest.TestCase):
    def setUp(self):
        self.x = State(np.zeros(3), np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3), 0)
        self.P = np.eye(15)*.01
        self.R = np.diag([.25]*3+[np.deg2rad(1.)**2]*3)

    def test_gate_and_so3(self):
        h = PoseHealth(np.zeros(3), gate=True)
        T = np.eye(4)
        ok, _, d = h.check(self.x, self.P, T, self.R, 0)
        self.assertTrue(ok)
        self.assertEqual(d['nis6'], 0.)
        T[0, 3] = 6.
        ok, _, d = h.check(self.x, self.P, T, self.R, 0)
        self.assertFalse(ok)
        self.assertEqual(d['reason'], 'PHYSICAL_BOUND')
        T[0, 3] = 3.
        ok, _, d = h.check(self.x, self.P, T, self.R, 0)
        self.assertFalse(ok)
        self.assertEqual(d['reason'], 'NIS_GATE')
        T = np.eye(4)
        T[:3, :3] = Rotation.from_euler('z', 10, degrees=True).as_matrix()
        ok, _, d = h.check(self.x, self.P, T, self.R, 0)
        self.assertFalse(ok)
        self.assertAlmostEqual(d['rotation_deg'], 10.)
        T[0, 3] = np.nan
        ok, _, d = h.check(self.x, self.P, T, self.R, 0)
        self.assertFalse(ok)
        self.assertEqual(d['reason'], 'INVALID_OBSERVATION')

    def test_loss_recovery(self):
        h = PoseHealth(np.zeros(3), gate=True, recovery=True)
        h.record(0, True)
        h.record(50_000_000, False)
        self.assertEqual(h.state(50_000_000), 'DEGRADED')
        self.assertEqual(h.state(2_100_000_000), 'LOST')
        T = np.eye(4)
        ok, R, d = h.check(self.x, self.P, T, self.R, 2_100_000_000)
        self.assertTrue(ok)
        self.assertEqual(d['R_factor'], 4.)
        h.record(2_100_000_000, True)
        self.assertEqual(h.state(2_150_000_000), 'RECOVERING')
        _, _, d = h.check(self.x, self.P, T, self.R, 2_150_000_000)
        self.assertEqual(d['R_factor'], 2.)
        h.record(2_150_000_000, True)
        h.record(2_200_000_000, True)
        self.assertEqual(h.state(2_250_000_000), 'TRACKING')

    def test_initial_missing_observations_become_lost(self):
        h = PoseHealth(np.zeros(3), gate=True, recovery=True)
        h.record(0, False)
        self.assertEqual(h.state(100_000_000), 'DEGRADED')
        self.assertEqual(h.state(2_100_000_000), 'LOST')


if __name__ == '__main__': unittest.main()
