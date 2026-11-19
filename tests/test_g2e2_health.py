import unittest
import numpy as np
from geometry_ieskf.state import State
from experiments.phase_gicp15d_pipeline.g2_velocity.g2e_health import PoseHealth
from experiments.phase_gicp15d_pipeline.g2_velocity.g2e2_health import SensorHealth


class EngineeringHealthTest(unittest.TestCase):
    def test_long_loss_keeps_base_pose_covariance_and_recovers(self):
        h = PoseHealth(np.zeros(3), gate=True, recovery=True, recovery_baseline=True)
        x = State(np.zeros(3), np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3), 0)
        P = np.eye(15)*.01
        R = np.diag([.25]*3+[np.deg2rad(1.)**2]*3)
        h.record(0, True)
        self.assertEqual(h.state(2_100_000_000), 'LOST')
        ok, used, diag = h.check(x, P, np.eye(4), R, 2_100_000_000)
        self.assertTrue(ok)
        self.assertEqual(diag['R_factor'], 1.)
        np.testing.assert_array_equal(used, R)
        h.record(2_100_000_000, True)
        self.assertEqual(h.state(2_110_000_000), 'RECOVERING')
        h.record(2_150_000_000, True)
        h.record(2_200_000_000, True)
        self.assertEqual(h.state(2_250_000_000), 'TRACKING')

    def test_velocity_dropout_and_lateral_ellipse(self):
        h = SensorHealth()
        x = State(np.zeros(3), np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3), 0)
        P = np.eye(15)*.01
        P[0, 0], P[1, 1] = .04, .09
        h.observe(dict(kind='imu', header_ns=0, omega=[0, 0, 0], force=[0, 0, 0]))
        h.observe(dict(kind='wheel', header_ns=0, last_update=dict(accepted=True)))
        h.observe(dict(kind='optical', header_ns=0, selected_vx_source='kistler', last_update=dict(accepted=True)))
        d = h.snapshot(0, x, P, 'TRACKING')
        self.assertEqual(d['vx_source'], 'kistler')
        self.assertEqual(d['status'], 'TRACKING')
        self.assertAlmostEqual(d['ellipse_major_3sigma_m'], .9)
        self.assertAlmostEqual(d['ellipse_lateral_3sigma_m'], .9)
        self.assertAlmostEqual(d['ellipse_longitudinal_3sigma_m'], .6)
        self.assertFalse(h.snapshot(26_000_000, x, P, 'TRACKING')['imu_valid'])
        d = h.snapshot(200_000_000, x, P, 'LOST')
        self.assertEqual(d['vx_source'], 'none')
        self.assertEqual(d['status'], 'STALE')
        h.observe(dict(kind='imu', header_ns=200_000_000, omega=[0, 0, 0], force=[0, 0, 0]))
        h.observe(dict(kind='wheel', header_ns=200_000_000, last_update=dict(accepted=True)))
        d = h.snapshot(200_000_000, x, P, 'TRACKING')
        self.assertEqual(d['vx_source'], 'wheel')
        self.assertEqual(d['status'], 'DEGRADED')

    def test_future_arrival_does_not_make_header_healthy(self):
        h = SensorHealth()
        x = State(np.zeros(3), np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3), 0)
        P = np.eye(15)*.01
        h.observe(dict(kind='imu', header_ns=100_000_000, omega=[0, 0, 0], force=[0, 0, 0]))
        h.observe(dict(kind='optical', header_ns=100_000_000,
                       selected_vx_source='kistler', last_update=dict(accepted=True)))
        d = h.snapshot(50_000_000, x, P, 'TRACKING')
        self.assertFalse(d['imu_valid'])
        self.assertEqual(d['vx_source'], 'none')
        self.assertEqual(d['status'], 'STALE')


if __name__ == '__main__':
    unittest.main()
