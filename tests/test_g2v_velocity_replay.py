import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / 'experiments' /
                       'phase_gicp15d_pipeline' / 'g2_velocity'))
from velocity_replay import VelocityReplay
from filter_optical import VelocitySensorFilter
from geometry_ieskf.optical_velocity import OpticalVelocity
from geometry_ieskf.state import State


class VelocityPolicyTests(unittest.TestCase):
    def replay(self, policy, vx=20.):
        state = State(np.zeros(3), np.array([vx, .5, 0.]), np.eye(3),
                      np.zeros(3), np.zeros(3), 1)
        held = dict(header_ns=1, omega=np.zeros(3), force=np.array([0, 0, 9.80665]))
        replay = VelocityReplay(state, np.eye(15)*.01, held, np.zeros(3))
        replay.policy = 'g2v'
        replay.velocity_policy = policy
        replay.optical_y = OpticalVelocity(np.zeros(3), np.eye(4), (1,))
        replay.optical_xy = OpticalVelocity(np.zeros(3), np.eye(4), (0, 1))
        replay.optical = replay.optical_y
        return replay

    @staticmethod
    def optical(timestamp, vx, good=True):
        return dict(kind='optical', header_ns=timestamp, message_id=timestamp,
                    velocity=[vx, .5], variance=[[.26, 0], [0, .26]],
                    status_good=good)

    @staticmethod
    def wheel(timestamp, vx, valid=True):
        return dict(kind='wheel', header_ns=timestamp, message_id=timestamp,
                    speed=vx, variance=.01, valid=valid)

    def test_b_keeps_wheel_vx_and_observes_only_vy(self):
        replay = self.replay('B')
        optical = self.optical(1_000_001, 20.)
        wheel = self.wheel(2_000_001, 20.)
        replay.ingest(optical)
        replay.ingest(wheel)
        self.assertEqual(optical['last_update']['observed_axes'], (1,))
        self.assertTrue(wheel['last_update']['accepted'])

    def test_c_uses_one_high_speed_vx_source(self):
        replay = self.replay('C')
        optical = self.optical(1_000_001, 20.)
        wheel = self.wheel(2_000_001, 20.)
        replay.ingest(optical)
        replay.ingest(wheel)
        self.assertEqual(optical['last_update']['observed_axes'], (0, 1))
        self.assertEqual(optical['selected_vx_source'], 'kistler')
        self.assertEqual(wheel['last_update']['reason'], 'KISTLER_VX_ACTIVE')

    def test_c_low_speed_wheel_and_invalid_wheel_no_zero(self):
        replay = self.replay('C', .5)
        wheel = self.wheel(1_000_001, .5)
        optical = self.optical(2_000_001, .5)
        invalid = self.wheel(3_000_001, 0., valid=False)
        replay.ingest(wheel)
        replay.ingest(optical)
        replay.ingest(invalid)
        self.assertTrue(wheel['last_update']['accepted'])
        self.assertEqual(optical['last_update']['observed_axes'], (1,))
        self.assertEqual(invalid['last_update']['reason'], 'WHEEL_INVALID_NO_ZERO_INJECTION')

    def test_c_kistler_fallback_when_wheel_missing(self):
        replay = self.replay('C', .5)
        optical = self.optical(1_000_001, .5)
        replay.ingest(optical)
        self.assertEqual(optical['last_update']['observed_axes'], (0, 1))

    def test_c_wheel_fallback_when_kistler_invalid(self):
        replay = self.replay('C')
        previous = self.optical(1_000_001, 20.)
        optical = self.optical(2_000_001, 20., good=False)
        wheel = self.wheel(3_000_001, 20.)
        replay.ingest(previous)
        replay.ingest(optical)
        replay.ingest(wheel)
        self.assertTrue(previous['last_update']['accepted'])
        self.assertFalse(optical['last_update']['accepted'])
        self.assertTrue(wheel['last_update']['accepted'])

    def test_production_velocity_filter_initialization_and_gap(self):
        filter_ = VelocitySensorFilter()
        self.assertEqual(filter_.push(10., 20.), 20.)
        value = filter_.push(10.01, 21.)
        self.assertGreater(value, 20.)
        self.assertLess(value, 21.)
        self.assertEqual(filter_.push(11.02, 18.), 18.)


if __name__ == '__main__':
    unittest.main()
