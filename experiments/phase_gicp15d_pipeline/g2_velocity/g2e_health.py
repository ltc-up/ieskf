"""Opt-in engineering health gate for a full six-dimensional map pose."""
import numpy as np
from scipy.linalg import cho_factor, cho_solve
from scipy.spatial.transform import Rotation

from geometry_ieskf.external_pose import ExternalPose


class PoseHealth:
    NIS6 = 22.457744
    NIS3 = 16.266236

    def __init__(self, lever, gate=False, recovery=False, yaw_variance_scale=1., recovery_baseline=False):
        self.observer = ExternalPose(lever, ('x', 'y', 'z', 'rx', 'ry', 'rz'))
        self.gate = gate
        self.recovery = recovery
        self.yaw_variance_scale = yaw_variance_scale
        self.recovery_baseline = recovery_baseline
        self.last_accepted_ns = None
        self.first_seen_ns = None
        self.recovery_steps = 0
        self.consecutive_rejections = 0

    def check(self, prior, P, T, R, stamp_ns):
        R = np.array(R, float, copy=True)
        R[5, 5] *= self.yaw_variance_scale
        anchor = self.last_accepted_ns if self.last_accepted_ns is not None else self.first_seen_ns
        age = 0. if anchor is None else max(0., (stamp_ns-anchor)*1e-9)
        factor = 1.
        if self.recovery and not self.recovery_baseline and (age >= .5 or self.recovery_steps):
            factor = 4. if age >= .5 else 2.
        R *= factor
        try:
            residual, H = self.observer.residual(prior, T)
            S = H @ P @ H.T + R
            def nis(r, s):
                return float(r @ cho_solve(cho_factor(s), r))
            full, pos, rot = nis(residual, S), nis(residual[:3], S[:3, :3]), nis(residual[3:], S[3:, 3:])
            translation = float(np.linalg.norm(residual[:3]))
            angle = float(np.linalg.norm(Rotation.from_matrix(prior.R.T @ np.asarray(T)[:3, :3]).as_rotvec()))
            if not np.isfinite([full,pos,rot,translation,angle]).all(): raise ValueError('NONFINITE_OBSERVATION')
        except (ValueError, np.linalg.LinAlgError):
            return False, R, dict(reason='INVALID_OBSERVATION', R_factor=factor,
                                  age_s=age, state=self.state(stamp_ns))
        reason = 'PASS'
        if translation > 5. or angle > np.deg2rad(5.): reason = 'PHYSICAL_BOUND'
        elif full > self.NIS6: reason = 'NIS_GATE'
        accepted = reason == 'PASS' or not self.gate
        return accepted, R, dict(reason=reason, nis6=full, nis_position3=pos,
                                 nis_rotation3=rot, translation_m=translation,
                                 rotation_deg=float(np.rad2deg(angle)), R_factor=factor,
                                 age_s=age, state=self.state(stamp_ns))

    def state(self, stamp_ns):
        if self.recovery_steps: return 'RECOVERING'
        anchor = self.last_accepted_ns if self.last_accepted_ns is not None else self.first_seen_ns
        if anchor is None: return 'TRACKING'
        age = (stamp_ns-anchor)*1e-9
        return 'LOST' if age >= 2. else 'DEGRADED' if age >= .5 or self.consecutive_rejections else 'TRACKING'

    def record(self, stamp_ns, accepted):
        if self.first_seen_ns is None: self.first_seen_ns = stamp_ns
        if accepted:
            age = 0. if self.last_accepted_ns is None else (stamp_ns-self.last_accepted_ns)*1e-9
            if self.recovery and age >= .5: self.recovery_steps = 2
            elif self.recovery_steps: self.recovery_steps -= 1
            self.last_accepted_ns = stamp_ns
            self.consecutive_rejections = 0
        else:
            self.consecutive_rejections += 1
