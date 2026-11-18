"""Velocity-source semantics for offline G2-V replay."""
import numpy as np
from ndt_global.delayed_local import DelayedLocalReplay
from ndt_global.optical_replay import OpticalReplay


class VelocityReplay(OpticalReplay):
    """One vx source at each timestamp; Kistler vy remains a real observation."""

    def step(self, checkpoint, event):
        state, covariance, imu = checkpoint
        timestamp = event['header_ns']
        last_vx = imu.get('last_optical_vx_ns', -10**30)
        last_wheel = imu.get('last_valid_wheel_ns', -10**30)
        if event['kind'] == 'optical':
            good = event['status_good'] and np.isfinite(event['velocity']).all()
            wheel_recent = 0 <= timestamp - last_wheel <= 100_000_000
            use_vx = self.velocity_policy == 'C' and good and (
                event['velocity'][0] >= 1.0 or not wheel_recent
            )
            self.optical = self.optical_xy if use_vx else self.optical_y
            result = super().step(checkpoint, event)
            updated = result[2].copy()
            accepted = event['last_update']['accepted']
            updated['last_optical_vx_ns'] = timestamp if accepted and use_vx else -10**30
            updated['last_valid_wheel_ns'] = last_wheel
            event['selected_vx_source'] = 'kistler' if accepted and use_vx else 'wheel_or_none'
            return result[0], result[1], updated
        if event['kind'] == 'wheel':
            valid = event.get('valid', True) and np.isfinite(
                [event.get('speed', np.nan), event.get('variance', np.nan)]
            ).all() and event.get('variance', -1) >= 0
            skip = self.velocity_policy == 'C' and 0 <= timestamp - last_vx <= 100_000_000
            if not valid or skip:
                advance = dict(kind='imu', header_ns=timestamp, omega=imu['omega'], force=imu['force'])
                next_state, next_covariance, _ = DelayedLocalReplay.step(self, checkpoint, advance)
                event['last_update'] = dict(accepted=False, reason=(
                    'WHEEL_INVALID_NO_ZERO_INJECTION' if not valid else 'KISTLER_VX_ACTIVE'
                ))
                updated = imu.copy()
            else:
                next_state, next_covariance, updated = super().step(checkpoint, event)
                updated = updated.copy()
                if event['last_update']['accepted']:
                    last_wheel = timestamp
            updated['last_optical_vx_ns'] = last_vx
            updated['last_valid_wheel_ns'] = last_wheel
            event['selected_vx_source'] = 'wheel' if event['last_update']['accepted'] else 'none'
            return next_state, next_covariance, updated
        next_state, next_covariance, updated = super().step(checkpoint, event)
        updated = updated.copy()
        updated['last_optical_vx_ns'] = last_vx
        updated['last_valid_wheel_ns'] = last_wheel
        return next_state, next_covariance, updated
