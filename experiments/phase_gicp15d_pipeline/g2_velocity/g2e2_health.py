"""Opt-in, per-modality replay diagnostics; no changes to the 15D filter."""
import numpy as np
from bisect import bisect_right
from geometry_ieskf.common import C


class SensorHealth:
    """Track usable updates, rather than treating message arrival as fusion."""

    def __init__(self):
        self.last = {name: None for name in ('imu', 'wheel_vx', 'kistler_vx', 'kistler_vy')}
        self.missing = {name: 0 for name in self.last}
        self.accepted_stamps = {name: [] for name in self.last}
        self.no_update_frames = {name: 0 for name in self.last}
        self.previous_snapshot_ns = None

    def observe(self, event):
        kind, stamp = event['kind'], event['header_ns']
        if kind == 'imu':
            good = np.isfinite(np.r_[event['omega'], event['force']]).all()
            name = 'imu'
        elif kind == 'wheel':
            good = event.get('last_update', {}).get('accepted', False)
            name = 'wheel_vx'
        elif kind == 'optical':
            good = event.get('last_update', {}).get('accepted', False)
            name = 'kistler_vx' if event.get('selected_vx_source') == 'kistler' else 'kistler_vy'
            if good and name == 'kistler_vx':
                self.last['kistler_vy'] = stamp
                self.missing['kistler_vy'] = 0
                self.accepted_stamps['kistler_vy'].append(stamp)
        else:
            return
        if good:
            self.last[name] = stamp
            self.missing[name] = 0
            self.accepted_stamps[name].append(stamp)
        else:
            self.missing[name] += 1

    def snapshot(self, stamp, state, covariance, pose_state):
        last = {}
        for name, stamps in self.accepted_stamps.items():
            index = bisect_right(stamps, stamp)
            last[name] = stamps[index-1] if index else None
            updated = last[name] is not None and (self.previous_snapshot_ns is None or last[name] > self.previous_snapshot_ns)
            self.no_update_frames[name] = 0 if updated else self.no_update_frames[name]+1
        self.previous_snapshot_ns = stamp
        age = {k: None if t is None else (stamp-t)*1e-9 for k, t in last.items()}
        def fresh(k, limit):
            return age[k] is not None and age[k] <= limit
        velocity = 'kistler' if fresh('kistler_vx', .1) else 'wheel' if fresh('wheel_vx', .1) else 'none'
        vy = fresh('kistler_vy', .1)
        imu = fresh('imu', C['max_imu_interval_s'])
        xy = np.asarray(covariance[:2, :2], float)
        eig = np.linalg.eigvalsh(xy)
        lateral = np.asarray([-state.R[1, 0], state.R[0, 0]])
        longitudinal = np.asarray([state.R[0, 0], state.R[1, 0]])
        lateral_3sigma = 3.*np.sqrt(max(0., float(lateral @ xy @ lateral)))
        longitudinal_3sigma = 3.*np.sqrt(max(0., float(longitudinal @ xy @ longitudinal)))
        return dict(age_s=age, no_update_frames=self.no_update_frames.copy(), vx_source=velocity,
                    vy_valid=bool(vy), imu_valid=bool(imu), pose_state=pose_state,
                    status=('STALE' if not imu else 'LOST' if pose_state == 'LOST' or velocity == 'none'
                            else 'DEGRADED' if pose_state != 'TRACKING' or not vy else 'TRACKING'),
                    ellipse_major_3sigma_m=3.*np.sqrt(max(0., float(eig[-1]))),
                    ellipse_lateral_3sigma_m=lateral_3sigma,
                    ellipse_longitudinal_3sigma_m=longitudinal_3sigma)
