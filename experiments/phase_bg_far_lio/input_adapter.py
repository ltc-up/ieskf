"""Research-only input contracts. No ROS graph, deskew, filtering or reference use."""
from dataclasses import dataclass
import numpy as np


def skew(v):
    x, y, z = np.asarray(v, dtype=float)
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def rigid(T):
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError('INVALID_TRANSFORM')
    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-12):
        raise ValueError('INVALID_HOMOGENEOUS_ROW')
    if not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-7) or np.linalg.det(T[:3, :3]) < 0:
        raise ValueError('INVALID_ROTATION')
    return T


@dataclass
class Cloud:
    xyz: np.ndarray
    intensity: np.ndarray
    offset_s: np.ndarray
    header_ns: int
    available_ns: int
    sensor_id: int
    identity: int
    T_base_sensor: np.ndarray


def decode_cloud(payload, fields, count, point_step, header_ns, available_ns, sensor_id, identity, T, bigendian=False):
    """Tightly packed scans; refuse an unsupported layout rather than flatten row padding."""
    if len(payload) != count * point_step or count <= 0:
        raise ValueError('INVALID_OR_PADDED_LAYOUT')
    if any(k not in fields for k in ('x', 'y', 'z', 'intensity', 'timestamp')):
        raise ValueError('REQUIRED_POINT_FIELD_MISSING')
    values = {}
    for k in ('x', 'y', 'z', 'intensity', 'timestamp'):
        f = fields[k]
        if f['datatype'] != 7 or f.get('count', 1) != 1 or f['offset'] + 4 > point_step:
            raise ValueError('EXPECTED_FLOAT32_POINT_FIELD')
        values[k] = np.ndarray((count,), dtype=('>' if bigendian else '<') + 'f4', buffer=payload,
                               offset=f['offset'], strides=(point_step,)).copy()
    xyz = np.column_stack([values[k] for k in ('x', 'y', 'z')])
    if not np.isfinite(xyz).all() or not np.isfinite(values['timestamp']).all():
        raise ValueError('NONFINITE_POINT_OR_TIME')
    if np.max(np.abs(values['timestamp'])) > 1.0:
        raise ValueError('POINT_TIME_NOT_SECONDS_OFFSET')
    return Cloud(xyz, values['intensity'], values['timestamp'], int(header_ns), int(available_ns),
                 sensor_id, identity, rigid(T))


def fuse_three(clouds):
    """Static fusion only. Use last acquisition as header; retain ns-rounded point epochs."""
    if len(clouds) != 3 or {c.sensor_id for c in clouds} != {0, 1, 2}:
        raise ValueError('EXACTLY_THREE_DISTINCT_SENSORS_REQUIRED')
    if len({c.identity for c in clouds}) != 3:
        raise ValueError('DUPLICATE_INPUT_ID')
    clouds = sorted(clouds, key=lambda c: c.sensor_id)
    epochs = [c.header_ns + np.rint(c.offset_s.astype(float) * 1e9).astype(np.int64) for c in clouds]
    header = max(int(t.max()) for t in epochs)
    available = max(c.available_ns for c in clouds)
    if available < header:
        raise ValueError('INPUT_AVAILABLE_BEFORE_LAST_POINT')
    dtype = [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('timestamp', '<f4'),
             ('intensity', '<f4'), ('sensor_id', 'u1')]
    out = np.empty(sum(len(c.xyz) for c in clouds), dtype=dtype)
    i = 0
    max_time_error = 0
    for c, t in zip(clouds, epochs):
        T = rigid(c.T_base_sensor)
        xyz = c.xyz.astype(float) @ T[:3, :3].T + T[:3, 3]
        n = len(xyz)
        for k, v in zip(('x', 'y', 'z'), xyz.T): out[k][i:i+n] = v
        out['timestamp'][i:i+n] = (t - header).astype(float) * 1e-9
        out['intensity'][i:i+n] = c.intensity
        out['sensor_id'][i:i+n] = c.sensor_id
        restored = header + np.rint(out['timestamp'][i:i+n].astype(float) * 1e9).astype(np.int64)
        max_time_error = max(max_time_error, int(np.max(np.abs(restored - t))))
        i += n
    return out, dict(header_ns=header, available_ns=available, max_point_time_roundtrip_error_ns=max_time_error,
                     frame='base_link', deskew_applied=False, sensors=[c.identity for c in clouds])


def covariance_or_none(C):
    """Preserve absent/unknown IMU covariance; never rotate a -1 sentinel as data."""
    if C is None: return None
    C = np.asarray(C, dtype=float)
    if C.size == 9: C = C.reshape(3, 3)
    if C.shape != (3, 3) or not np.isfinite(C).all(): raise ValueError('INVALID_IMU_COVARIANCE')
    if C[0, 0] == -1 or np.count_nonzero(C) == 0: return None
    if not np.allclose(C, C.T, atol=1e-12) or np.linalg.eigvalsh(C).min() < 0:
        raise ValueError('INVALID_IMU_COVARIANCE')
    return C


def adapt_imu(omega_sensor, force_sensor, T_base_imu, stamp_ns, previous=None,
              gyro_cov=None, force_cov=None):
    """Specific force at base origin. Alpha uses past/current gyro only, no future input.

    Output acceleration/gyro cross-correlation and temporal correlation cannot be
    represented by sensor_msgs/Imu; expose this limitation, do not claim calibrated R.
    """
    T = rigid(T_base_imu); R = T[:3, :3]; r = T[:3, 3]
    gyro_cov = covariance_or_none(gyro_cov); force_cov = covariance_or_none(force_cov)
    w = R @ np.asarray(omega_sensor, dtype=float)
    f = R @ np.asarray(force_sensor, dtype=float)
    if not np.isfinite(np.r_[w, f]).all(): raise ValueError('NONFINITE_IMU')
    if previous is None and np.linalg.norm(r) > 0:
        raise ValueError('PREVIOUS_GYRO_REQUIRED_FOR_LEVER_ARM')
    dt = None
    alpha = np.zeros(3)
    if previous is not None:
        dt = (int(stamp_ns) - int(previous['stamp_ns'])) * 1e-9
        if not 0 < dt <= .03: raise ValueError('INVALID_CAUSAL_IMU_INTERVAL')
        alpha = (w - previous['omega_base']) / dt
    correction = np.cross(alpha, r) + np.cross(w, np.cross(w, r))
    result = dict(stamp_ns=int(stamp_ns), omega_base=w, force_base=f-correction,
                  force_rotated_only=f, angular_acceleration_base=alpha, lever_correction=correction,
                  gravity_removed=False, frame='base_link', cross_covariance_representable=False, gyro_cov_available=gyro_cov is not None,
                  force_cov_available=force_cov is not None, statistical_noise_calibration=False)
    if gyro_cov is not None:
        Cw = R @ np.asarray(gyro_cov) @ R.T
        result['gyro_cov'] = Cw
    if force_cov is not None:
        Cf = R @ np.asarray(force_cov) @ R.T
        if dt is not None and gyro_cov is not None and 'gyro_cov' in previous:
            Jc = skew(r)/dt + skew(np.cross(w, r)) + skew(w) @ skew(r)
            Jp = -skew(r)/dt
            Cf = Cf + Jc @ Cw @ Jc.T + Jp @ previous['gyro_cov'] @ Jp.T
            result['force_gyro_cross_cov'] = Jc @ Cw
        else: result['lever_noise_covariance_complete'] = False
        result['force_cov'] = Cf
    return result
