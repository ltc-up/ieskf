"""Pure frozen linear candidates, never writes nominal state or caller P."""
import numpy as np
from scipy.linalg import cho_factor, cho_solve
from .posterior import valid_prior
from .state import chart_map
from .common import C

def linear_shadow_candidate(P, H, residual, R):
    """Conditional independent-noise algebra ONLY; not an information certificate."""
    P = np.array(valid_prior(P), copy=True)
    H = np.array(H, float, copy=True)
    r = np.array(residual, float, copy=True).reshape(-1)
    R = np.array(R, float, copy=True)
    m = len(r)
    if m == 0 or H.shape != (m, 15) or R.shape != (m, m) or not np.isfinite(np.r_[H.ravel(), r, R.ravel()]).all():
        raise ValueError('SHADOW_INPUT')
    if not np.allclose(R, R.T, atol=1e-12, rtol=0):
        raise ValueError('SHADOW_R_ASYMMETRIC')
    np.linalg.cholesky(R)
    S = H @ P @ H.T + R
    K = cho_solve(cho_factor(S), H @ P).T
    d = K @ r
    A = np.eye(15) - K @ H
    before_reset = A @ P @ A.T + K @ R @ K.T
    B = chart_map(d)
    post = B @ before_reset @ B.T
    post = (post + post.T) / 2
    valid_prior(post)
    guards = []
    for sl, limit, name in [(slice(6, 9), C['max_total_rotation_rad'], 'ATTITUDE_LINEARIZATION_DOMAIN'),
                            (slice(9, 12), C['max_bias_gyro_change_rad_s'], 'GYRO_BIAS_CHANGE_DOMAIN'),
                            (slice(12, 15), C['max_bias_accel_change_m_s2'], 'ACCEL_BIAS_CHANGE_DOMAIN')]:
        if np.linalg.norm(d[sl]) > limit:
            guards.append(name)
    return dict(committed=False, actual_applied_delta=np.zeros(15),
                delta_proposed=d, H=H, residual=r, R=R, S=S, K=K,
                P_candidate_before_reset=before_reset, P_candidate=post,
                P_change=post-P, reset=B,
                z_signed_innovation_contributions=K[2]*r,
                vz_signed_innovation_contributions=K[5]*r,
                conditional_NIS=float(r @ cho_solve(cho_factor(S), r)),
                original_domain_failures=guards,
                covariance_calibrated=False, shared_IMU_noise_modeled=False)
