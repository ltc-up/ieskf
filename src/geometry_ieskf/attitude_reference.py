"""S2 tilt mathematics/source diagnostics only; no fusion/ROS interface."""
import numpy as np
from .common import skew

def tilt_model(x, reference_rotation, up=(0., 0., 1.)):
    """Fixed reference tangent, state right-SO3 perturbation. Unitless residual."""
    R = np.asarray(reference_rotation, float)
    k = np.asarray(up, float)
    if R.shape != (3, 3) or not np.isfinite(R).all() or not np.allclose(R.T @ R, np.eye(3), atol=1e-10, rtol=0) or not np.isclose(np.linalg.det(R), 1., atol=1e-10):
        raise ValueError('REFERENCE_ROTATION')
    if k.shape != (3,) or not np.isfinite(k).all() or not np.isclose(k @ k, 1., atol=1e-10, rtol=0):
        raise ValueError('UP_DIRECTION')
    uref = R.T @ k
    u = x.R.T @ k
    if float(u @ uref) <= 0:
        raise ValueError('TILT_OUTSIDE_LOCAL_HEMISPHERE')
    axis = np.eye(3)[int(np.argmin(abs(uref)))]
    e1 = np.cross(uref, axis)
    e1 /= np.linalg.norm(e1)
    E = np.column_stack((e1, np.cross(uref, e1)))
    residual = E.T @ (uref - u)
    H = np.zeros((2, 15))
    H[:, 6:9] = E.T @ skew(u)
    return residual, H, E

def source_diagnostic(provenance=None):
    p = {} if provenance is None else dict(provenance)
    risks = [k for k in ('uses_bosch', 'uses_gnss_ins', 'uses_self_deskew') if p.get(k) is not False]
    missing = [k for k in ('source_id', 'timestamp_contract', 'covariance_provenance') if not p.get(k)]
    return dict(fusion_allowed=False, stage='STAGE2A_SHADOW_ONLY',
                source_status='UNKNOWN' if risks or missing else 'DECLARED_NOT_INDEPENDENTLY_VALIDATED',
                correlation_unknown=bool(risks), correlation_risks=risks,
                missing_contracts=missing)
