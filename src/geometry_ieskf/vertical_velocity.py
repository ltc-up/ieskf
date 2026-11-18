"""BE5 shadow-only base-point velocity models. No state/P update API."""
import numpy as np
from .common import skew

def _inputs(x, omega, lever):
    w = np.asarray(omega, float)
    r = np.asarray(lever, float)
    if w.shape != (3,) or r.shape != (3,) or not np.isfinite(np.r_[w, r, x.v, x.bg, x.R.ravel()]).all():
        raise ValueError('VERTICAL_INPUT')
    return w - x.bg, r

def fixed_normal_model(x, omega, lever, normal, *, normal_source):
    """A1: fixed unit normal in map; provenance must be explicit."""
    w, r = _inputs(x, omega, lever)
    n = np.asarray(normal, float)
    if not normal_source or n.shape != (3,) or not np.isfinite(n).all() or not np.isclose(n @ n, 1., atol=1e-10, rtol=0):
        raise ValueError('FIXED_NORMAL_CONTRACT')
    c = np.cross(w, r)
    h = float(n @ (x.v - x.R @ c))
    H = np.zeros((1, 15))
    H[0, 3:6] = n
    H[0, 6:9] = n @ x.R @ skew(c)
    H[0, 9:12] = -n @ x.R @ skew(r)
    return h, H

def body_normal_model(x, omega, lever):
    """A2: n(x)=R ez; distinct from A1 with a frozen predicted normal."""
    w, r = _inputs(x, omega, lever)
    vb = x.R.T @ x.v
    h = float((vb - np.cross(w, r))[2])
    H = np.zeros((1, 15))
    H[0, 3:6] = x.R.T[2]
    H[0, 6:9] = skew(vb)[2]
    H[0, 9:12] = -skew(r)[2]
    return h, H
