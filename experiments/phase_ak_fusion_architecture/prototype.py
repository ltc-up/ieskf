"""Independent synthetic research. No production inputs or calibrated sensor noise."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize_scalar
from scipy.special import logsumexp
from geometry_ieskf.state import State, inject, chart_map
from geometry_ieskf.inertial import InertialPredictor


def update(P, H, R, residual, active=None):
    """Constrained minimum-MSE gain with exact arbitrary-gain Joseph covariance.

    active=None: full KF. Otherwise only listed state means may change (Schmidt).
    Independent zero-mean measurement noise is an explicit assumption.
    """
    S = H @ P @ H.T + R
    C = P @ H.T
    K = np.zeros_like(C)
    rows = np.arange(len(P)) if active is None else np.asarray(active)
    K[rows] = np.linalg.solve(S, C[rows].T).T
    A = np.eye(len(P)) - K @ H
    return K @ residual, A @ P @ A.T + K @ R @ K.T, K


def transform(x):
    T = np.eye(4)
    T[:3,:3], T[:3,3] = x.R, x.p
    return T


class Health:
    """Orthogonal source facts; accepted records one consumption of an input group."""
    def __init__(self):
        self.consumed = set()

    def assess(self, source, group, now, stamp=None, converged=True,
               rank=None, peaks=1, consistent=True, timeout=1.):
        if rank is None:
            rank = 6 if source == 'ndt' else 3 if source == 'gnss' else 1
        reason = ('MISSING' if stamp is None else 'STALE' if now-stamp > timeout
                  else 'NOT_CONVERGED' if not converged else 'UNSUPPORTED' if rank == 0
                  else 'AMBIGUOUS' if peaks > 1
                  else 'REJECTED_INCONSISTENT' if not consistent
                  else 'DUPLICATE' if group in self.consumed
                  else 'ACCEPTED_PARTIAL' if source == 'ndt' and rank < 6 else 'ACCEPTED')
        accepted = reason.startswith('ACCEPTED')
        if accepted:
            self.consumed.add(group)
        return dict(source=source, reason=reason, accepted=accepted, rank=rank,
                    converged=converged, peaks=peaks, received=stamp is not None)


def hierarchy():
    # Same inertial model as repository. Covariance is synthetic, NOT real P.
    x = State(np.zeros(3), np.array([2.,0.,0.]), np.eye(3), np.zeros(3), np.zeros(3))
    a, b, control = x.copy(), x.copy(), x.copy()
    P = np.eye(15)
    Pb = np.eye(15)
    predictor = InertialPredictor()
    H = np.zeros((4,15)); H[:3,:3] = np.eye(3); H[3,8] = 1
    R = np.diag([.04,.04,.04,.01])
    correction = np.eye(4)
    errors, yaw_errors, local_delta, direct_delta = [], [], [], []
    gnss_recovery = None
    max_sigma = 0.
    for k in range(1,101):
        a, P = predictor.predict(a,P,np.zeros(3),np.array([0.,0.,9.80665]),.02)
        b, Pb = predictor.predict(b,Pb,np.zeros(3),np.array([0.,0.,9.80665]),.02)
        control, _ = predictor.predict(control,np.eye(15),np.zeros(3),np.array([0.,0.,9.80665]),.02)
        max_sigma = max(max_sigma,float(np.sqrt(P[2,2])))
        # NDT accepted deliberately without integrity gates to expose failure propagation.
        if k in (20,40,60,90):
            M = transform(control)
            if k == 40: M[2,3] += 8.  # wrong Z
            if k == 60:
                M[:2,3] += [10.,-5.]
                M[:3,:3] = Rotation.from_euler('z',.4).as_matrix()
            if k == 90: gnss_recovery = True  # qualified synthetic position anchor returns
            residual = np.r_[M[:3,3]-a.p, Rotation.from_matrix(a.R.T@M[:3,:3]).as_rotvec()[2]]
            d,P,_ = update(P,H[:3] if k == 90 else H,R[:3,:3] if k == 90 else R,residual[:3] if k == 90 else residual)
            a = inject(a,d); B=chart_map(d); P=B@P@B.T
            if k == 90:
                # Single-antenna GNSS restores position only; it does not observe yaw.
                correction[:3,3] = M[:3,3]-correction[:3,:3]@b.p
            else:
                correction = M @ np.linalg.inv(transform(b))
            global_pose = correction @ transform(b)
            yaw_errors.append(float(Rotation.from_matrix(global_pose[:3,:3]).magnitude()))
            errors.append(float(np.linalg.norm(global_pose[:3,3]-control.p)))
            direct_delta.append(dict(step=k,velocity=float(np.linalg.norm(d[3:6])),bias=float(np.linalg.norm(d[9:15]))))
        local_delta.append(float(np.linalg.norm(b.p-control.p)+np.linalg.norm(b.v-control.v)+np.linalg.norm(b.bg-control.bg)+np.linalg.norm(b.ba-control.ba)))
    return dict(local_difference_max=max(local_delta), global_position_errors=errors, global_rotation_errors=yaw_errors,
                local_sigma_z_final=float(np.sqrt(Pb[2,2])),
                direct_corrections=direct_delta, gnss_recovery=gnss_recovery,
                global_covariance='UNCALIBRATED_CORRELATION_UNKNOWN', direct_max_sigma_z=max_sigma)


def geometry_rows(normals):
    # Independent known infinite physical surfaces: no sampled-grid tangential forces.
    points = np.array([[u,v,0.] for u in (-1.,0.,1.) for v in (-1.,0.,1.)])
    rows=[]
    for n in normals:
        n=np.asarray(n,float)
        axis=np.eye(3)[np.argmin(abs(n))]; u=np.cross(n,axis); v=np.cross(n,u)
        for p in points:
            q=p[0]*u+p[1]*v
            rows.append(np.r_[n,np.cross(q,n)])
    return np.array(rows)


def geometry():
    cases={'single':[[0,0,1]],'orthogonal':[[0,0,1],[1,0,0]],
           'multi':[[0,0,1],[1,0,0],[0,1,0]],'missing_block':[[0,0,1]]}
    ranks={k:int(np.linalg.matrix_rank(geometry_rows(v))) for k,v in cases.items()}
    # One-dimensional NDT-style Gaussian-mixture map with repeated cells, embedded
    # in other well-constrained axes. This is NOT the production CUDA objective.
    centers=np.arange(-10.,11.,5.)
    def cost(t): return float(-logsumexp(-.5*((t-centers)/.2)**2))
    fits=[minimize_scalar(cost,bounds=(s-.5,s+.5),method='bounded').x for s in (0.,5.)]
    curvature=[(cost(t+1e-4)-2*cost(t)+cost(t-1e-4))/1e-8 for t in fits]
    return dict(ranks=ranks, repeated_peaks=fits, repeated_costs=[cost(t) for t in fits],
                repeated_curvature=curvature, missing_support_fraction=1/3)


def gaussian():
    P=np.array([[2.,.6,.4],[.6,1.5,.3],[.4,.3,1.]])
    H=np.array([[1.,0.,1.],[0.,1.,0.]])
    R=np.diag([.2,.3]); r=np.array([1.,-.5])
    full=update(P,H,R,r); schmidt=update(P,H,R,r,[0,1])
    U=np.array([[1.,0.]])
    projected=update(P,U@H,U@R@U.T,U@r)
    reference=np.linalg.inv(np.linalg.inv(P)+H.T@np.linalg.solve(R,H))
    # Direct joint error transform, not the short KF covariance formula.
    K=schmidt[2]; J=np.block([P,np.zeros((3,2))]); J=np.vstack([J,np.c_[np.zeros((2,3)),R]])
    E=np.c_[np.eye(3)-K@H,-K]
    return dict(full_delta=full[0].tolist(),schmidt_delta=schmidt[0].tolist(),
                full_reference_error=float(np.max(abs(full[1]-reference))),
                schmidt_reference_error=float(np.max(abs(schmidt[1]-E@J@E.T))),
                schmidt_cross=schmidt[1][:2,2].tolist(), schmidt_variance=float(schmidt[1][2,2]),
                min_eigenvalues=[float(np.linalg.eigvalsh(q[1]).min()) for q in (full,schmidt,projected)],
                symmetry_errors=[float(abs(q[1]-q[1].T).max()) for q in (full,schmidt,projected)],
                full_trace=float(np.trace(full[1])),schmidt_trace=float(np.trace(schmidt[1])),
                projected_trace=float(np.trace(projected[1])))


def health_scenario():
    h=Health()
    args=[('ndt','n1',0,0,{}),('ndt','n2',1,1,{'rank':3}),
          ('ndt','n3',2,2,{'peaks':2}),('gnss','g1',2,2,{}),
          ('gnss','g2',4,2,{}),('kistler','k1',4,None,{}),
          ('wheel','w1',4,4,{'consistent':False}),('ndt','n4',4,4,{'converged':False}),
          ('ndt','n5',5,5,{'consistent':False}),('gnss','g3',5,5,{}),
          ('ndt','n1',5,5,{})]
    return [h.assess(s,g,n,t,**kw) for s,g,n,t,kw in args]


def run():
    return dict(hierarchy=hierarchy(), gaussian=gaussian(),geometry=geometry(),health=health_scenario())

if __name__=='__main__':
    Path(__file__).with_name('results.json').write_text(json.dumps(run(),indent=2)+'\n')
