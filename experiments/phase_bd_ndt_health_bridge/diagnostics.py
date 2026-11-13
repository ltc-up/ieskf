"""Transparent offline diagnostics; no acceptance gate or estimator mutation."""
import numpy as np

def period_two(scores, minimum_alternations=4):
    s=np.asarray(scores,float)
    if len(s)<minimum_alternations+2 or not np.isfinite(s).all():return False
    d=np.diff(s);alternate=d[1:]*d[:-1]<0
    recurrence=np.abs(s[2:]-s[:-2])
    amplitude=np.abs(s[2:]-s[1:-1])
    cycle=alternate&(recurrence<.1*np.maximum(amplitude,1e-9))&(amplitude>1e-4)
    return any(np.all(cycle[i:i+minimum_alternations]) for i in range(len(cycle)-minimum_alternations+1))

def scaled_xy_yaw(score_hessian, rotation_chart_jacobian, length):
    """World XY/yaw conditional curvature, preserving coupling; not a covariance.

    rotation_chart_jacobian maps additive Euler increments to world small-angle
    increments. The third coordinate is length*yaw, in the same length unit as XY.
    """
    if not np.isfinite(length) or length<=0:raise ValueError('positive length required')
    h=np.asarray(score_hessian,float);J=np.asarray(rotation_chart_jacobian,float)
    if h.shape!=(6,6) or J.shape!=(3,3):raise ValueError('matrix shape')
    if not np.isfinite(h).all() or not np.isfinite(J).all():raise ValueError('nonfinite')
    B=np.zeros((6,3));B[0,0]=B[1,1]=1
    B[3:,2]=np.linalg.solve(J,np.array([0.,0.,1.]))/length
    info=-B.T@((h+h.T)*.5)@B
    values,vectors=np.linalg.eigh(info)
    return info,values,vectors
