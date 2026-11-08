from dataclasses import dataclass, field
from bisect import bisect_left
import numpy as np
from scipy.spatial.transform import Rotation

UNKNOWN = 'UNKNOWN'

def se3(T):
    T = np.array(T, dtype=float, copy=True)
    if (T.shape != (4,4) or not np.isfinite(T).all() or
        not np.allclose(T[3], [0,0,0,1], atol=1e-10) or
        not np.allclose(T[:3,:3].T @ T[:3,:3], np.eye(3), atol=2e-5) or
        abs(np.linalg.det(T[:3,:3])-1) > 2e-5):
        raise ValueError('INVALID_TRANSFORM')
    return T

@dataclass(frozen=True)
class Contract:
    map_version: str
    extrinsic_version: str
    epoch: str
    map_frame: str = 'map'
    odom_frame: str = 'research_odom'
    reference: str = 'base_link'
    direction: str = 'parent_from_child'
    def valid(self):
        return all(v and v != UNKNOWN for v in self.__dict__.values())

@dataclass(frozen=True)
class Candidate:
    stamp_ns: int
    source_id: str
    correlation_group: str
    pose: object
    contract: Contract
    converged: object = None
    fitness: object = None
    backend_health: str = UNKNOWN
    lidars: tuple = ()
    seed_id: str = UNKNOWN
    failure: str = ''
    support: object = None
    directions: object = None
    modes: object = None
    covariance: object = None

class PoseHistory:
    def __init__(self, contract, horizon_ns=2_000_000_000, max_gap_ns=100_000_000):
        if not contract.valid(): raise ValueError('UNKNOWN_CONTRACT')
        self.contract, self.horizon_ns, self.max_gap_ns = contract, horizon_ns, max_gap_ns
        self.stamps, self.poses = [], []
    def add(self, stamp, pose, contract):
        if contract != self.contract: raise ValueError('CONTRACT_MISMATCH')
        T = se3(pose)
        if self.stamps and stamp <= self.stamps[-1]: raise ValueError('ODOM_TIME_ROLLBACK_OR_DUPLICATE')
        self.stamps.append(stamp); self.poses.append(T)
        # Keep one bracketing sample before horizon, but total memory remains bounded
        # for physically timestamped finite-rate data. Hard cap guards malformed rates.
        while len(self.stamps)>2 and self.stamps[1] < stamp-self.horizon_ns:
            self.stamps.pop(0); self.poses.pop(0)
        if len(self.stamps)>10000:
            self.stamps.pop(0); self.poses.pop(0)
    def at(self, stamp):
        i=bisect_left(self.stamps, stamp)
        if i<len(self.stamps) and self.stamps[i]==stamp: return self.poses[i].copy()
        if i==0 or i==len(self.stamps): raise ValueError('NO_BRACKET')
        a,b=self.stamps[i-1:i+1]
        if b-a>self.max_gap_ns: raise ValueError('HISTORY_GAP')
        u=(stamp-a)/(b-a); A,B=self.poses[i-1:i+1]; T=np.eye(4)
        T[:3,3]=(1-u)*A[:3,3]+u*B[:3,3]
        T[:3,:3]=A[:3,:3]@Rotation.from_rotvec(u*Rotation.from_matrix(A[:3,:3].T@B[:3,:3]).as_rotvec()).as_matrix()
        return T

class ShadowCorrector:
    """Candidate consumption and publication are distinct; output is UNQUALIFIED.

    Only pose history is accepted. This class cannot write a local state/P.
    Latency bounds are scheduling contracts, never noise calibration.
    """
    def __init__(self, history, max_delay_ns=500_000_000):
        self.history=history; self.max_delay_ns=max_delay_ns
        self.seen=set(); self.last_stamp=None; self.correction=None; self.anchor=None
        self.events=[]
    def consume(self, c, now_ns):
        reason='SHADOW_ACCEPTED'; C=None
        try:
            if c.contract != self.history.contract or not c.contract.valid(): raise ValueError('CONTRACT_MISMATCH')
            if not c.source_id or not c.correlation_group: raise ValueError('MISSING_IDENTITY')
            if c.correlation_group in self.seen: raise ValueError('DUPLICATE')
            if c.stamp_ns > now_ns: raise ValueError('FUTURE')
            if now_ns-c.stamp_ns > self.max_delay_ns: raise ValueError('STALE')
            if self.last_stamp is not None and c.stamp_ns < self.last_stamp: raise ValueError('OBS_TIME_ROLLBACK')
            if c.converged is not True: raise ValueError('NOT_CONVERGED' if c.converged is False else 'CONVERGENCE_UNKNOWN')
            if c.backend_health != 'OK': raise ValueError('BACKEND_UNHEALTHY_OR_UNKNOWN')
            M=se3(c.pose); O=self.history.at(c.stamp_ns); C=M@np.linalg.inv(O)
        except (ValueError, np.linalg.LinAlgError) as e: reason=str(e)
        accepted=C is not None
        if accepted:
            self.correction=C; self.anchor=c.stamp_ns; self.last_stamp=c.stamp_ns
            self.seen.add(c.correlation_group)
        event=dict(candidate_received=True,accepted=accepted,reason=reason,stamp_ns=c.stamp_ns,
                   covariance='UNCALIBRATED_CORRELATION_UNKNOWN',trusted=False)
        self.events.append(event)
        return event
    def publish(self, stamp_ns):
        if self.correction is None: return dict(published=False,reason='NO_CORRECTION',trusted=False)
        try: O=self.history.at(stamp_ns)
        except ValueError as e: return dict(published=False,reason=str(e),trusted=False)
        return dict(published=True,pose=self.correction@O,correction=self.correction.copy(),
                    anchor_ns=self.anchor,stamp_ns=stamp_ns,trusted=False,
                    status='SHADOW_UNQUALIFIED',covariance='UNCALIBRATED_CORRELATION_UNKNOWN')
