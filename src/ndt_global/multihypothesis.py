"""Small 15D Gaussian bank: branch means are never averaged across modes.
Weights are explicit generalized-Bayes research scores, NOT calibrated integrity.
"""
from dataclasses import dataclass
import pickle
import numpy as np
from scipy.special import logsumexp
from scipy.spatial.transform import Rotation
from .shadow import se3

@dataclass
class Hypothesis:
    uid: str
    replay: object
    backend: str
    log_weight: float = 0.

class GaussianBank:
    def __init__(self, replay, max_branches=2, switch_log_odds=2., confirm_frames=3):
        if max_branches not in (2,3):raise ValueError('SMALL_BANK_ONLY')
        self.branches=[Hypothesis('ndt',replay,'ndt')]
        self.max_branches=max_branches;self.main='ndt';self.seen=set();self.pending=None;self.confirm=0
        self.switch_log_odds=switch_log_odds;self.confirm_frames=confirm_frames;self.switches=[]
        self.weight_events={};self.base_log_weights=None;self.weight_revisions=[]
    @property
    def weights(self):
        w=np.array([b.log_weight for b in self.branches]);return np.exp(w-logsumexp(w))
    def fork(self, parent_uid, uid, backend):
        if self.weight_events and len(self.branches)>1:raise ValueError('FORK_AFTER_EVIDENCE_REQUIRES_NEW_EPOCH')
        if len(self.branches)==1:self.weight_events={};self.base_log_weights=None
        if uid in [b.uid for b in self.branches]:raise ValueError('DUPLICATE_BRANCH')
        if len(self.branches)>=self.max_branches:raise ValueError('BANK_CAPACITY')
        parent=next(b for b in self.branches if b.uid==parent_uid)
        # Split the parent's prior mass; no new evidence is fabricated.
        parent.log_weight-=np.log(2.)
        b=Hypothesis(uid,parent.replay.fork(),backend,parent.log_weight);self.branches.append(b);return b
    def evidence(self, group, costs):
        if group in self.seen:return False
        if set(costs)!=set(b.uid for b in self.branches):raise ValueError('EVIDENCE_BRANCH_SET')
        v=np.array([costs[b.uid] for b in self.branches],float)
        if not np.isfinite(v).all():raise ValueError('INVALID_EVIDENCE')
        # One common held-out observation, competing explanations, no NDT/GICP product.
        if self.base_log_weights is None:self.base_log_weights={b.uid:b.log_weight for b in self.branches}
        self.weight_events[group]=dict(costs)
        v-=v.min()
        for b,c in zip(self.branches,v):b.log_weight-=float(min(c,4.))
        z=logsumexp([b.log_weight for b in self.branches])
        for b in self.branches:b.log_weight-=float(z)
        self.seen.add(group);return True
    def revoke_evidence(self, group):
        if group not in self.weight_events:return False
        del self.weight_events[group]
        for b in self.branches:b.log_weight=self.base_log_weights[b.uid]
        for costs in self.weight_events.values():
            values=np.array([costs[b.uid] for b in self.branches],float);values-=values.min()
            for b,c in zip(self.branches,values):b.log_weight-=float(min(c,4.))
        z=logsumexp([b.log_weight for b in self.branches])
        for b in self.branches:b.log_weight-=float(z)
        self.weight_revisions.append(group);self.pending=None;self.confirm=0
        # seen is a tombstone: revoked evidence cannot reappear through replay.
        return True
    def select(self, stamp):
        best=max(self.branches,key=lambda b:b.log_weight);current=next(b for b in self.branches if b.uid==self.main)
        if best.uid!=self.main and best.log_weight-current.log_weight>self.switch_log_odds:
            self.confirm=self.confirm+1 if self.pending==best.uid else 1;self.pending=best.uid
            if self.confirm>=self.confirm_frames:
                self.switches.append(dict(stamp_ns=int(stamp),old=self.main,new=best.uid));self.main=best.uid;self.pending=None;self.confirm=0
        else:self.pending=None;self.confirm=0
        return next(b for b in self.branches if b.uid==self.main)
    def prune(self, uid):
        if uid==self.main:raise ValueError('CANNOT_PRUNE_MAIN')
        if len(self.branches)<=1:raise ValueError('EMPTY_BANK')
        self.branches=[b for b in self.branches if b.uid!=uid]
    def merge_same_mode(self, keep, remove, translation_tolerance=1e-6, rotation_tolerance=1e-6):
        # Only effectively identical historical states can be merged without discarding
        # branch-specific replay. General moment-matching would destroy that history.
        if self.weight_events:raise ValueError('NO_MERGE_AFTER_EVIDENCE_REQUIRES_NEW_EPOCH')
        a=next(b for b in self.branches if b.uid==keep);b=next(b for b in self.branches if b.uid==remove)
        x,P,_=a.replay.current();y,Q,_=b.replay.current()
        if np.linalg.norm(x.p-y.p)>translation_tolerance or np.linalg.norm(Rotation.from_matrix(x.R.T@y.R).as_rotvec())>rotation_tolerance or not np.allclose(x.v,y.v) or not np.allclose(P,Q) or not np.allclose(x.bg,y.bg) or not np.allclose(x.ba,y.ba) or x.t!=y.t or a.replay.keys!=b.replay.keys or pickle.dumps(a.replay.events,protocol=5)!=pickle.dumps(b.replay.events,protocol=5):
            raise ValueError('DISTINCT_MODES_OR_HISTORIES_NO_MERGE')
        a.log_weight=float(np.logaddexp(a.log_weight,b.log_weight))
        if self.main==remove:self.main=keep
        self.prune(remove)
    @staticmethod
    def distinct(A,B,translation=.1,rotation=.01):
        A,B=se3(A),se3(B)
        return np.linalg.norm(A[:3,3]-B[:3,3])>translation or np.linalg.norm(Rotation.from_matrix(A[:3,:3].T@B[:3,:3]).as_rotvec())>rotation

class ContinuousMapOdom:
    """Versioned frame contract. Does not move odom or command a controller."""
    def __init__(self):self.version=0;self.map_from_odom=np.eye(4)
    def correct(self, map_from_base, odom_from_base):
        A,O=se3(map_from_base),se3(odom_from_base)
        self.map_from_odom=A@np.linalg.inv(O);self.version+=1
        return dict(version=self.version,map_from_odom=self.map_from_odom.copy(),odom_from_base=O.copy())
    def path_in_odom(self, map_points, version):
        if version!=self.version:raise ValueError('FRAME_VERSION_MISMATCH')
        T=np.linalg.inv(self.map_from_odom);p=np.asarray(map_points,float)
        return p@T[:3,:3].T+T[:3,3]
