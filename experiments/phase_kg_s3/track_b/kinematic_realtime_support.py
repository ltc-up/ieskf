"""CPU-only immutable motion/Pose transactions for actual wall-clock B harness.

No CUDA/ROS imports and no 15D state, covariance or global posterior inputs.
"""
import copy
import threading
import numpy as np
from kinematic_predictor import IntervalKinematicPredictor
from kinematic_bridge import from_common_motion


class MotionStore:
    """Event thread appends released measurements; worker/publisher copy atomically.

    No callback can drain a scan or invoke GPU work on the event/publisher thread.
    Pruning retains one held sample per kind and all headers after committed ref.
    """
    def __init__(self, init_ns, audit=None, limit=1024):
        self.init_ns=int(init_ns);self.audit=audit;self.limit=int(limit)
        self._events=[];self._lock=threading.Lock();self._last_available=None
    def append(self,event):
        e=from_common_motion(event)
        # Validate before changing shared storage. No cached state can be global.
        validator=IntervalKinematicPredictor(self.init_ns);validator.ingest(e)
        with self._lock:
            if self._last_available is not None and e['available_ns']<self._last_available:raise ValueError('MOTION_AVAILABLE_REVERSAL')
            if len(self._events)>=self.limit:raise MemoryError('MOTION_SNAPSHOT_RESOURCE_LIMIT')
            self._events.append(copy.deepcopy(e));self._last_available=e['available_ns']
    def prune(self,ref_ns):
        with self._lock:
            held={}
            for e in self._events:
                if e['header_ns']<=ref_ns:
                    key=e['kind'];old=held.get(key)
                    if old is None or (e['header_ns'],e['message_id'])>(old['header_ns'],old['message_id']):held[key]=e
            self._events=[e for e in self._events if e['header_ns']>ref_ns or any(e is x for x in held.values())]
    def predictor(self):
        with self._lock:events=copy.deepcopy(self._events)
        p=IntervalKinematicPredictor(self.init_ns,self.audit,maximum_samples=self.limit)
        for e in events:p.ingest(e)
        return p


class PoseStore:
    """Only completed local-chain worker transactions may advance this snapshot."""
    def __init__(self):self._lock=threading.Lock();self._snapshot=None;self._revision=0
    def commit(self,ref_ns,O_T_base,frame_id,source_identity,motion_ids):
        T=np.asarray(O_T_base,float)
        if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1],rtol=0,atol=1e-10):raise ValueError('INVALID_LOCAL_POSE')
        if not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),rtol=0,atol=1e-6) or abs(np.linalg.det(T[:3,:3])-1)>1e-6:raise ValueError('INVALID_LOCAL_ROTATION')
        with self._lock:
            if self._snapshot is not None and ref_ns<=self._snapshot['ref_ns']:raise ValueError('NONMONOTONIC_LOCAL_COMMIT')
            self._revision+=1;self._snapshot=dict(ref_ns=int(ref_ns),O_T_base=T.copy(),frame_id=str(frame_id),source_identity=str(source_identity),motion_ids=copy.deepcopy(motion_ids),revision=self._revision)
    def read(self):
        with self._lock:return copy.deepcopy(self._snapshot)


def predict_publication(poses,motion,target_ns,available_ns):
    """Extrapolate real motion to requested stamp, or publish health without Pose.

    The cloud anchor is never relabeled with a newer timestamp. No zero/held
    speed fallback is made when actual IMU/Wheel age limits are exceeded.
    """
    snap=None;predictor=None
    for _ in range(3):
        snap=poses.read()
        if snap is None:break
        predictor=motion.predictor()
        latest=poses.read()
        if latest is not None and latest['revision']==snap['revision']:break
    else:return dict(valid=False,reason='CONCURRENT_LOCAL_COMMIT_RETRY_LIMIT',target_ns=int(target_ns))
    if snap is None:return dict(valid=False,reason='NO_COMMITTED_LOCAL_POSE',target_ns=int(target_ns))
    start=snap['ref_ns'];target=int(target_ns)
    if target<start:return dict(valid=False,reason='TARGET_BEFORE_LOCAL_ANCHOR',target_ns=target,anchor_ref_ns=start)
    if target==start:
        return dict(valid=True,reason='EXACT_LOCAL_ANCHOR',target_ns=target,anchor_ref_ns=start,O_T_base=snap['O_T_base'].tolist(),revision=snap['revision'],anchor_frame_id=snap['frame_id'],source_identity=snap['source_identity'],motion_ids=[])
    r=predictor.query(start,target,int(available_ns),int(available_ns))
    if r['status']!='VALID':return dict(valid=False,reason=r['status'],detail=r,target_ns=target,anchor_ref_ns=start,revision=snap['revision'])
    T=snap['O_T_base']@np.asarray(r['delta_T'])
    return dict(valid=True,reason='CAUSAL_SE3_EXTRAPOLATION',target_ns=target,anchor_ref_ns=start,O_T_base=T.tolist(),revision=snap['revision'],anchor_frame_id=snap['frame_id'],source_identity=snap['source_identity'],motion_ids=r['used_motion_ids'],interval=r)
