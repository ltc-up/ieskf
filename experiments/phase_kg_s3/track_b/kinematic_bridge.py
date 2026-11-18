"""Independent KINEMATIC_BK observer; frozen Legacy modules are only reused.

Root runner feeds recorder-order motion events and immutable frame packets.
Every motion/event release can retry the head delayed scan; later scans never
skip it. Old Source/Target rules and original NDT services are unchanged.
"""
from collections import deque
import hashlib
from dataclasses import dataclass
import time
import numpy as np
from local_odometry import Scan, pose
from kinematic_predictor import IntervalKinematicPredictor

TARGET_TO_INTERNAL={'NONE':'B-P','PREVIOUS':'B0','ROLLING':'B-M','KEYFRAME':'B-K','KEYFRAME_MR':'B-K-MR'}


def original_legacy_initial_O0(U):
    """One-time approved Legacy first-scan axis only; not a motion increment."""
    U=pose(U);yaw=np.arctan2(U[1,0],U[0,0]);c,s=np.cos(yaw),np.sin(yaw)
    H=np.eye(4);H[:3,:3]=[[c,s,0],[-s,c,0],[0,0,1]];H[:3,3]=-H[:3,:3]@U[:3,3]
    return H@U


def from_common_motion(event):
    kind=event['kind'];r=dict(kind=kind,header_ns=event['header'],available_ns=event['available'],message_id=event['id'])
    if kind=='imu':r['omega_base']=event['omega']
    elif kind=='wheel':r.update(speed_base=event['speed'],variance=event['variance'])
    else:raise ValueError('not a common motion event')
    return r


@dataclass
class PendingFrame:
    scan: Scan
    original_O0: object
    enqueued_monotonic_ns: int
    attempts: int=0
    @property
    def bytes(self):return self.scan.dense.nbytes+self.scan.source.nbytes+self.scan.point_ids.nbytes


class KinematicShadowGroup:
    def __init__(self,init_ns,executable,scratch,targets=('NONE','PREVIOUS','ROLLING','KEYFRAME'),audit=None,group_factory=None,requested_end_ns=None,maximum_motion_samples=1024,maximum_pending_frames=10,queue_bytes=128*1024**2):
        if len(set(targets))!=len(targets) or any(x not in TARGET_TO_INTERNAL for x in targets):raise ValueError('invalid target axis')
        if group_factory is None:
            from bridge import ShadowGroup
            group_factory=ShadowGroup
        self.targets=tuple(targets);self.group=group_factory(executable,scratch,modes=tuple(TARGET_TO_INTERNAL[x] for x in targets))
        self.predictor=IntervalKinematicPredictor(init_ns,audit,maximum_samples=maximum_motion_samples)
        self.last_ref=int(init_ns);self.Ukin=np.eye(4);self.H_init=None;self.pending=deque();self.queue_bytes=0
        self.max_records=maximum_pending_frames;self.max_bytes=queue_bytes;self.terminal=None;self.requested_end_ns=requested_end_ns
        self.current_available=None;self.last_enqueued_ref=None;self.completed=0;self.delay_attempts=0;self.T_map_from_O=None;self.initial_O0_sha256=None
    def _terminate(self,status,reason,available,frame_id=None,detail=None):
        self.terminal=dict(status=status,reason=reason,query_available_ns=available,frame_id=frame_id,last_committed_ref_ns=self.last_ref,requested_end_ns=self.requested_end_ns,remaining_requested_range_ns=None if self.requested_end_ns is None else [self.last_ref,self.requested_end_ns],completed_scans=self.completed,queued_frames=len(self.pending),detail=detail)
        return [self.terminal.copy()]
    def _advance_available(self,available):
        available=int(available)
        if self.current_available is not None and available<self.current_available:raise ValueError('OBSERVER_AVAILABLE_REVERSAL')
        self.current_available=available
    def on_motion(self,event):
        available=int(event['available_ns']);self._advance_available(available)
        if self.terminal:return []
        try:
            self.predictor.ingest(event)
            if self.completed==0 and not self.pending and event['header_ns']<self.predictor.init_ns:self.predictor.prune_before(self.predictor.init_ns)
        except MemoryError as exc:return self._terminate('RESOURCE_FAILURE',str(exc),available)
        except ValueError as exc:return self._terminate('INVALID_INPUT',str(exc),available)
        return self._drain(available)
    def on_input_available(self,available_ns):
        """Also call for each real non-motion release (no fabricated timer tick)."""
        self._advance_available(available_ns)
        return [] if self.terminal else self._drain(int(available_ns))
    def on_frame(self,*,frame_id,ref_ns,available_ns,dense,source,deskew_history,legacy_initial_O0=None,legacy_initial_T_map_from_O=None,point_ids=None):
        self._advance_available(available_ns)
        if self.terminal:return []
        if self.last_enqueued_ref is not None and (legacy_initial_O0 is not None or legacy_initial_T_map_from_O is not None):raise ValueError('LATER_LEGACY_INITIALIZATION_FORBIDDEN')
        if self.last_enqueued_ref is not None and ref_ns<=self.last_enqueued_ref:raise ValueError('NONCAUSAL_FRAME_ORDER')
        if self.last_enqueued_ref is None and legacy_initial_O0 is None:raise ValueError('MISSING_ONE_TIME_LEGACY_O0')
        enqueued=time.monotonic_ns()
        if self.last_enqueued_ref is None and legacy_initial_T_map_from_O is not None:self.T_map_from_O=pose(legacy_initial_T_map_from_O)
        scan=Scan.create(frame_id,ref_ns,available_ns,dense,source,deskew_history,point_ids)
        packet=PendingFrame(scan,None if legacy_initial_O0 is None else pose(legacy_initial_O0),enqueued)
        if len(self.pending)>=self.max_records or self.queue_bytes+packet.bytes>self.max_bytes:
            return self._terminate('RESOURCE_FAILURE','PENDING_SOURCE_QUEUE_LIMIT',available_ns,scan.frame_id)
        self.pending.append(packet);self.queue_bytes+=packet.bytes;self.last_enqueued_ref=int(ref_ns)
        return self._drain(int(available_ns))
    def _drain(self,available):
        events=[]
        while self.pending:
            packet=self.pending[0];scan=packet.scan;tick=time.perf_counter()
            result=self.predictor.query(self.last_ref,scan.ref_ns,available,scan.available_ns);packet.attempts+=1
            query_ms=(time.perf_counter()-tick)*1000
            common=dict(predictor='KINEMATIC_BK',frame_id=scan.frame_id,ref_ns=scan.ref_ns,original_cloud_available_ns=scan.available_ns,query_available_ns=available,delay_ns=available-scan.available_ns,attempt=packet.attempts,source_identity=scan.source_identity,dense_identity=scan.dense_identity,query_ms=query_ms)
            if result['status']=='DELAY':
                self.delay_attempts+=1;events.append(dict(common,status='DELAY',reason=result['reason'],missing=result['missing'],delta_T=None));break
            if result['status']!='VALID':
                events.extend(self._terminate(result['status'],result['reason'],available,scan.frame_id,result));break
            self.Ukin=self.Ukin@np.asarray(result['delta_T'])
            if self.H_init is None:
                if packet.original_O0 is None:raise ValueError('MISSING_INITIAL_AXIS')
                self.H_init=packet.original_O0@np.linalg.inv(self.Ukin)
                self.initial_O0_sha256=hashlib.sha256(np.ascontiguousarray(packet.original_O0,dtype='<f8').tobytes()).hexdigest()
            Uadapter=self.H_init@self.Ukin
            start=time.perf_counter()
            # Actual retry available is passed explicitly; original value retained
            # in outer record. Source object/values are those copied at release.
            matched=self.group.on_frame(frame_id=scan.frame_id,ref_ns=scan.ref_ns,available_ns=available,dense=scan.dense,source=scan.source,U_T_base=Uadapter,deskew_history=scan.deskew_history,point_ids=scan.point_ids)
            callback_ms=(time.perf_counter()-start)*1000
            by_target={}
            for target in self.targets:
                z=matched['variants'][TARGET_TO_INTERNAL[target]]
                z=dict(z,predictor='KINEMATIC_BK',target=target,legacy_internal_target_label=TARGET_TO_INTERNAL[target])
                if self.completed==0:
                    z['T_map_from_O']=None if self.T_map_from_O is None else self.T_map_from_O.tolist()
                    z['initial_O0_sha256']=self.initial_O0_sha256
                    z['H_init_sha256']=hashlib.sha256(np.ascontiguousarray(self.H_init,dtype='<f8').tobytes()).hexdigest()
                by_target['KINEMATIC_BK__'+target]=z
            self.pending.popleft();self.queue_bytes-=packet.bytes;self.last_ref=scan.ref_ns;self.completed+=1;self.predictor.prune_before(self.last_ref)
            events.append(dict(common,status='COMMITTED',reason='CAUSAL_INTERVAL_VALID',interval=result,H_init=self.H_init.tolist() if self.completed==1 else None,T_map_from_O=None if self.completed!=1 or self.T_map_from_O is None else self.T_map_from_O.tolist(),initial_O0_sha256=self.initial_O0_sha256,U_kin=self.Ukin.tolist(),variants=by_target,callback_ms=callback_ms,total_processing_ms=query_ms+callback_ms,observed_wall_wait_ms=(time.monotonic_ns()-packet.enqueued_monotonic_ns)/1e6,queue_bytes_after=self.queue_bytes,completed_scans=self.completed))
        return events
    def finish(self,available_ns):
        events=self.on_input_available(available_ns)
        if self.pending and not self.terminal:
            # End of caller stream alone proves neither physical data loss nor
            # illegality of samples that have not yet become available.
            events.append(dict(status='DELAY_UNRESOLVED_AT_CALLER_END',reason='NO_COMPLETE_AUDIT_PROOF_OF_GAP',pending_frames=len(self.pending),last_committed_ref_ns=self.last_ref,query_available_ns=available_ns,requested_end_ns=self.requested_end_ns))
        return events
    def close(self):self.group.close()
