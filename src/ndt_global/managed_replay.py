"""Opt-in 15D lifecycle and bounded rollback; production queues are untouched."""
import copy
from dataclasses import dataclass
import numpy as np
from .optical_replay import OpticalReplay

@dataclass
class Record:
    uid: str
    stamp_ns: int
    status: str = 'PENDING'
    reason: str = ''
    revision: int = 0

class ManagedReplay(OpticalReplay):
    """Retraction is a historical revision, distinct from initial admission.

    All retained events are replayed from the unmodified oldest checkpoint.
    Immutable terminal IDs prevent resurrection even after checkpoint eviction.
    """
    def __init__(self, *args, gnss_enabled=False, **kwargs):
        if gnss_enabled:
            raise ValueError('GNSS_RESEARCH_INPUT_FORBIDDEN_USE_PRODUCTION_PATH')
        super().__init__(*args, **kwargs)
        self.records = {}
        self.pending_ids = set()
        self.revision_log = []
        self.policy = 'baseline'
    @staticmethod
    def identity(e):
        return str(e.get('observation_id', f"{e['kind']}:{e['message_id']}"))
    def fork(self):
        return copy.deepcopy(self)
    def ingest(self, e):
        if e['kind'] not in ('imu', 'wheel', 'optical', 'clock', 'ndt'):
            raise ValueError('UNAUTHORIZED_ONLINE_SOURCE')
        if e.get('source', '').lower().startswith(('gnss', 'gps')):
            raise ValueError('GNSS_DISABLED')
        if e['kind'] != 'ndt':
            z=copy.deepcopy(e);result=super().ingest(z)
            if 'last_update' in z:e['last_update']=copy.deepcopy(z['last_update'])
            self._refresh_pending()
            return result
        uid=self.identity(e)
        if uid in self.records:
            return 'DUPLICATE_TERMINAL' if self.records[uid].status!='PENDING' else 'DUPLICATE_PENDING'
        r=self.records[uid]=Record(uid,int(e['header_ns']));self.pending_ids.add(uid)
        if e['header_ns']<=self.base[0].t:
            r.status='EXPIRED';r.reason='OUTSIDE_CHECKPOINT_HISTORY';self.pending_ids.discard(uid);return r.reason
        z=copy.deepcopy(e)
        # Never commit a pose until its update has actually run successfully.
        try:
            result=super().ingest(z)
        except Exception:
            r.status='REJECTED';r.reason='INGEST_EXCEPTION';self.pending_ids.discard(uid);raise
        if 'last_update' not in z:
            r.reason=result
            if result in ('DUPLICATE_INFORMATION_GROUP','DUPLICATE','OUTSIDE_CHECKPOINT_HISTORY'):
                r.status='EXPIRED' if result=='OUTSIDE_CHECKPOINT_HISTORY' else 'REJECTED';self.pending_ids.discard(uid)
            return result
        e['last_update']=copy.deepcopy(z['last_update'])
        r.status='ACCEPTED' if z['last_update']['accepted'] else 'REJECTED'
        r.reason=z['last_update']['reason'];self.pending_ids.discard(uid)
        if r.status=='REJECTED':self._remove(uid)
        return result
    def _refresh_pending(self):
        if not self.pending_ids:return
        retained={self.identity(e):e for e in self.events if e['kind']=='ndt'}
        rejected=[]
        for uid in list(self.pending_ids):
            r=self.records[uid]
            if r.status!='PENDING':self.pending_ids.discard(uid);continue
            e=retained.get(uid)
            if e and 'last_update' in e:
                r.status='ACCEPTED' if e['last_update']['accepted'] else 'REJECTED'
                r.reason=e['last_update']['reason']
                if r.status=='REJECTED':rejected.append(uid)
            elif r.stamp_ns<=self.base[0].t:r.status='EXPIRED';r.reason='OUTSIDE_CHECKPOINT_HISTORY'
        self.pending_ids={uid for uid in self.pending_ids if self.records[uid].status=='PENDING'}
        for uid in rejected:self._remove(uid)
    def reject(self, uid, stamp_ns, reason='EXPLICIT_REJECTION'):
        uid=str(uid)
        if uid in self.records:return 'ALREADY_'+self.records[uid].status
        self.records[uid]=Record(uid,int(stamp_ns),'REJECTED',reason)
        return 'REJECTED'
    def _remove(self, uid):
        self.events=[e for e in self.events if self.identity(e)!=uid]
        self.keys=[self.key(e) for e in self.events]
        self.seen=set(self.keys)
        # Keep information-group tombstones even when events age out or are revoked.
        checkpoint=copy.deepcopy(self.base);self.snapshots=[]
        for e in self.events:
            checkpoint=self.step(checkpoint,e);self.snapshots.append(checkpoint)
        self.replayed+=len(self.events)
        # A historical revision may invalidate a later domain-limited pose update.
        # Such an update is explicitly revoked, never left labelled accepted.
        for event in list(self.events):
            other=self.identity(event)
            if event['kind']=='ndt' and other in self.records and self.records[other].status=='ACCEPTED' and not event.get('last_update',{}).get('accepted',False):
                rec=self.records[other];rec.status='REJECTED';rec.revision+=1;rec.reason='REPLAY_INVALID_AFTER_REVOCATION'
                self.revision_log.append(dict(uid=other,stamp_ns=rec.stamp_ns,revision=rec.revision,reason=rec.reason))
                self._remove(other)
                break
    def retract(self, uid, reason='HISTORICAL_REVOCATION'):
        uid=str(uid);r=self.records[uid]
        if r.status!='ACCEPTED':raise ValueError('NOT_ACCEPTED')
        if r.stamp_ns<=self.base[0].t:raise ValueError('REVOCATION_OUTSIDE_HISTORY')
        if not any(self.identity(e)==uid for e in self.events):raise ValueError('REVOCATION_EVENT_NOT_RETAINED')
        self._remove(uid)
        r.revision+=1;r.status='REJECTED';r.reason=reason
        self.revision_log.append(dict(uid=uid,stamp_ns=r.stamp_ns,revision=r.revision,reason=reason))
        self.validate()
    def validate(self):
        x,P,_=self.current()
        if P.shape!=(15,15) or not np.isfinite(P).all() or not np.allclose(P,P.T,atol=1e-9):raise ValueError('INVALID_STATE_COVARIANCE')
        if np.linalg.eigvalsh(P).min() < -1e-9:raise ValueError('INDEFINITE_STATE_COVARIANCE')
        if not np.isfinite(np.r_[x.p,x.v,x.R.ravel(),x.bg,x.ba]).all():raise ValueError('INVALID_STATE')
        return True

class LegacyComparisonReplay(ManagedReplay):
    """Original replay semantics, with only copying/validation for the new runner.

    No terminal ledger, admission policy or historical retraction is enabled.
    This is an explicit comparison path, never the robust lifecycle backend.
    """
    def ingest(self,e):
        if e['kind'] not in ('imu','wheel','optical','clock','ndt'):
            raise ValueError('UNAUTHORIZED_ONLINE_SOURCE')
        if e.get('source','').lower().startswith(('gnss','gps')):
            raise ValueError('GNSS_DISABLED')
        z=copy.deepcopy(e);result=OpticalReplay.ingest(self,z)
        if 'last_update' in z:e['last_update']=copy.deepcopy(z['last_update'])
        return result
    def reject(self,*args,**kwargs):raise ValueError('LEGACY_COMPARISON_NO_REJECTION_POLICY')
    def retract(self,*args,**kwargs):raise ValueError('LEGACY_COMPARISON_NO_RETRACTION')
