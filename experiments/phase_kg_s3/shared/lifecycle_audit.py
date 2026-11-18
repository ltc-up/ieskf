"""Passive historical-geometry audit; never changes a correction result.

Each observer invocation is bound to its actual pre-observation full mean/P.
Keep the latest result even after the replay evicts the event. Log full changed
linearizations, and compare final numerical admission with the terminal ledger.
"""
import copy, hashlib, json, time
import numpy as np


def clean(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    raise TypeError(type(x).__name__)


def state(x,P):
    return dict(t=int(x.t),p=x.p.copy(),v=x.v.copy(),R=x.R.copy(),bg=x.bg.copy(),ba=x.ba.copy(),P=P.copy())


class LifecycleAudit:
    def __init__(self,log):
        self.log=log;self.latest={};self.calls=0;self.changed=0;self.audit_ms=0.;self.observer_ms=0.;self.observer_max_ms=0.
    def wrap(self,observer,uid):return AuditedObserver(observer,str(uid),self)
    def record(self,uid,x,P,y,Q,info,observer_ms):
        start=time.perf_counter_ns();self.calls+=1
        self.observer_ms+=observer_ms;self.observer_max_ms=max(self.observer_max_ms,observer_ms)
        # Evaluation/cache counters do not alter the geometric transaction.
        semantic={k:v for k,v in info.items() if k not in ('score_calls',)}
        payload=dict(prior=state(x,P),posterior=state(y,Q),info=semantic)
        encoded=json.dumps(payload,default=clean,sort_keys=True).encode()
        sha=hashlib.sha256(encoded).hexdigest();old=self.latest.get(uid)
        revision=0 if old is None else old['revision']+(old['linearization_sha256']!=sha)
        self.latest[uid]=dict(revision=revision,linearization_sha256=sha,accepted=bool(info.get('accepted')),reason=info.get('reason'),stamp_ns=int(x.t),calls=1 if old is None else old['calls']+1)
        if old is None or old['linearization_sha256']!=sha:
            self.changed+=1
            # JSON round-trip freezes arrays against any future event mutation.
            self.log.put(dict(observation_id=uid,revision=revision,linearization_sha256=sha,actual_observer_invocation=True,observer_compute_wall_ms=observer_ms,**json.loads(encoded)))
        self.audit_ms+=(time.perf_counter_ns()-start)/1e6
    def finish(self,replay):
        records={uid:vars(record).copy() for uid,record in getattr(replay,'records',{}).items()} if replay is not None else {}
        mismatch=[]
        for uid,z in self.latest.items():
            r=records.get(uid)
            if r and r['status'] in ('ACCEPTED','REJECTED') and (r['status']=='ACCEPTED')!=z['accepted']:
                mismatch.append(dict(observation_id=uid,ledger=r,last_numerical_result=z))
        return dict(observer_calls=self.calls,changed_linearizations=self.changed,passive_audit_cpu_ms=self.audit_ms,total_observer_wall_ms=self.observer_ms,max_observer_wall_ms=self.observer_max_ms,
                    latest_numerical_admission=self.latest,terminal_records=records,ledger_numerical_mismatches=mismatch,
                    geometric_model_changed=False,latest_retained_after_event_eviction=True)


class AuditedObserver:
    def __init__(self,observer,uid,audit):self.observer=observer;self.uid=uid;self.audit=audit
    def __deepcopy__(self,memo):return type(self)(copy.deepcopy(self.observer,memo),self.uid,self.audit)
    def correct(self,x,P,T,R=None,projection=None):
        start=time.perf_counter_ns()
        y,Q,info=self.observer.correct(x,P,T,R,projection)
        elapsed=(time.perf_counter_ns()-start)/1e6
        self.audit.record(self.uid,x,P,y,Q,info,elapsed)
        return y,Q,info
