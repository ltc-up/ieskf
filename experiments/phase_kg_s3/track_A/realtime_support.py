"""CPU-only transaction helpers for the separate actual-wallclock harness."""
import copy
import numpy as np
from score_map import prior_identity

class HistoricalGeometryRequiresAsyncRebuild(ValueError):
    pass

class PreparedGeometry:
    """A worker result commits only on its exact pre-observation prior.

    Geometry is NEVER invoked on the IMU/Wheel event thread. A changed prior
    before initial admission schedules a new worker job. A later input that
    changes an already committed historical prior raises an explicit unsupported
    asynchronous-rebuild failure; no stale result or synchronous GPU fallback
    is applied. This limitation must produce NO-GO if encountered in a run.
    """
    def __init__(self,uid,prior_hash,state,P,info):
        self.uid=uid;self.prior_hash=prior_hash;self.state=state.copy();self.P=P.copy();self.info=copy.deepcopy(info)
    def __deepcopy__(self,memo):return self
    def correct(self,x,P,T,R=None,projection=None):
        actual=prior_identity(x,P)
        if actual!=self.prior_hash:raise HistoricalGeometryRequiresAsyncRebuild('HISTORICAL_GEOMETRY_REQUIRES_ASYNC_REBUILD:'+self.uid)
        return self.state.copy(),self.P.copy(),copy.deepcopy(self.info)

class ExpiredHistory(ValueError):
    def __init__(self,ref,base):
        self.ref=ref;self.base=base
        super().__init__('EXPIRED_HISTORY')

def replay_prior(replay,ref,clock_id):
    if ref<=replay.base[0].t:raise ExpiredHistory(ref,replay.base[0].t)
    result=replay.ingest(dict(kind='clock',header_ns=ref,message_id=clock_id))
    if result=='OUTSIDE_CHECKPOINT_HISTORY':raise ExpiredHistory(ref,replay.base[0].t)
    for z in reversed(replay.snapshots):
        if z[0].t==ref:return z
    raise ExpiredHistory(ref,replay.base[0].t)

class AttemptCost:
    """Full first-worker→commit elapsed time plus every attempt's compute."""
    def __init__(self):
        self.first_worker_begin_ns=None
        self.totals=dict(worker_wall_ms=0.,align_ms=0.,geometry_ms=0.,deskew_ms=0.,attempts=0)
    def add(self,result):
        begin=int(result['worker_begin_ns'])
        if self.first_worker_begin_ns is None:self.first_worker_begin_ns=begin
        else:self.first_worker_begin_ns=min(self.first_worker_begin_ns,begin)
        for key in ['worker_wall_ms','align_ms','geometry_ms','deskew_ms']:self.totals[key]+=float(result[key])
        self.totals['attempts']+=1
    def elapsed_ms(self,finish_ns):
        if self.first_worker_begin_ns is None:raise ValueError('NO_COMPLETED_WORKER_ATTEMPT')
        return (int(finish_ns)-self.first_worker_begin_ns)/1e6
