"""Non-Bayesian global anchoring and hypothesis health; no NDT R, no local writeback."""
from dataclasses import dataclass
import numpy as np
from .shadow import Contract,PoseHistory,se3

@dataclass(frozen=True)
class PositionObservation:
    stamp_ns: int
    source_id: str
    correlation_group: str
    position_base_map: object
    contract: Contract
    dimensions: tuple = (0,1,2)
    fix_valid: bool = False
    datum_qualified: bool = False
    reference_qualified: bool = False
    clock_qualified: bool = False
    # Optional independently validated deterministic error bounds, not sigma.
    error_bounds_m: object = None
    bounds_qualified: bool = False
    covariance: object = None
    covariance_calibrated: bool = False
    source_independent_for_selection: bool = False

class GlobalManager:
    def __init__(self,history,orientation_qualified=False,fresh_ns=500_000_000,coast_ns=2_000_000_000):
        self.history=history;self.orientation_qualified=orientation_qualified
        self.fresh_ns,self.coast_ns=fresh_ns,coast_ns
        self.C=np.eye(4);self.anchor_ns=None;self.dimensions=set();self.dimension_stamps={};self.seen=set()
        self.pending=[];self.ambiguity_ns=None;self.last_ndt=None;self.last_gnss=None
        self.health={};self.recovery_count=0;self.state='INITIALIZING';self.last_now=None;self.events=[]
    def _clock(self,now):
        if self.last_now is not None and now<self.last_now:raise ValueError('RECEIPT_TIME_ROLLBACK')
        self.last_now=now
    def _event(self,source,reason,accepted=False,**kw):
        d=dict(source=source,reason=reason,accepted=accepted,trusted=False,**kw)
        if source in self.health:self.health[source].update(reason=reason,accepted=accepted)
        self.events.append(d);return d
    def position(self,g,now):
        self._clock(now);self.last_gnss=dict(received=True,stamp_ns=g.stamp_ns,qualified=False)
        self.health[g.source_id]=self.last_gnss
        try:
            if g.contract!=self.history.contract:raise ValueError('CONTRACT_MISMATCH')
            if not g.correlation_group:raise ValueError('MISSING_IDENTITY')
            if g.correlation_group in self.seen:raise ValueError('DUPLICATE')
            if self.anchor_ns is not None and g.stamp_ns<self.anchor_ns:raise ValueError('OBS_TIME_ROLLBACK')
            if g.stamp_ns>now:raise ValueError('FUTURE')
            if now-g.stamp_ns>self.fresh_ns:raise ValueError('STALE')
            if not g.fix_valid:raise ValueError('FIX_UNQUALIFIED')
            if not (g.datum_qualified and g.reference_qualified and g.clock_qualified):raise ValueError('FRAME_REFERENCE_TIME_UNQUALIFIED')
            dims=tuple(g.dimensions)
            if not dims or len(set(dims))!=len(dims) or any(i not in (0,1,2) for i in dims):raise ValueError('INVALID_DIMENSIONS')
            p=np.asarray(g.position_base_map,float)
            if p.shape!=(3,) or not np.isfinite(p[list(dims)]).all():raise ValueError('INVALID_POSITION')
            O=self.history.at(g.stamp_ns)
            pending=self.pending
            if pending and g.bounds_qualified and g.source_independent_for_selection:
                bounds=np.asarray(g.error_bounds_m,float)
                if bounds.shape!=(3,) or not np.isfinite(bounds[list(dims)]).all() or np.any(bounds[list(dims)]<0):raise ValueError('INVALID_BOUNDS')
                # Compare at the SAME observation time, using only arrived odom history.
                try:old=self.history.at(self.ambiguity_ns)
                except ValueError:old=None  # No stale-mode pruning; do not block a valid position anchor.
                if old is not None:
                    delta=np.linalg.inv(old)@O
                    transported=[T@delta for T in pending]
                    pending=[T for T in transported if np.all(abs(T[list(dims),3]-p[list(dims)])<=bounds[list(dims)])]
            was_lost=self.anchor_ns is None or now-self.anchor_ns>self.fresh_ns
            if was_lost:self.recovery_count=0
            # Coordinate anchoring only, not a Kalman update: preserve all unobserved
            # components including rotation. Global covariance stays UNKNOWN.
            proposed=self.C.copy();shift=self.C[:3,:3]@O[:3,3]
            for i in dims:proposed[i,3]=p[i]-shift[i]
            jump=float(np.linalg.norm(proposed[:3,3]-self.C[:3,3]))
            self.C=proposed;self.anchor_ns=g.stamp_ns;self.dimensions.update(dims);self.seen.add(g.correlation_group)
            self.recovery_count+=1;self.last_gnss['qualified']=True
            for i in dims:self.dimension_stamps[i]=g.stamp_ns
            if pending is not self.pending:
                self.pending=pending
                self.ambiguity_ns=g.stamp_ns if len(pending)>1 else None
            self.state='RECOVERING' if self.recovery_count<2 else ('GLOBAL_TRACKING' if self.orientation_qualified and len(self.dimensions)==3 else 'PARTIAL')
            return self._event(g.source_id,'POSITION_ANCHORED_NONBAYESIAN',True,jump_m=jump,observed_dimensions=list(dims),remaining_hypotheses=len(self.pending),covariance='UNKNOWN_CORRELATION')
        except (ValueError,np.linalg.LinAlgError) as ex:
            return self._event(g.source_id,str(ex))
    def ndt(self,candidates,classification,stamp_ns,group,now,contract,converged=None):
        self._clock(now)
        if contract!=self.history.contract:return self._event('ndt','CONTRACT_MISMATCH')
        if self.last_ndt is not None and stamp_ns<self.last_ndt['stamp_ns']:return self._event('ndt','OBS_TIME_ROLLBACK')
        self.last_ndt=dict(received=True,stamp_ns=stamp_ns,classification=classification,converged=converged,mode_count=len(candidates))
        self.health['ndt']=self.last_ndt
        if not group:return self._event('ndt','MISSING_IDENTITY')
        key='ndt:'+group
        if key in self.seen:return self._event('ndt','DUPLICATE')
        if stamp_ns>now:return self._event('ndt','FUTURE')
        if now-stamp_ns>self.fresh_ns:return self._event('ndt','STALE')
        if classification=='AMBIGUOUS':
            try:self.pending=[se3(T) for T in candidates]
            except ValueError:return self._event('ndt','INVALID_TRANSFORM')
            self.ambiguity_ns=stamp_ns;self.seen.add(key)
            return self._event('ndt','AMBIGUOUS_NO_ANCHOR',hypotheses=len(self.pending))
        # Converged, score, rank or one detected basin never authorizes R or a trusted anchor.
        self.seen.add(key)
        return self._event('ndt',classification+'_NO_QUALIFIED_NDT_OBSERVATION')
    def status(self,now):
        self._clock(now)
        if self.ambiguity_ns is not None and now-self.ambiguity_ns<=self.fresh_ns and len(self.pending)>1:state='AMBIGUOUS'
        elif self.anchor_ns is None:state='INITIALIZING' if not self.events else 'UNAVAILABLE'
        elif now-self.anchor_ns>self.coast_ns:state='UNAVAILABLE'
        elif now-self.anchor_ns>self.fresh_ns:state='COASTING'
        else:state=self.state
        fresh_dims={i for i,t in self.dimension_stamps.items() if now-t<=self.fresh_ns}
        if state=='GLOBAL_TRACKING' and len(fresh_dims)<3:state='PARTIAL'
        unresolved=[d for i,d in enumerate(('x','y','z')) if i not in fresh_dims]
        if not self.orientation_qualified:unresolved+=['roll','pitch','yaw']
        return dict(state=state,last_anchor_ns=self.anchor_ns,local_continuous=True,trusted_global=False,
          source_health={s:v|{'fresh':0<=now-v['stamp_ns']<=self.fresh_ns} for s,v in self.health.items()},
          unresolved=unresolved,ndt=self.last_ndt,gnss=self.last_gnss,covariance='UNKNOWN_CORRELATION',hypotheses=len(self.pending))
    def publish(self,stamp,now):
        status=self.status(now)
        if self.anchor_ns is None or status['state']=='UNAVAILABLE':return dict(published=False,**status)
        try:O=self.history.at(stamp)
        except ValueError as ex:return dict(published=False,reason=str(ex),**status)
        return dict(published=True,pose=self.C@O,correction=self.C.copy(),velocity_semantics='rotate local velocity by R_map_odom; never differentiate correction jumps',**status)
