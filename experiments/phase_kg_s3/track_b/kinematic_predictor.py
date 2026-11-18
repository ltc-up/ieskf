"""Approved independent interval kinematics. No global/LiDAR posterior input.

Full-stream header metadata may establish an input gap; it never supplies future
measurement VALUES to a query. Query availability always remains explicit.
"""
from dataclasses import dataclass
import bisect
import hashlib
import json
import numpy as np


def hat(w):
    x,y,z=w
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def twist_step(T, omega, speed, dt):
    """Exact constant body twist integration; base vy=vz=0 assumption."""
    if not np.isfinite(dt) or dt<0:raise ValueError('invalid dt')
    phi=np.asarray(omega,float)*dt;A=hat(phi);theta2=float(phi@phi)
    if theta2<1e-12:
        sinc=1-theta2/6+theta2*theta2/120
        c2=.5-theta2/24+theta2*theta2/720
        c3=1/6-theta2/120+theta2*theta2/5040
    else:
        theta=np.sqrt(theta2);sinc=np.sin(theta)/theta;c2=(1-np.cos(theta))/theta2;c3=(theta-np.sin(theta))/(theta2*theta)
    E=np.eye(3)+sinc*A+c2*A@A;J=np.eye(3)+c2*A+c3*A@A
    out=T.copy();out[:3,3]+=T[:3,:3]@J@np.array([speed*dt,0.,0.]);out[:3,:3]=T[:3,:3]@E
    return out


@dataclass(frozen=True)
class MotionSample:
    kind: str
    header_ns: int
    available_ns: int
    message_id: int
    value: tuple
    variance: float=0.


class HeaderCoverageAudit:
    """Complete ORIGINAL header index only; no gyro/speed values or interpolation."""
    def __init__(self, imu_headers, wheel_headers, covers_from_ns, covers_to_ns, identity):
        self.headers={'imu':np.unique(np.asarray(imu_headers,dtype=np.int64)),'wheel':np.unique(np.asarray(wheel_headers,dtype=np.int64))}
        self.start=int(covers_from_ns);self.end=int(covers_to_ns);self.identity=str(identity)
    def gaps(self,start,end):
        if start<self.start or end>self.end:return None
        gaps=[]
        for kind,limit in [('imu',25_000_000),('wheel',100_000_000)]:
            h=self.headers[kind];i=int(np.searchsorted(h,start,side='right'))-1
            cursor=int(start)
            if i<0:
                first=int(h[0]) if len(h) else int(end)
                gaps.append(dict(kind=kind,start_ns=cursor,end_ns=min(end,first),reason='NO_AUDITED_HELD_PREFIX'));continue
            while cursor<end:
                held=int(h[i]);nxt=int(h[i+1]) if i+1<len(h) else int(end)
                stop=min(int(end),nxt)
                if stop-held>limit:gaps.append(dict(kind=kind,start_ns=max(cursor,held+limit),end_ns=stop,reason='AUDITED_HEADER_COVERAGE_GAP'))
                if nxt>=end:break
                cursor=nxt;i+=1
        return gaps


class IntervalKinematicPredictor:
    def __init__(self, init_ns, audit=None, maximum_samples=1024):
        self.init_ns=int(init_ns);self.audit=audit;self.maximum_samples=maximum_samples
        self.samples={'imu':[],'wheel':[]};self.identities={};self.last_available=None
        self.bias=np.zeros(3);self.bias.flags.writeable=False  # Approved original legal-init fixed zero, never updated.
    def ingest(self,event):
        kind=event['kind']
        if kind not in self.samples:raise ValueError('motion source not permitted')
        header=int(event['header_ns']);available=int(event['available_ns']);ident=int(event['message_id'])
        if kind=='imu':
            value=np.asarray(event['omega_base'],float)
            if value.shape!=(3,) or not np.isfinite(value).all():raise ValueError('INVALID_GYRO')
            variance=0.
        else:
            value=np.asarray([event['speed_base']],float);variance=float(event['variance'])
            if not np.isfinite(value).all() or not np.isfinite(variance) or variance<0:raise ValueError('INVALID_WHEEL')
        sample=MotionSample(kind,header,available,ident,tuple(map(float,value)),variance)
        key=(kind,ident)
        if key in self.identities:
            if self.identities[key]!=sample:raise ValueError('MOTION_IDENTITY_CHANGED')
            return 'DUPLICATE'
        if self.last_available is not None and available<self.last_available:raise ValueError('AVAILABLE_REVERSAL')
        if sum(map(len,self.samples.values()))>=self.maximum_samples:raise MemoryError('RESOURCE_FAILURE_MOTION_BUFFER')
        self.identities[key]=sample;self.samples[kind].append(sample);self.last_available=available
        return 'INGESTED'
    def prune_before(self,ref_ns):
        # Preserve the newest header<=committed ref for each stream as held prefix;
        # any future delayed interval must still reference a retained query window.
        for kind,items in self.samples.items():
            before=[s for s in items if s.header_ns<=ref_ns]
            keep=max(before,key=lambda s:(s.header_ns,s.message_id),default=None)
            new=[s for s in items if s.header_ns>ref_ns or s is keep]
            self.samples[kind]=new
        self.identities={(s.kind,s.message_id):s for items in self.samples.values() for s in items}
    def query(self,prev_ref_ns,ref_ns,query_available_ns,original_cloud_available_ns):
        start,end=int(prev_ref_ns),int(ref_ns);available=int(query_available_ns);original=int(original_cloud_available_ns)
        if start<self.init_ns or end<=start:raise ValueError('INVALID_INTERVAL')
        if available<original:raise ValueError('QUERY_BEFORE_ORIGINAL_AVAILABLE')
        selected={kind:sorted((s for s in items if s.available_ns<=available and s.header_ns<=end),key=lambda s:(s.header_ns,s.message_id)) for kind,items in self.samples.items()}
        boundaries=sorted({start,end}|{s.header_ns for items in selected.values() for s in items if start<s.header_ns<end})
        T=np.eye(4);segments=[];used=set();missing=[]
        for a,b in zip(boundaries,boundaries[1:]):
            held={}
            for kind,limit in [('imu',25_000_000),('wheel',100_000_000)]:
                candidates=[s for s in selected[kind] if s.header_ns<=a]
                sample=candidates[-1] if candidates else None
                if sample is None or b-sample.header_ns>limit:
                    missing.append(dict(kind=kind,start_ns=a,end_ns=b,held_header_ns=None if sample is None else sample.header_ns,limit_ns=limit))
                else:held[kind]=sample
            if len(held)!=2:continue
            gyro=held['imu'];wheel=held['wheel'];T=twist_step(T,np.asarray(gyro.value)-self.bias,wheel.value[0],(b-a)*1e-9)
            used.update([(gyro.kind,gyro.message_id),(wheel.kind,wheel.message_id)])
            segments.append(dict(start_ns=a,end_ns=b,imu_id=gyro.message_id,wheel_id=wheel.message_id,imu_header_ns=gyro.header_ns,wheel_header_ns=wheel.header_ns,imu_available_ns=gyro.available_ns,wheel_available_ns=wheel.available_ns))
        common=dict(prev_ref_ns=start,ref_ns=end,original_cloud_available_ns=original,query_available_ns=available,query_delay_ns=available-original,segments=segments,used_motion_ids=sorted([list(x) for x in used]),bias_base_rad_s=self.bias.tolist(),body_lateral_vertical_zero=True)
        if missing:
            audited=None if self.audit is None else self.audit.gaps(start,end)
            if audited:
                return dict(common,status='IRRECOVERABLE_INPUT_GAP',reason='COMPLETE_HEADER_AUDIT_PROVES_COVERAGE_GAP',missing=missing,audited_gaps=audited,audit_identity=self.audit.identity,delta_T=None)
            return dict(common,status='DELAY',reason='CURRENT_AVAILABLE_INPUT_CANNOT_COVER_INTERVAL',missing=missing,audited_gaps=audited,delta_T=None)
        return dict(common,status='VALID',reason='CAUSAL_ZOH_COVERED',delta_T=T.tolist())


def earliest_retry(predictor,prev_ref_ns,ref_ns,original_available_ns,input_events):
    """Offline scheduler: only retry at actual next input availability, in order.

    Caller provides original recorder-order events (may include kind='clock_tick'
    with availability only). Each future VALUE is ingested only at its release.
    Never relabel a later legal result with original availability.
    """
    attempts=[];r=predictor.query(prev_ref_ns,ref_ns,original_available_ns,original_available_ns);attempts.append(r)
    if r['status']!='DELAY':return r,attempts
    for event in input_events:
        available=int(event['available_ns'])
        if available<original_available_ns:raise ValueError('RETRY_EVENT_BEFORE_QUERY')
        if event['kind'] in ('imu','wheel'):predictor.ingest(event)
        r=predictor.query(prev_ref_ns,ref_ns,available,original_available_ns);attempts.append(r)
        if r['status']!='DELAY':return r,attempts
    return dict(r,status='DELAY',reason='AVAILABLE_RETRY_STREAM_EXHAUSTED_NOT_PROVEN_GAP'),attempts
