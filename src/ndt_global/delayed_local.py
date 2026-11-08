"""Bounded offline checkpoint/repropagation prototype using the existing 15D models.
Each observation is reapplied once from its pre-observation checkpoint, not fused
again into a posterior containing that observation. No map input or new Q/R.
This is not the production augmented-state filter or a ROS executor.
"""
from bisect import bisect_right
import numpy as np
from geometry_ieskf.inertial import InertialPredictor
from geometry_ieskf.wheel import WheelObservation

class DelayedLocalReplay:
    def __init__(self,state,P,imu,lever,horizon_ns=500_000_000):
        self.base=(state.copy(),P.copy(),{k:np.array(v,copy=True) if k!='header_ns' else v for k,v in imu.items()})
        self.events=[];self.snapshots=[];self.keys=[];self.seen=set();self.horizon_ns=horizon_ns
        self.predictor=InertialPredictor();self.wheel=WheelObservation(lever);self.replayed=0;self.max_buffer=0
    @staticmethod
    def key(e):return (e['header_ns'],0 if e['kind']=='imu' else 1,e['message_id'])
    def step(self,checkpoint,e):
        x,P,imu=checkpoint;x=x.copy();P=P.copy();t=e['header_ns']
        if t<x.t:raise ValueError('LATE')
        if (t-imu['header_ns'])*1e-9>self.predictor.c['max_imu_interval_s']:raise ValueError('IMU_GAP')
        if t>x.t:x,P=self.predictor.predict(x,P,imu['omega'],imu['force'],(t-x.t)*1e-9);x.t=t
        if e['kind']=='imu':imu=dict(header_ns=t,omega=np.array(e['omega']),force=np.array(e['force']))
        else:x,P,_=self.wheel.correct(x,P,imu['omega'],e['speed'],e['variance'])
        return x,P,imu
    def ingest(self,e):
        k=self.key(e)
        if k in self.seen:return 'DUPLICATE'
        if e['header_ns']<=self.base[0].t:return 'OUTSIDE_CHECKPOINT_HISTORY'
        if e['header_ns']-self.current()[0].t>self.horizon_ns:raise ValueError('INPUT_OUTSIDE_FORWARD_HORIZON')
        if len(self.events)>=10000:raise ValueError('BUFFER_CAPACITY')
        i=bisect_right(self.keys,k);new=self.events[:i]+[e]+self.events[i:];start=min(i,len(self.snapshots));snaps=self.snapshots[:start];checkpoint=self.base if start==0 else snaps[-1]
        # A late IMU can unblock a wheel message that arrived before the IMU.
        # Retain the pending suffix, but never publish an unpropagated state.
        deferred=False
        for item in new[start:]:
            try:checkpoint=self.step(checkpoint,item)
            except ValueError as err:
                if str(err)!='IMU_GAP':raise
                deferred=True;break
            snaps.append(checkpoint)
        self.replayed+=len(snaps)-start;self.events=new;self.snapshots=snaps;self.keys=[self.key(v) for v in new];self.seen.add(k);self.max_buffer=max(self.max_buffer,len(new));latest=self.snapshots[-1][0].t if self.snapshots else self.base[0].t
        while self.events and self.events[0]['header_ns']<latest-self.horizon_ns:
            self.base=self.snapshots.pop(0);self.events.pop(0);self.keys.pop(0)
        # Dedup identity storage is bounded with the same temporal horizon.
        self.seen={k for k in self.seen if k[0]>=self.base[0].t}
        return 'DEFERRED_IMU_GAP' if deferred else 'REPROPAGATED' if i<len(new)-1 else 'IN_ORDER'
    def current(self):
        x,P,imu=self.snapshots[-1] if self.snapshots else self.base
        return x.copy(),P.copy(),imu
