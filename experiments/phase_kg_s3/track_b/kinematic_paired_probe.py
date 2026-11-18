"""Read-only fixed-history 2x2 seed/ROI probe, separate from committed chains.

The original Legacy request/result remain unchanged. Diagnostic proposals never
enter O, U, target history or A0. Source/target arrays stay in memory only.
"""
import copy
import time
import numpy as np
from local_odometry import cloud_hash, pose
from bridge import Validator


class FixedHistoryPairProbe:
    def __init__(self, matcher, chain, predictor, emit):
        self.matcher=matcher;self.chain=chain;self.predictor=predictor;self.emit=emit;self.context=None;self.called=False
    def arm(self, scan, cases):
        self.context=(scan,list(cases));self.called=False
    def __call__(self, request):
        baseline=self.matcher(request)
        if self.context is None:return baseline
        self.called=True;scan,cases=self.context;tick=time.perf_counter()
        previous=self.chain.previous.scan.ref_ns
        interval=self.predictor.query(previous,scan.ref_ns,scan.available_ns,scan.available_ns)
        row=dict(kind='FIXED_HISTORY_SEED_ROI_2X2',cases=cases,frame_id=scan.frame_id,ref_ns=scan.ref_ns,prev_ref_ns=previous,query_available_ns=scan.available_ns,source_identity=scan.source_identity,interval=interval,diagnostic_only=True,feeds_online_state=False)
        if interval['status']!='VALID':
            self.emit(dict(row,status='KIN_SEED_UNAVAILABLE_AT_ORIGINAL_RELEASE',cells=[]));return baseline
        legacy_seed=pose(request['initial_guess']);kin_seed=self.chain.O@pose(interval['delta_T'])
        alt,_,identities=self.chain._target(scan,kin_seed)
        columns=(('LEGACY_ROI',request['target'],request['target_identity']),('KINEMATIC_ROI',alt,cloud_hash(alt)))
        cells=[];validator=Validator();validator.dt_s=(scan.ref_ns-previous)*1e-9
        for target_name,target,target_id in columns:
            for seed_name,seed in [('LEGACY_15D_DIFF',legacy_seed),('KINEMATIC_BK',kin_seed)]:
                req=dict(request,target=target,target_identity=target_id,initial_guess=seed.copy())
                if not len(target):
                    cells.append(dict(target=target_name,seed=seed_name,status='EMPTY_TARGET',target_identity=target_id));continue
                try:
                    result=copy.deepcopy(baseline) if target_name=='LEGACY_ROI' and seed_name=='LEGACY_15D_DIFF' else self.matcher(req)
                    accepted=validator(req,result)
                    cells.append(dict(target=target_name,seed=seed_name,initial_guess=seed.tolist(),target_identity=target_id,target_points=len(target),engineering_accepted=accepted,proposal=result))
                except Exception as exc:cells.append(dict(target=target_name,seed=seed_name,target_identity=target_id,status='PROBE_FAILURE',error=repr(exc)))
        self.emit(dict(row,status='EVALUATED',alternate_target_recipe=identities,cells=cells,extra_probe_wall_ms=(time.perf_counter()-tick)*1000))
        return baseline
    def end_frame(self):
        if self.context is not None and not self.called:
            scan,cases=self.context
            self.emit(dict(kind='FIXED_HISTORY_SEED_ROI_2X2',status='NO_LEGACY_MATCH_REQUEST',cases=cases,ref_ns=scan.ref_ns,frame_id=scan.frame_id,diagnostic_only=True))
        self.context=None
    def close(self):self.matcher.close()
