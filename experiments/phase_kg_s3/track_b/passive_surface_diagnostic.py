"""Passive real historical-base ray diagnostics; never changes matching output."""
import hashlib,time
import numpy as np
from glim_diagnostics import normals,multires_support,surface_mask
from local_odometry import transform
from kinematic_paired_probe import FixedHistoryPairProbe


def historical_base_diagnostic(source,target_O,candidate_O,previous_O,source_normals=None):
    n=normals(source) if source_normals is None else source_normals
    H=np.linalg.inv(np.asarray(previous_O));relative=H@np.asarray(candidate_O);target=transform(H,target_O)
    return dict(multires_support(source,n,target,relative),ray_origin='real immediately previous scan base',surface_pass_points=int(surface_mask(source,n,relative).sum()),historical_T_current=relative.tolist())


class PassiveMatcher:
    def __init__(self,matcher,anchor,emit,case_context):self.matcher=matcher;self.anchor=np.asarray(anchor).copy();self.emit=emit;self.context=case_context;self.cached_normals=None;self.cached_source=None
    def __call__(self,request):
        result=self.matcher(request)
        begin=time.perf_counter_ns();record=dict(kind='PASSIVE_SOURCE_FACING_SURFACE',diagnostic_only=True,feeds_online_state=False,context=self.context,source_identity=request.get('source_identity'),target_identity=request.get('target_identity'),previous_O_T_base=self.anchor.tolist())
        try:
            seed=np.ascontiguousarray(request['initial_guess'],dtype='<f8');record['seed_sha256']=hashlib.sha256(seed.tobytes()).hexdigest()
            if self.cached_source is not request['source']:
                self.cached_normals=normals(request['source']);self.cached_source=request['source']
            if 'O_T_base' not in result:raise ValueError('NO_CANDIDATE_POSE')
            record.update(status='EVALUATED',source_points=len(request['source']),target_points=len(request['target']),diagnostic=historical_base_diagnostic(request['source'],request['target'],result['O_T_base'],self.anchor,self.cached_normals))
        except Exception as exc:record.update(status='DIAGNOSTIC_ERROR',error=repr(exc))
        record['diagnostic_cpu_ms']=(time.perf_counter_ns()-begin)/1e6;self.emit(record)
        return result


def probe_class(emit):
    class PassiveSurfacePairProbe(FixedHistoryPairProbe):
        def __call__(self,request):
            if self.context is None:return super().__call__(request)
            scan,cases=self.context;original=self.matcher
            context=dict(frame_id=scan.frame_id,ref_ns=scan.ref_ns,cases=cases,previous_ref_ns=self.chain.previous.scan.ref_ns)
            self.matcher=PassiveMatcher(original,self.chain.previous.O_T_base,emit,context)
            try:return super().__call__(request)
            finally:self.matcher=original
    return PassiveSurfacePairProbe


class SurfaceStatus:
    """Separate sidecar completion/quality evidence, independent of runner status."""
    def __init__(self):self.counts={};self.cpu_ms=0.
    def record(self,row):
        status=row['status'];self.counts[status]=self.counts.get(status,0)+1;self.cpu_ms+=float(row.get('diagnostic_cpu_ms',0.))
    def summary(self,created,closed,written,error=None):
        attempted=sum(self.counts.values())
        return dict(variant='HISTORICAL_BASE_SOURCE_FACING_PASSIVE',sidecar_created=bool(created),sidecar_close_succeeded=bool(closed),status_counts=self.counts.copy(),diagnostic_total_cpu_ms=self.cpu_ms,attempted_records=attempted,written_records=int(written),sidecar_complete=bool(created and closed and written==attempted),diagnostic_errors=self.counts.get('DIAGNOSTIC_ERROR',0),error=error,runner_COMPLETED_does_not_certify_this_sidecar=True)
