"""Bounded single-request JSON bridge to real original CUDA NDT local session."""
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import time
import numpy as np
from local_odometry import LocalOdometry, Config, Scan, pose


def write_cloud(path, p):
    p = np.ascontiguousarray(p, dtype='<f4')
    with open(path, 'wb') as f:
        f.write(struct.pack('<Q', len(p)))
        f.write(p.tobytes())


class MatcherBridge:
    def __init__(self, executable, scratch):
        self.scratch=Path(scratch);self.scratch.mkdir(parents=True, exist_ok=True)
        self.process=subprocess.Popen([str(executable)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)

    def __call__(self, request):
        tick=time.perf_counter()
        a,b=self.scratch/'source.bin',self.scratch/'target.bin'
        write_cloud(a,request['source']);write_cloud(b,request['target'])
        payload={k:v for k,v in request.items() if k not in ('source','target','initial_guess')}
        payload.update(source_path=str(a),target_path=str(b),initial_guess=request['initial_guess'].tolist())
        self.process.stdin.write(json.dumps(payload)+'\n');self.process.stdin.flush()
        line=self.process.stdout.readline()
        if not line:
            raise RuntimeError('local NDT service terminated; stderr='+self.process.stderr.read(4096))
        result=json.loads(line);result['bridge_total_ms']=(time.perf_counter()-tick)*1000
        return result

    def close(self):
        self.process.stdin.close()
        try:self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:self.process.terminate();self.process.wait(timeout=5)
        self.process.stdout.close();self.process.stderr.close()
        for name in ['source.bin','target.bin']:
            (self.scratch/name).unlink(missing_ok=True)


class MultiresMatcher:
    """Explicit B-K-MR comparator; default B-K is untouched.

    Fine failure remains a failure: coarse output is never returned as the pose.
    The fine proposal is validated against the ORIGINAL local prediction by the
    outer validator, rather than measuring its innovation against coarse output.
    """
    def __init__(self, bridge):self.bridge=bridge

    def __call__(self, request):
        tick=time.perf_counter()
        coarse_request=dict(request,comparison='B-K-MR',resolution=3.0)
        coarse=self.bridge(coarse_request)
        fine_request=dict(request,comparison='B-K-MR',resolution=1.5)
        if coarse.get('candidate_valid') and coarse.get('converged'):
            try:fine_request['initial_guess']=pose(coarse['O_T_base'])
            except (ValueError,KeyError):pass
        fine=self.bridge(fine_request)
        fine['multiresolution']=dict(levels=[3.0,1.5],coarse_proposal=coarse,coarse_used_as_seed=not np.array_equal(fine_request['initial_guess'],request['initial_guess']),total_bridge_ms=(time.perf_counter()-tick)*1000,total_target_build_ms=coarse.get('target_build_ms',0)+fine.get('target_build_ms',0),total_align_ms=coarse.get('align_wall_ms',0)+fine.get('align_wall_ms',0),fine_failure_falls_back_to_prediction=True)
        return fine

    def close(self):self.bridge.close()


class Validator:
    def __init__(self, path=None):
        path=Path(path) if path else Path(__file__).with_name('validator_v1.json')
        self.config=json.loads(path.read_text());self.sha256=hashlib.sha256(path.read_bytes()).hexdigest()
        self.dt_s=None

    def __call__(self, request, result):
        c=self.config;reasons=[]
        for key in ('candidate_valid','converged'):
            if not result.get(key,False):reasons.append(key)
        if result.get('target_valid_voxels',0)<c['minimum_target_valid_voxels']:reasons.append('target_voxels')
        count=result.get('neighbor_points',0);n=len(request['source'])
        if count<c['minimum_distinct_neighbor_source_points'] or count/max(1,n)<c['minimum_distinct_neighbor_source_fraction']:reasons.append('distinct_source_support')
        if count>n:reasons.append('invalid_distinct_source_count')
        if self.dt_s is None or self.dt_s<=0:raise ValueError('validator requires true positive scan reference dt')
        correction=np.linalg.inv(request['initial_guess'])@pose(result['O_T_base'])
        translation=float(np.linalg.norm(correction[:3,3]))
        rotation=float(np.rad2deg(np.arccos(np.clip((np.trace(correction[:3,:3])-1)/2,-1,1))))
        scale=min(c['maximum_dt_scale'],self.dt_s/.05)
        if translation>c['translation_innovation_at_50ms_m']*scale:reasons.append('translation_innovation')
        if rotation>c['rotation_innovation_at_50ms_deg']*scale:reasons.append('rotation_innovation')
        result['validation']=dict(rule_sha256=self.sha256,reasons=reasons,translation_innovation_m=translation,rotation_innovation_deg=rotation,actual_dt_s=self.dt_s,dt_scale=scale,conditional_engineering_rule=True)
        return not reasons


class ShadowGroup:
    """Called synchronously before dense scratch is overwritten by next frame.

    Each variant owns its history, O pose and matcher session. U is shared pure
    motion input; a B posterior cannot enter another variant or global matcher.
    """
    def __init__(self, executable, scratch, modes=('B-P','B0','B-M','B-K')):
        self.chains={mode:LocalOdometry(Config(mode=mode)) for mode in modes}
        self.matchers={mode:(MultiresMatcher(MatcherBridge(executable,Path(scratch)/mode)) if mode=='B-K-MR' else MatcherBridge(executable,Path(scratch)/mode)) for mode in modes if mode!='B-P'}
        self.validators={mode:Validator() for mode in modes}
        self.last_ref=None

    def on_frame(self, *, frame_id, ref_ns, available_ns, dense, source, U_T_base, deskew_history, point_ids=None):
        tick=time.perf_counter()
        scan=Scan.create(frame_id,ref_ns,available_ns,dense,source,deskew_history,point_ids)
        input_prepare_ms=(time.perf_counter()-tick)*1000
        rows={}
        for mode,chain in self.chains.items():
            start=time.perf_counter();v=self.validators[mode]
            v.dt_s=None if self.last_ref is None else (ref_ns-self.last_ref)*1e-9
            row=chain.step(scan,U_T_base,self.matchers.get(mode),v)
            row['full_step_ms']=(time.perf_counter()-start)*1000
            row['validator_sha256']=v.sha256
            row['shared_input_prepare_ms']=input_prepare_ms
            if len(self.chains)==1:row['full_callback_variant_ms']=row['full_step_ms']+input_prepare_ms
            rows[mode]=row
        self.last_ref=ref_ns
        return dict(frame_id=str(frame_id),ref_ns=ref_ns,available_ns=available_ns,source_identity=scan.source_identity,dense_identity=scan.dense_identity,deskew_history=deskew_history,variants=rows,input_prepare_ms=input_prepare_ms,all_variants_ms=(time.perf_counter()-tick)*1000)

    def close(self):
        for m in self.matchers.values():m.close()
