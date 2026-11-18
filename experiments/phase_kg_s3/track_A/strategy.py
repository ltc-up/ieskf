"""KG-S3 strategies for the coordinator's causal continuous replay runner.

No current-posterior state or mutable active_scratch is retained. Every callback
owns a versioned immutable source artifact and target/ROI recipe; it resubmits
that complete recipe at every evaluation, including historical relinearization.
"""
import copy, threading, time
from collections import OrderedDict
import numpy as np
from score_map import ScoreMapObserver, DirectionalObserver, prior_identity
from geometry_ieskf.external_pose import ExternalPose
from geometry_ieskf.ndt_pose_observation import engineering_R

class SerializedService:
    def __init__(self,service):self.service=service;self.lock=threading.RLock()
    def call(self,request):
        with self.lock:
            answer=self.service.call(request)
            return answer[0] if isinstance(answer,tuple) else answer

class BoundScore:
    """Immutable evaluation recipe with bounded, exact-pose memoization.

    Memoization changes no information use: the identical point cloud, Target,
    ROI, model and q produce identical geometry. Replay's prior still rebuilds
    its full objective and solves anew, never reuses a prior-fused posterior.
    """
    def __init__(self,service,recipe,corrected_pitch,spatial_blocks):
        self.service=service;self.recipe=copy.deepcopy(recipe)
        if not self.recipe.get('source_version'):raise ValueError('MISSING_SOURCE_VERSION')
        if not self.recipe.get('target_sha'):raise ValueError('MISSING_TARGET_VERSION')
        self.corrected_pitch=bool(corrected_pitch);self.spatial_blocks=bool(spatial_blocks)
        from pathlib import Path
        self.source_path=Path(self.recipe['source_path'])
        stat=self.source_path.stat();self.source_stat=(stat.st_size,stat.st_mtime_ns)
        version=self.recipe['source_version']
        if len(version)==64:
            import hashlib
            if hashlib.sha256(self.source_path.read_bytes()).hexdigest()!=version:raise ValueError('SOURCE_ARTIFACT_SHA_MISMATCH')
        self.cache=OrderedDict();self.cache_hits=0;self.calls=0;self.wall_ms=0
    def __deepcopy__(self,memo):return self  # immutable version-bound service handle
    def __call__(self,q):
        stat=self.source_path.stat()
        if (stat.st_size,stat.st_mtime_ns)!=self.source_stat:raise ValueError('IMMUTABLE_SOURCE_ARTIFACT_CHANGED')
        q=np.asarray(q,dtype='<f8');key=q.tobytes()
        if key in self.cache:
            self.cache_hits+=1;self.cache.move_to_end(key);return copy.deepcopy(self.cache[key])
        begin=time.monotonic_ns()
        result=self.service.call(self.recipe|dict(op='score',pose_q=q.tolist(),corrected_pitch=self.corrected_pitch,spatial_blocks=self.spatial_blocks))
        self.wall_ms+=(time.monotonic_ns()-begin)/1e6;self.calls+=1
        if result.get('target_sha')!=self.recipe['target_sha'] or result.get('source_version')!=self.recipe['source_version']:raise ValueError('GEOMETRY_VERSION_MISMATCH')
        self.cache[key]=copy.deepcopy(result)
        if len(self.cache)>64:self.cache.popitem(last=False)
        return result

class LegacyObserver(ExternalPose):
    def correct(self,x,P,T,R,projection=None):
        old=engineering_R(float(R))
        matrix=np.diag([old[0,0]]*3+[old[2,2]]*3)
        return super().correct(x,P,T,matrix,projection)

def make_observer(strategy,lever,service,immutable_recipe,worker_prior=None):
    """Return (observer, R_payload). Use one event/correlation_group per scan.

    A0 expects the worker's scalar fitness as its event R payload. A1/A2 ignore
    that payload and never map fitness to their new geometric information.
    Source artifacts must be retained while the event is in 500ms history.
    """
    if strategy=='A0':return LegacyObserver(lever,('x','y','z','rx','ry','rz')),None
    if strategy not in ('A1','A2'):raise ValueError('UNKNOWN_STRATEGY')
    evaluator=BoundScore(service,immutable_recipe,corrected_pitch=strategy=='A2',spatial_blocks=strategy=='A1')
    source=immutable_recipe['source_version'];target=immutable_recipe['target_sha']+':'+str(immutable_recipe['roi'][-1])
    if strategy=='A1':return DirectionalObserver(lever,evaluator,source,target),None
    worker_hash=None if worker_prior is None else prior_identity(*worker_prior)
    return ScoreMapObserver(lever,evaluator,source,target,worker_hash),None

def align_admission(result):
    if not result.get('converged',False):return False,'ALIGN_NOT_CONVERGED'
    if not result.get('candidate_valid',False):return False,'ALIGN_CANDIDATE_INVALID'
    try:
        T=np.asarray(result['final'],float)
        if T.shape!=(4,4) or not np.isfinite(T).all():raise ValueError()
        if not np.allclose(T[3],[0,0,0,1],atol=1e-5,rtol=0):raise ValueError()
        if not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-4,rtol=0):raise ValueError()
        if abs(np.linalg.det(T[:3,:3])-1)>1e-4:raise ValueError()
    except (ValueError,KeyError):return False,'ALIGN_NONFINITE_OR_INVALID_SE3'
    return True,None

class RejectedAlignObserver:
    def __init__(self,detail):self.detail=detail
    def correct(self,x,P,T,R=None,projection=None):
        return x.copy(),P.copy(),dict(accepted=False,reason='A1_MATCHER_INVALID',detail=self.detail,delta=np.zeros(15).tolist(),covariance_updates=0,covariance_changed=False,covariance_calibrated=False,fallback='IMU_WHEEL_PREDICTION_ONLY')

class ReplayStrategyFactory:
    def __init__(self,config,paths,out,lever):
        import subprocess
        from pathlib import Path
        from score_map import Contract
        self.strategy=config['strategy'];self.paths=copy.deepcopy(paths);self.lever=np.asarray(lever)
        if self.strategy=='A2':
            if not config.get('a2_beta_frozen',False):raise ValueError('A2_BETA_NOT_APPROVED_FROZEN')
            if 'a2_beta' not in config:raise ValueError('EXPLICIT_A2_BETA_REQUIRED')
            if float(config['a2_beta'])!=1.0:raise ValueError('A2_BETA_MUST_BE_FROZEN_ONE')
            self.contract=Contract(beta=1.0)
        if self.strategy=='A0':self.proc=None;return
        binary=paths.get('score_service')
        if not binary:raise ValueError('SCORE_SERVICE_PATH_REQUIRED')
        Path(out).mkdir(parents=True,exist_ok=True)
        self.stderr=(Path(out)/'score_service.stderr').open('w')
        self.proc=subprocess.Popen([binary],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.stderr,text=True)
        self.lock=threading.RLock()
    def call(self,request):
        import json
        with self.lock:
            self.proc.stdin.write(json.dumps(request)+'\n');self.proc.stdin.flush();line=self.proc.stdout.readline()
            if not line:raise RuntimeError('SCORE_SERVICE_EOF')
            return json.loads(line)
    def create_observer(self,source_path,source_version,target_version,roi,worker_prior_hash,align_result):
        if self.strategy=='A0':return LegacyObserver(self.lever,('x','y','z','rx','ry','rz'))
        available,invalid_reason=align_admission(align_result)
        if self.strategy=='A1' and not available:return RejectedAlignObserver(invalid_reason)
        recipe=dict(source_path=str(source_path),source_version=source_version,target_path=self.paths['target'],target_sha=target_version,roi=copy.deepcopy(roi))
        evaluator=BoundScore(self,recipe,corrected_pitch=self.strategy=='A2',spatial_blocks=self.strategy=='A1')
        fulltarget=target_version+':'+str(roi[-1])
        if self.strategy=='A1':return DirectionalObserver(self.lever,evaluator,source_version,fulltarget)
        return ScoreMapObserver(self.lever,evaluator,source_version,fulltarget,worker_prior_hash,self.contract,align_valid=available,align_failure=invalid_reason)
    def close(self):
        if self.proc is None:return
        self.proc.stdin.close();self.proc.wait(timeout=30);self.stderr.close()

def factory(config,paths,out,lever):
    return ReplayStrategyFactory(config,paths,out,lever)
