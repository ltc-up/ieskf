"""Research Score-MAP mean and separately labelled conditional covariance.

q is the production [t, RxRyRz] chart; z is the frozen prior's 15D right-error
chart [p,v,theta,bg,ba]. The prior is quadratic exactly in z. Score callbacks
must evaluate the actual CUDA multi-neighbor Score and its approved derivatives.
Synthetic callbacks are permitted only for numerical unit tests.
"""
from dataclasses import dataclass, asdict
import hashlib, json, struct
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_ieskf.state import inject, difference, chart_map
from geometry_ieskf.common import C
from geometry_ieskf.posterior import valid_prior
from geometry_ieskf.external_pose import ExternalPose

POSE = np.array([0,1,2,6,7,8])

@dataclass(frozen=True)
class Contract:
    beta: float = 1.0
    iterations: int = 3
    step_cap: float = .2
    step_min: float = .005
    gll_window: int = 3
    armijo: float = 1e-4
    max_evaluations: int = 10
    position_condition_scale_m: float = .5
    rotation_condition_scale_rad: float = np.pi/60
    curvature_difference_step: float = 1e-4
    calibrated: bool = False
    snap_guard_version: str = "continuous-euler-rate-bound-v1"
    snap_threshold_rad: float = 1e-4
    path_subdivision_budget: int = 128

    @property
    def sha256(self):
        return hashlib.sha256(json.dumps(asdict(self),sort_keys=True).encode()).hexdigest()

CONTRACT = Contract()

def prior_identity(x,P):
    values=np.r_[x.p,x.v,x.R.ravel(),x.bg,x.ba,np.asarray(P).ravel()]
    return hashlib.sha256(values.astype('<f8').tobytes()+struct.pack('<q',int(x.t))).hexdigest()

def state_q(x,lever):
    return np.r_[x.p-x.R@lever,Rotation.from_matrix(x.R).as_euler('XYZ')]

def unwrap_q(q,anchor):
    q=q.copy();q[3:]=anchor[3:]+(q[3:]-anchor[3:]+np.pi)%(2*np.pi)-np.pi
    return q

def pose_chart(prior,z,lever,h=1e-4):
    """Only chart derivatives are finite differenced; never the Score model.

    Includes q'' contracted with Score g in the full objective Hessian.
    q uses the exact IMU-to-base lever arm and production RxRyRz convention.
    """
    q=state_q(inject(prior,z),lever);J=np.zeros((6,15));Q=np.zeros((6,15,15))
    def f(delta):return unwrap_q(state_q(inject(prior,z+delta),lever),q)
    for i in POSE:
        ei=np.eye(15)[i]*h
        plus,minus=f(ei),f(-ei);J[:,i]=(plus-minus)/(2*h)
        Q[:,i,i]=(plus-2*q+minus)/h**2
        for j in POSE[POSE>i]:
            ej=np.eye(15)[j]*h
            value=(f(ei+ej)-f(ei-ej)-f(-ei+ej)+f(-ei-ej))/(4*h*h)
            Q[:,i,j]=Q[:,j,i]=value
    return q,J,Q

def conditional_information(Hq,contract=CONTRACT):
    """Frozen engineering information; NOT a calibrated Hessian inverse.

    Normalize positions/angles by declared conditional uncertainty scales,
    retain positive curvature, cap dimensionless directional information at 1.
    Negative/zero curvature directions give exactly zero information.
    No LM damping or prior Hessian is used to manufacture observation info.
    """
    D=np.diag([contract.position_condition_scale_m]*3+[contract.rotation_condition_scale_rad]*3)
    curvature=-contract.beta*(np.asarray(Hq)+np.asarray(Hq).T)/2
    w,U=np.linalg.eigh(D@curvature@D)
    kept=np.minimum(np.maximum(w,0),1)
    invD=np.diag(1/np.diag(D))
    info=invD@(U*kept)@U.T@invD
    return info,dict(raw_normalized_eigenvalues=w.tolist(),retained_eigenvalues=kept.tolist(),covariance_calibrated=False,model='positive-curvature-capped-conditional-v1')

SNAP_EPSILON = 1e-4
AXES = ('roll','pitch','yaw')

class NumericalDomainInvalid(ValueError):
    def __init__(self,details):
        self.details=details
        super().__init__('NUMERICAL_DOMAIN_INVALID')

def score_domain(q):
    # Boundary itself is conservatively invalid: original Score is
    # discontinuous across the snap edge, so a two-sided derivative is absent.
    return [AXES[k] for k in range(3) if abs(float(q[3+k]))<=SNAP_EPSILON]

def certify_snap_path(prior,z0,z1,lever,max_segments=128):
    """Conservative continuous-path proof, not endpoint-only sampling.

    For a straight frozen-prior rotation-vector segment, ||omega|| <= ||dz||
    since the SO(3) right-Jacobian operator norm is at most one. XYZ Euler
    pitch rate is <= ||omega|| and roll/yaw rates <= sec(pitch)||omega||.
    Therefore pitch stays within |pitch_start|+||dz|| and the corresponding
    secant bound certifies the entire segment. Inconclusive segments subdivide;
    reaching the explicit work budget rejects rather than accepting an
    uncertified path. This also catches same-sign endpoints with an interior
    excursion. No Score/gradient is evaluated along an uncertified path.
    """
    queue=[(z0.copy(),z1.copy())];segments=0
    while queue:
        a,b=queue.pop();qa=state_q(inject(prior,a),lever);qb=state_q(inject(prior,b),lever)
        segments+=1
        bad=sorted(set(score_domain(qa)+score_domain(qb)))
        if bad:return dict(valid=False,reason='SNAP_ENDPOINT_OR_INTERIOR',axes=bad,segments=segments,start_q=qa.tolist(),end_q=qb.tolist())
        radius=float(np.linalg.norm((b-a)[6:9]));pitch_upper=abs(float(qa[4]))+radius
        if pitch_upper<np.pi/2:
            angular_bound=np.array([radius/np.cos(pitch_upper),radius,radius/np.cos(pitch_upper)])
            uncertain=np.abs(qa[3:])-angular_bound<=SNAP_EPSILON
            if not uncertain.any():continue
        else:uncertain=np.ones(3,dtype=bool)
        if segments>=max_segments:
            return dict(valid=False,reason='SNAP_PATH_NOT_CERTIFIED_WITHIN_BUDGET',axes=[AXES[k] for k in np.flatnonzero(uncertain)],segments=segments,start_q=qa.tolist(),end_q=qb.tolist())
        mid=(a+b)/2
        queue.append((mid,b));queue.append((a,mid))
    return dict(valid=True,segments=segments,axes=[])

class FrozenMap:
    def __init__(self,prior,P,lever,evaluate,contract=CONTRACT):
        valid_prior(P);self.prior=prior.copy();self.P=np.array(P,copy=True)
        self.invP=np.linalg.inv(P);self.lever=np.asarray(lever);self.evaluate=evaluate
        self.contract=contract;self.prior_hash=prior_identity(prior,P);self.calls=0;self.domain_rejections=[]
    def value(self,z,derivatives=True):
        if np.linalg.norm(z[6:9])>=np.pi-1e-3:raise ValueError('PRIOR_CHART_DOMAIN')
        q0=state_q(inject(self.prior,z),self.lever)
        bad=score_domain(q0)
        if bad:
            info=dict(kind='candidate_linearization',axes=bad,q=q0.tolist(),reason='SNAP_LINEARIZATION_POINT')
            self.domain_rejections.append(info);raise NumericalDomainInvalid(info)
        q,J,Q=pose_chart(self.prior,z,self.lever,self.contract.curvature_difference_step)
        result=self.evaluate(q);self.calls+=1
        S=float(result['score']);gq=np.asarray(result['gradient']);Hq=np.asarray(result['hessian'])
        if not np.isfinite(np.r_[S,gq,Hq.ravel()]).all() or result.get('valid_pairs',1)<=0:raise ValueError('INVALID_SCORE')
        value=float(.5*z@self.invP@z-self.contract.beta*S)
        g=self.invP@z-self.contract.beta*J.T@gq
        H=self.invP-self.contract.beta*(J.T@Hq@J+np.einsum('k,kij->ij',gq,Q))
        prior_gradient=self.invP@z;geometric_gradient=-self.contract.beta*J.T@gq
        product=float(np.linalg.norm(prior_gradient)*np.linalg.norm(geometric_gradient))
        angle=None if product==0 else float(np.arccos(np.clip(prior_gradient@geometric_gradient/product,-1,1)))
        return dict(z=z.copy(),q=q,F=value,g=g,H=(H+H.T)/2,score=S,valid_pairs=int(result.get('valid_pairs',0)),prior_quadratic=float(z@self.invP@z),prior_contribution=float(.5*z@self.invP@z),geometry_contribution=float(-self.contract.beta*S),prior_gradient=prior_gradient.copy(),geometry_gradient=geometric_gradient.copy(),prior_gradient_norm=float(np.linalg.norm(prior_gradient)),geometry_gradient_norm=float(np.linalg.norm(geometric_gradient)),gradient_angle_rad=angle,Hq=Hq,J=J,original_score_hessian=np.asarray(result.get('original',{}).get('hessian',Hq)),approved_score_hessian=Hq.copy())
    def refine(self,start,label):
        c=self.contract;v=self.value(start.copy());history=[v['F']];trace=[];status='ITERATION_LIMIT'
        for iteration in range(c.iterations):
            # Exact/approved Hessian direction; SVD and sign flip follow raw
            # align's approach. No damping information enters covariance.
            raw=np.linalg.lstsq(v['H'],-v['g'],rcond=1e-12)[0]
            norm=np.linalg.norm(v['J']@raw)
            if not np.isfinite(norm):raise ValueError('NONFINITE_MAP_STEP')
            if norm<1e-10:status='STATIONARY';break
            direction=raw/norm;slope=float(v['g']@direction)
            if slope>0:direction=-direction;slope=-slope
            if slope>=0:status='NON_DESCENT';break
            step=max(c.step_min,min(norm,c.step_cap));accepted=False
            for trial in range(c.max_evaluations):
                trial_z=v['z']+step*direction
                domain=certify_snap_path(self.prior,v['z'],trial_z,self.lever,c.path_subdivision_budget)
                if not domain['valid']:
                    self.domain_rejections.append(domain|dict(kind='line_search_path',iteration=iteration,trial=trial))
                    trace.append(dict(iteration=iteration,trial=trial,step=step,accepted=False,reason='NUMERICAL_DOMAIN_INVALID',domain=domain))
                    if step<=c.step_min:break
                    step=max(c.step_min,.5*step);continue
                candidate=self.value(trial_z)
                bound=max(history[-c.gll_window:])+c.armijo*step*slope
                actual_step=float(np.linalg.norm(unwrap_q(candidate['q'],v['q'])-v['q']))
                accepted=candidate['F']<=bound and actual_step<=c.step_cap+1e-10
                trace.append(dict(iteration=iteration,trial=trial,step=step,total_objective=candidate['F'],bound=bound,accepted=accepted,score=candidate['score'],actual_pose_step=actual_step))
                if accepted:break
                if step<=c.step_min:break
                step=max(c.step_min,.5*step)
            if not accepted:status='TOTAL_MAP_LINE_SEARCH_FAILED';break
            oldF=v['F'];v=candidate;history.append(v['F'])
            if actual_step<.01:status='STEP_EPSILON';break
            if len(history)>=3 and all(abs(history[k]-history[k-1])/max(1,abs(history[k-1]))<.001 for k in [-1,-2]):status='OBJECTIVE_EPSILON';break
        v.update(label=label,status=status,trace=trace)
        return v
    def align_start(self,T):
        candidate=self.prior.copy();candidate.R=np.asarray(T)[:3,:3].copy()
        candidate.p=np.asarray(T)[:3,3]+candidate.R@self.lever
        z=difference(candidate,self.prior);pose_delta=z[POSE]
        # Minimize the frozen complete prior over unconstrained v and biases
        # conditional on the Align pose; no NDT observation is fused here.
        return self.P[:,POSE]@np.linalg.solve(self.P[np.ix_(POSE,POSE)],pose_delta)
    def solve(self,T,align_valid=True,align_failure=None):
        zero=np.zeros(15);align=None;align_error=align_failure or 'A2_ALIGN_UNAVAILABLE'
        if align_valid:
            try:align=self.align_start(T)
            except (ValueError,np.linalg.LinAlgError,IndexError) as exc:align_error='A2_ALIGN_INVALID_SE3:'+str(exc)
        # Preserve the four required candidates, then use one identical F.
        candidates=[]
        for label,start,refine in [('prior',zero,False),('align',align,False),('prior_refined',zero,True),('align_refined',align,True)]:
            try:
                if start is None:raise ValueError(align_error)
                value=self.refine(start,label) if refine else self.value(start)
                value['label']=label
            except (ValueError,np.linalg.LinAlgError) as exc:
                value=dict(label=label,z=None if start is None else start.copy(),status='CANDIDATE_FAILED',failure=str(exc),F=None,domain=getattr(exc,'details',None))
            candidates.append(value)
        legal=[]
        for v in candidates:
            d=v['z'];reason=v.get('failure')
            if d is None:
                v['domain_rejection']=reason;continue
            if np.linalg.norm(d[6:9])>C['max_total_rotation_rad']:reason='ATTITUDE_LINEARIZATION_DOMAIN'
            if np.linalg.norm(d[9:12])>C['max_bias_gyro_change_rad_s']:reason='BIAS_CHANGE_DOMAIN'
            if np.linalg.norm(d[12:15])>C['max_bias_accel_change_m_s2']:reason='BIAS_CHANGE_DOMAIN'
            v['domain_rejection']=reason
            if reason is None:legal.append(v)
        if not legal:
            if self.domain_rejections:raise NumericalDomainInvalid(dict(reason='NO_VALID_MAP_CANDIDATE',domain_rejections=self.domain_rejections,candidates=candidates))
            raise ValueError('NO_VALID_MAP_CANDIDATE')
        selected=min(legal,key=lambda v:v['F'])
        I,metadata=conditional_information(selected['Hq'],self.contract)
        Qz=np.linalg.inv(self.invP+selected['J'].T@I@selected['J'])
        B=chart_map(selected['z']);Q=B@Qz@B.T;Q=(Q+Q.T)/2;valid_prior(Q)
        x=inject(self.prior,selected['z'])
        return x,Q,dict(accepted=True,reason='CONDITIONAL_SCORE_MAP',prior_hash=self.prior_hash,contract_sha256=self.contract.sha256,selected=selected['label'],delta=selected['z'].tolist(),prior_delta=zero.tolist(),original_ndt_delta=None if align is None else align.tolist(),domain_rejections=self.domain_rejections,domain_rejection_count=len(self.domain_rejections),beta=self.contract.beta,selected_metrics={k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in selected.items() if k in ('score','valid_pairs','prior_quadratic','prior_contribution','geometry_contribution','prior_gradient','geometry_gradient','prior_gradient_norm','geometry_gradient_norm','gradient_angle_rad','F')},score_calls=self.calls,candidates=[{k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in z.items() if k not in ('Hq','J','H','g')} for z in candidates],covariance_updates=1,**metadata)

class ScoreMapObserver:
    """PoseDelayReplay-compatible callback; actual checkpoint supplies prior.

    factory(q) must bind immutable Source, exact Target version and derivative
    version; it must never use the current matcher Target implicitly. Replay
    invokes solve again from that event's actual pre-observation checkpoint.
    """
    def __init__(self,lever,evaluate,source_version,target_version,worker_prior_hash=None,contract=CONTRACT,align_valid=True,align_failure=None):
        self.lever=np.asarray(lever);self.evaluate=evaluate;self.source_version=source_version
        self.target_version=target_version;self.worker_prior_hash=worker_prior_hash;self.contract=contract;self.align_valid=align_valid;self.align_failure=align_failure
    def __deepcopy__(self,memo):
        # Evaluator is a version-bound immutable handle, not mutable matcher
        # state. Deep-copying a service would duplicate locks and GPU ownership.
        return type(self)(self.lever.copy(),self.evaluate,self.source_version,self.target_version,self.worker_prior_hash,self.contract,self.align_valid,self.align_failure)
    def correct(self,x,P,T,R=None,projection=None):
        actual=prior_identity(x,P)
        try:
            y,Q,info=FrozenMap(x,P,self.lever,self.evaluate,self.contract).solve(T,self.align_valid,self.align_failure)
        except (ValueError,np.linalg.LinAlgError) as exc:
            return x.copy(),P.copy(),dict(accepted=False,reason=str(exc),prior_hash=actual,domain=getattr(exc,'details',None),covariance_updates=0,covariance_calibrated=False,delta=np.zeros(15).tolist(),covariance_changed=False,fallback='IMU_WHEEL_PREDICTION_ONLY')
        info.update(source_version=self.source_version,target_version=self.target_version,worker_prior_changed=self.worker_prior_hash is not None and actual!=self.worker_prior_hash)
        return y,Q,info

def q_transform(q):
    T=np.eye(4);T[:3,:3]=Rotation.from_euler('XYZ',q[3:]).as_matrix();T[:3,3]=q[:3]
    return T

def sandwich_information(Hq,block_gradients,contract=CONTRACT):
    """Spatial-cluster estimating-equation covariance, capped conservatively.

    Neither this covariance nor the fixed cap has independent statistical
    calibration. Blocks must be supplied by the same Score at the same pose;
    multiple Gaussian neighbors of one source point stay in the same block.
    """
    G=np.asarray(block_gradients,float)
    if G.ndim!=2 or G.shape[1]!=6 or len(G)<7:raise ValueError('INSUFFICIENT_SPATIAL_BLOCKS')
    D=np.diag([contract.position_condition_scale_m]*3+[contract.rotation_condition_scale_rad]*3)
    Cq=-(np.asarray(Hq)+np.asarray(Hq).T)/2
    w,U=np.linalg.eigh(D@Cq@D);keep=w>max(1e-9,float(abs(w).max())*1e-8)
    if not keep.any():raise ValueError('NO_POSITIVE_GEOMETRIC_SUBSPACE')
    V=U[:,keep];curvature=w[keep]
    # Center spatial cluster scores; this remains a misspecified-estimator
    # diagnostic where association discontinuity or bias violates assumptions.
    scores=G@D@V;scores-=scores.mean(axis=0)
    meat=scores.T@scores*len(G)/(len(G)-1)
    cov=(meat/curvature[:,None])/curvature[None,:]
    ew,EV=np.linalg.eigh((cov+cov.T)/2)
    # Dimensionless variance floor 1 is an explicit engineering information
    # ceiling; no unobservable direction receives a covariance pseudo-inverse.
    subinfo=(EV*(1/np.maximum(ew,1)))@EV.T
    invD=np.diag(1/np.diag(D));info=invD@V@subinfo@V.T@invD
    return info,dict(model='spatial-score-sandwich-capped-conditional-v1',spatial_blocks=len(G),meat_rank=int(np.linalg.matrix_rank(meat)),curvature_eigenvalues=w.tolist(),sandwich_variance_eigenvalues=ew.tolist(),floor_variance_eigenvalues=np.maximum(ew,1).tolist(),R_q=(D@V@(EV*np.maximum(ew,1))@EV.T@V.T@D).tolist(),R_q_active_basis=(D@V).tolist(),R_q_semantics='observed-subspace embedding; null zeros mean absent information',covariance_calibrated=False)

class DirectionalObserver:
    """A1 one pose update; geometry contains no prior information.

    evaluator(q) must return raw original Score H plus spatial block gradients.
    Its final Align pose is never separately admitted through a second update.
    """
    def __init__(self,lever,evaluate,source_version,target_version):
        self.lever=np.asarray(lever);self.evaluate=evaluate
        self.source_version=source_version;self.target_version=target_version
    def __deepcopy__(self,memo):
        return type(self)(self.lever.copy(),self.evaluate,self.source_version,self.target_version)
    def correct(self,x,P,T,R=None,projection=None):
        try:
            T=np.asarray(T);q=np.r_[T[:3,3],Rotation.from_matrix(T[:3,:3]).as_euler('XYZ')]
            geom=self.evaluate(q);Iq,meta=sandwich_information(geom['hessian'],geom['block_gradients'])
            # Map Euler additive covariance into ExternalPose innovation chart
            # log(R_prior^T R_measurement), preserving all cross terms.
            L=np.zeros((6,6));h=1e-5
            def measurement(v):return np.r_[v[:3],Rotation.from_matrix(x.R.T@q_transform(v)[:3,:3]).as_rotvec()]
            for j in range(6):
                d=np.eye(6)[j]*h;L[:,j]=(measurement(q+d)-measurement(q-d))/(2*h)
            invL=np.linalg.inv(L);I=invL.T@Iq@invL;w,U=np.linalg.eigh((I+I.T)/2)
            keep=w>max(1e-10,float(w.max())*1e-9)
            if not keep.any():raise ValueError('NO_A1_INFORMATION')
            projection=U[:,keep].T
            observer=ExternalPose(self.lever,('x','y','z','rx','ry','rz'))
            # R in full innovation chart must be SPD for the existing backend;
            # discarded directions are removed by projection and thus cannot
            # contribute information. Unit variance there is algebraically unused.
            Rfull=(U*np.where(keep,1/np.maximum(w,1e-30),1))@U.T
            y,Q,info=observer.correct(x,P,T,Rfull,projection)
            info.update(meta,full_chart_R=Rfull.tolist(),projection=projection.tolist(),discarded_chart_variance_completion_unused=True,source_version=self.source_version,target_version=self.target_version,retained_rank=int(keep.sum()),prior_hash=prior_identity(x,P))
            return y,Q,info
        except (ValueError,np.linalg.LinAlgError,KeyError) as exc:
            return x.copy(),P.copy(),dict(accepted=False,reason=str(exc),covariance_calibrated=False)
