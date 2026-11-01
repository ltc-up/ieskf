"""Diagnostic update eligibility, without clipping or changing the historical solver."""
import numpy as np

def pair_retention(a,b):
    """Actual source-to-target pairs; target-set reuse alone is not stability."""
    A=set(zip(map(int,a['source_index']),map(int,a['target_ids'])))
    B=set(zip(map(int,b['source_index']),map(int,b['target_ids'])))
    return {'pair_jaccard':len(A&B)/max(1,len(A|B)),
            'retained_from_previous':len(A&B)/max(1,len(A)),
            'identical':A==B,'entered':len(B-A),'exited':len(A-B)}

def eligible(metrics,tolerance):
    """Reuse the frozen numerical tolerance, plus an exact local association witness.

    This is a conservative numerical qualification, not a calibrated probability test.
    Both the untruncated prior-white GN step and the GN-metric dual gradient
    (Newton decrement) must meet the EXISTING tolerance. No fitted cutoffs.
    """
    values=[metrics.get('gn_white_norm',float('inf')),
            metrics.get('newton_decrement',float('inf'))]
    finite=bool(np.isfinite(values).all())
    local=finite and max(values)<=tolerance
    reasons=[]
    if not finite:reasons.append('NUMERIC_FAILURE')
    if not local:reasons.append('LOCAL_NOT_CONVERGED')
    if not metrics.get('association_witness',False):reasons.append('ASSOCIATION_NOT_CERTIFIED')
    if not metrics.get('support_good',False):reasons.append('LOW_SUPPORT')
    if not metrics.get('engineering_safe',False):reasons.append('ENGINEERING_GUARD')
    return {'accepted':not reasons,'local_converged':local,
            'association_stable':bool(metrics.get('association_witness',False)),
            'reasons':reasons,'status':'QUALIFIED_LOCAL_UPDATE' if not reasons else 'REJECTED_KEEP_PRIOR',
            'STATISTICAL_POSTERIOR_CALIBRATED':False}

def atomic_select(prior,Pprior,candidate,Pcandidate,decision):
    """Frozen A/B selector; state/covariance commit or rollback is atomic."""
    result=dict(decision)
    if result['accepted']:
        from .posterior import valid_prior
        try:
            valid_prior(Pcandidate)
            if not np.isfinite(np.r_[candidate.p,candidate.v,candidate.R.ravel(),candidate.bg,candidate.ba]).all():
                raise ValueError('NONFINITE_STATE')
        except (ValueError,np.linalg.LinAlgError):
            result.update(accepted=False,status='NUMERIC_FAILURE',reasons=['INVALID_CANDIDATE_STATE_OR_P'])
    if result['accepted']:
        return candidate.copy(),np.array(Pcandidate,copy=True),result
    return prior.copy(),np.array(Pprior,copy=True),result

def local_optimality(P,z,J,r,w):
    """Exact objective gradient and UNCLIPPED GN residual in the frozen prior chart."""
    from scipy.linalg import cho_factor,cho_solve,solve_triangular
    L=np.linalg.cholesky(P);u=solve_triangular(L,z,lower=True);A=J@L
    g=u+A.T@(w*r);M=np.eye(len(P))+A.T@(w[:,None]*A)
    du=cho_solve(cho_factor(M),-g)
    return {'gn_white_norm':float(np.linalg.norm(du)),
            'newton_decrement':float(np.sqrt(max(0.,-g@du))),
            'gradient_white_norm':float(np.linalg.norm(g)),
            'gradient_white_inf':float(np.max(abs(g))),
            'gn_delta':L@du}

from .geometry import GeometryCorrespondence
class TrackedCorrespondence(GeometryCorrespondence):
    """Read-only identity reconstruction; original matching and normals unchanged."""
    def get(self,x,source,factor):
        c=super().get(x,source,factor);T=factor.transform(x)
        distance,ids=self.tree.query(source@T[:3,:3].T+T[:3,3],k=1)
        use=np.flatnonzero(distance<=self.c['correspondence_distance_m'])
        use=use[self.valid[ids[use]]]
        if not np.array_equal(ids[use],c['target_ids']) or not np.array_equal(source[use],c['p']):
            raise ValueError('SOURCE_IDENTITY_RECONSTRUCTION_MISMATCH')
        c['source_index']=use
        return c

from .instrumented import InstrumentedUpdate
from .state import inject,chart_map
class QualifiedUpdate(InstrumentedUpdate):
    """One mechanism: post-verify eligibility; otherwise keep both prediction objects.

    No extra solve iterations, changed source/noise/guards, or state clipping.
    Conditional IRLS covariance is unchanged and explicitly uncalibrated.
    """
    def observe(self,x,corr):
        super().observe(x,corr)
        self.qualification_correspondences.append(corr)

    def correct(self,prior,P0,source,correspondence):
        import time
        start=time.perf_counter();self.qualification_correspondences=[]
        candidate,P,info=super().correct(prior,P0,source,correspondence)
        info['historical_proposal_accepted']=info['accepted'];info['historical_solver_reason']=info['reason']
        if not info['accepted']:
            info['qualification']={'accepted':False,'status':'REJECTED_KEEP_PRIOR','reasons':['HISTORICAL_SOLVER_REJECTED'],'local_converged':False,'association_stable':False}
            info['wall_ms']=(time.perf_counter()-start)*1000
            return prior,P0,info
        z=np.array(info['delta']);corr=self.qualification_correspondences[-1]
        r,H,_=self.factor.linearize(candidate,corr['p'],corr['q'],corr['n'])
        J=H@chart_map(z);w=corr['w']*self.robust(r)[1];m=local_optimality(P0,z,J,r,w)
        previous=self.qualification_correspondences[-2]
        # A conservative witness, not a proof of association stability in a neighborhood.
        trial=correspondence.get(inject(prior,z+m['gn_delta']),source,self.factor)
        m.update(association_witness=pair_retention(previous,corr)['identical'] and pair_retention(corr,trial)['identical'],support_good=bool(corr['good'] and trial['good']),engineering_safe=bool(np.linalg.norm(z[:3])<=self.c['max_total_translation_m'] and np.linalg.norm(z[6:9])<=self.c['max_total_rotation_rad'] and np.linalg.norm(z[9:12])<=self.c['max_bias_gyro_change_rad_s'] and np.linalg.norm(z[12:15])<=self.c['max_bias_accel_change_m_s2']))
        decision=eligible(m,self.c['convergence_whitened_step'])
        x,Q,decision=atomic_select(prior,P0,candidate,P,decision)
        info['qualification_metrics']={k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in m.items()};info['qualification']=decision
        info['accepted']=decision['accepted'];info['reason']=decision['status']
        info['covariance_updates']=int(decision['accepted'])
        # Original delta remains a proposal; never mislabel it as committed after rollback.
        info['proposed_delta']=info['delta'];info['delta']=z.tolist() if decision['accepted'] else np.zeros(15).tolist()
        info['wall_ms']=(time.perf_counter()-start)*1000
        return x,Q,info
