"""Frozen-prior alternating geometry capture. Diagnostic states are not posteriors."""
import time
import numpy as np
from scipy.linalg import solve_triangular
from scipy.optimize import minimize
from .common import C
from .state import inject
from .objective import Objective
from .partition import BOUNDS,margins,con_jac_z,kkt,Partition
from .qualification import local_optimality,pair_retention,eligible

def step_margins(z,anchor,config=C):
    d=z-anchor
    return np.array([1-d[:3]@d[:3]/config['max_translation_step_m']**2,
                     1-d[6:9]@d[6:9]/config['max_rotation_step_rad']**2])

def step_jacobian(z,anchor,config=C):
    d=z-anchor;J=np.zeros((2,15));J[0,:3]=-2*d[:3]/config['max_translation_step_m']**2
    J[1,6:9]=-2*d[6:9]/config['max_rotation_step_rad']**2
    return J

def starting_seeds(P):
    """Predetermined pose-white axes, conditional other-state means; prior unchanged."""
    ix=np.array([0,1,2,6,7,8]);other=np.array([3,4,5,9,10,11,12,13,14])
    Pp=P[np.ix_(ix,ix)];L=np.linalg.cholesky(Pp);K=np.linalg.solve(Pp,P[np.ix_(other,ix)].T).T
    seeds=[{'id':'nominal','z':np.zeros(15),'white_radius':0.}]
    for j in range(6):
        direction=np.zeros(15);direction[ix]=L[:,j];direction[other]=K@L[:,j]
        radius=.25
        for name,indices,limit in BOUNDS:
            radius=min(radius,.25*limit/max(np.linalg.norm(direction[indices]),1e-30))
        for sign in [-1,1]:
            z=sign*radius*direction
            seeds.append({'id':f'axis{j}_{"minus" if sign<0 else "plus"}','z':z,
                          'white_radius':float(np.linalg.norm(np.linalg.solve(np.linalg.cholesky(P),z)))})
    return seeds

def assess(prior,P,z,corr,factor):
    o=Objective(prior,P,corr,factor);r,J,rho,h=o.data(z)
    m=local_optimality(P,z,J,r,corr['w']*h)
    m.update(cost=o.cost(z),prior_cost=float(.5*np.sum(np.linalg.solve(o.L,z)**2)),
             geometry_cost=float(corr['w']@rho),support_good=bool(corr['good']),
             engineering_safe=bool(np.min(margins(z))>=0),engineering_margins=margins(z),
             residual_quantiles=np.percentile(abs(r),[50,90,95,99,100]) if len(r) else np.array([]))
    return o,m

class AlternatingCapture:
    def __init__(self,factor,config=C,max_outer=60):
        self.factor=factor;self.c=config;self.max_outer=max_outer

    def run(self,prior,P,source,query,z_start=None):
        start=time.perf_counter();z=np.zeros(15) if z_start is None else np.array(z_start,copy=True)
        if np.min(margins(z))<0:raise ValueError('INFEASIBLE_OPTIMIZER_START')
        L=np.linalg.cholesky(P);history=[];status='OUTER_BUDGET_EXHAUSTED';certificate=None
        corr=query.get(inject(prior,z),source,self.factor)
        for outer in range(self.max_outer):
            if not corr['good']:status='LOW_SUPPORT';break
            o,before=assess(prior,P,z,corr,self.factor);anchor=z.copy();u0=solve_triangular(L,z,lower=True)
            evaluations=[]
            def fun(u):
                zz=L@u;value=o.cost(zz);evaluations.append({'cost':value,'z':zz.copy()});return value
            constraints=[{'type':'ineq','fun':lambda u:margins(L@u),'jac':lambda u:con_jac_z(L@u)@L},
                         {'type':'ineq','fun':lambda u:step_margins(L@u,anchor,self.c),'jac':lambda u:step_jacobian(L@u,anchor,self.c)@L}]
            tick=time.perf_counter();sol=minimize(fun,u0,jac=lambda u:L.T@o.gradient(L@u),constraints=constraints,method='SLSQP',options={'maxiter':200,'ftol':1e-12});proposed=L@sol.x
            # Original scale set; no guard relaxation or state-component clipping.
            used=None;nextz=z.copy();fixed_after=o.cost(z)
            for alpha in self.c['line_search_scales']:
                trial=z+alpha*(proposed-z)
                if np.min(margins(trial))<0 or np.min(step_margins(trial,z,self.c))<0:continue
                value=o.cost(trial)
                if np.isfinite(value) and value<=before['cost']+1e-12:
                    nextz=trial;fixed_after=value;used=float(alpha);break
            if used is None:
                # A zero feasible step can still be a legitimate stationary boundary.
                nextz=z.copy();fixed_after=before['cost']
            newcorr=query.get(inject(prior,nextz),source,self.factor)
            pair=pair_retention(corr,newcorr);no,nm=assess(prior,P,nextz,newcorr,self.factor)
            trialcorr=query.get(inject(prior,nextz+nm['gn_delta']),source,self.factor) if np.isfinite(nm['gn_delta']).all() else newcorr
            witness=bool(pair['identical'] and pair_retention(newcorr,trialcorr)['identical'])
            nm.update(association_witness=witness,support_good=bool(newcorr['good'] and trialcorr['good']))
            certificate=eligible(nm,self.c['convergence_whitened_step']);global_kkt=kkt(no,nextz)
            change=float(np.linalg.norm(solve_triangular(L,nextz-z,lower=True)))
            history.append({'outer':outer,'z_before':z.copy(),'z_after':nextz.copy(),
                'fixed_cost_before':before['cost'],'fixed_cost_after':fixed_after,
                'reassociated_selected_factor_cost':nm['cost'],'reassociated_metrics':nm,
                'support_before':corr['metrics'],'support_after':newcorr['metrics'],
                'pair_change':pair,'exact_next_GN_pairs':pair_retention(newcorr,trialcorr),
                'inner_success':bool(sol.success),'inner_message':str(sol.message),'inner_iterations':int(sol.nit),
                'inner_evaluations':evaluations,'line_scale':used,'white_outer_change':change,
                'global_original_guard_KKT':global_kkt,'wall_ms':1000*(time.perf_counter()-tick)})
            if getattr(self,'on_iteration',None):self.on_iteration(history[-1])
            z=nextz;corr=newcorr
            if not newcorr['good']:status='REASSOCIATION_SUPPORT_FAILURE';break
            if certificate['accepted']:status='INTERIOR_ASSOCIATION_FIXED_POINT';break
            if pair['identical'] and global_kkt['optimal'] and global_kkt['active']:
                status='BOUNDARY_ASSOCIATION_FIXED_POINT_DIAGNOSTIC';break
            if used is None:status='NO_FEASIBLE_FIXED_OBJECTIVE_DESCENT';break
            if change<self.c['convergence_whitened_step'] and pair['identical']:
                status='SMALL_STEP_WITHOUT_OPTIMALITY';break
        o,final=assess(prior,P,z,corr,self.factor)
        return {'status':status,'interior_qualified':status=='INTERIOR_ASSOCIATION_FIXED_POINT',
                'z':z,'final':final,'certificate':certificate,'history':history,
                'outer_iterations':len(history),'final_support':corr['metrics'],
                'final_original_guard_KKT':kkt(o,z),'wall_ms':1000*(time.perf_counter()-start),
                'covariance_updates':0,'posterior_scope':'NONE: capture diagnostic, not state/P commit'}
