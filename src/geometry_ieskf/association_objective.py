"""Objective-consistent compact Huber plane mixture. Research, not calibrated R."""
import time
import numpy as np
from scipy.linalg import solve_triangular
from scipy.optimize import minimize
from .common import C,jr
from .state import inject
from .valid_correspondence import NearestValidCorrespondence
from .partition import margins,con_jac_z,kkt
from .capture import step_margins,step_jacobian

class CompactHuberMixture:
    def __init__(self,prior,P,source,factor,query,config=C):
        self.prior=prior;self.P=P;self.L=np.linalg.cholesky(P);self.Q=solve_triangular(self.L,np.eye(15),lower=True)
        self.source=np.asarray(source);self.factor=factor;self.query=query;self.c=config
        self._key=None;self._value=None;self.eval_times=[]
    def evaluate(self,z):
        if self._key is not None and np.array_equal(z,self._key):return self._value
        start=time.perf_counter();x=inject(self.prior,z);T=self.factor.transform(x);y=self.source@T[:3,:3].T+T[:3,3]
        balls=self.query.tree.query_ball_point(y,self.c['correspondence_distance_m'])
        counts=np.array([len(v) for v in balls]);si=np.repeat(np.arange(len(y)),counts)
        ti=np.concatenate([np.asarray(v,dtype=int) for v in balls]) if counts.sum() else np.empty(0,dtype=int)
        self.query.warm_normals(ti);valid=self.query.valid[ti];si=si[valid];ti=ti[valid]
        delta=y[si]-self.query.target[ti];n=self.query.normals[ti]
        R=self.c['correspondence_distance_m'];h=self.c['huber_delta_m'];tau=h*h
        s=np.einsum('ij,ij->i',delta,delta)/R**2;t=np.maximum(0.,1-s);kernel=t**3
        r=np.einsum('ij,ij->i',delta,n);rho=np.where(abs(r)<=h,.5*r*r,h*(abs(r)-.5*h));psi=np.clip(r,-h,h)
        cap=.5*R*R if R<=h else h*(R-.5*h);ex=np.exp(-rho/tau)
        den=1+np.bincount(si,weights=kernel,minlength=len(y));num=np.exp(-cap/tau)+np.bincount(si,weights=kernel*ex,minlength=len(y))
        loss=-tau*np.log(num/den);a=kernel*ex/num[si];b=tau*(1/den[si]-ex/num[si])
        pair_grad=a[:,None]*psi[:,None]*n+b[:,None]*(-6*t*t/R**2)[:,None]*delta
        gy=np.zeros_like(y)
        for j in range(3):gy[:,j]=np.bincount(si,weights=pair_grad[:,j],minlength=len(y))
        gy/=self.c['lidar_sigma_m']**2;g=self.Q.T@(self.Q@z);g[:3]+=gy.sum(0)
        g[6:9]+=np.cross(self.source-self.factor.lever,gy@x.R).sum(0)@jr(z[6:9])
        cost=float(.5*np.sum((self.Q@z)**2)+loss.sum()/self.c['lidar_sigma_m']**2)
        supported=np.unique(si);blocks=len(np.unique(np.floor(self.source[supported]/self.c['source_block_m']).astype(int),axis=0))
        metrics={'support_n':len(supported),'source_n':len(y),'unique_targets':len(np.unique(ti)),'source_blocks':blocks,'fraction':len(supported)/max(1,len(y)),
                 'candidate_pairs':len(ti),'mixture_plane_mass':float(a.sum()),'effective_source_mass':float(np.bincount(si,weights=a,minlength=len(y)).sum()),'prior_cost':float(.5*np.sum((self.Q@z)**2)),
                 'geometry_cost':float(loss.sum()/self.c['lidar_sigma_m']**2),'residual_quantiles':np.percentile(abs(r),[50,90,99,100]).tolist() if len(r) else []}
        metrics['support_good']=bool(metrics['support_n']>=self.c['minimum_support_points'] and metrics['unique_targets']>=self.c['minimum_unique_targets'] and blocks>=self.c['minimum_source_blocks'] and metrics['fraction']>=self.c['minimum_support_fraction'])
        elapsed=1000*(time.perf_counter()-start);self.eval_times.append(elapsed);metrics['evaluation_ms']=elapsed
        self._key=z.copy();self._value=(cost,g,metrics,{'source_index':si,'target_ids':ti,'kernel':kernel,'responsibility':a,'residual':r,'rho':rho,'loss_per_source':loss,'normals':n})
        return self._value
    def cost(self,z):return self.evaluate(z)[0]
    def gradient(self,z):return self.evaluate(z)[1]

def solve_mixture(o,z_start,max_stages=60,on_stage=None):
    z=np.array(z_start,copy=True);history=[];start=time.perf_counter();status='BUDGET_EXHAUSTED'
    if min(margins(z))<0:raise ValueError('INFEASIBLE_START')
    for stage in range(max_stages):
        anchor=z.copy();before=o.cost(z)
        cons=[{'type':'ineq','fun':lambda u:margins(o.L@u),'jac':lambda u:con_jac_z(o.L@u)@o.L},
              {'type':'ineq','fun':lambda u:step_margins(o.L@u,anchor,o.c),'jac':lambda u:step_jacobian(o.L@u,anchor,o.c)@o.L}]
        sol=minimize(lambda u:o.cost(o.L@u),np.linalg.solve(o.L,z),jac=lambda u:o.L.T@o.gradient(o.L@u),constraints=cons,method='SLSQP',options={'ftol':1e-12,'maxiter':200})
        proposed=o.L@sol.x;used=None
        for alpha in o.c['line_search_scales']:
            zz=z+alpha*(proposed-z)
            if min(margins(zz))<0 or min(step_margins(zz,z,o.c))<0:continue
            if np.isfinite(o.cost(zz)) and o.cost(zz)<=before+1e-12:used=alpha;z=zz;break
        value,g,m,_=o.evaluate(z);white=o.L.T@g;kk=kkt(o,z)
        change=float(np.linalg.norm(np.linalg.solve(o.L,z-anchor)))
        row={'stage':stage,'z':z.copy(),'cost_before':before,'cost':value,'gradient_white_inf':float(np.max(abs(white))),'white_step':change,'alpha':used,'inner_success':bool(sol.success),'inner_message':str(sol.message),'inner_nit':int(sol.nit),'metrics':m,'KKT':kk}
        history.append(row)
        if on_stage:on_stage(row)
        if np.max(abs(white))<=o.c['convergence_whitened_step'] and min(margins(z))>1e-5 and m['support_good']:status='INTERIOR_OBJECTIVE_STATIONARY';break
        if kk['optimal'] and kk['active']:status='BOUNDARY_KKT_DIAGNOSTIC';break
        if used is None:status='NO_FEASIBLE_DESCENT';break
        if change<1e-12:status='NUMERIC_STALL_NOT_CONVERGED';break
    cost,g,m,pairs=o.evaluate(z)
    return {'status':status,'z':z,'cost':cost,'gradient_white_inf':float(np.max(abs(o.L.T@g))),'metrics':m,'KKT':kkt(o,z),'history':history,'stages':len(history),'evaluation_times_ms':o.eval_times,'wall_ms':1000*(time.perf_counter()-start),'covariance_updates':0}
