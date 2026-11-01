"""Signed analytic curvature decomposition of the UNCHANGED AD energy.

Summands are algebraic, never independent measurement covariances.
"""
import numpy as np
from scipy.linalg import cho_factor,cho_solve
from scipy.spatial.transform import Rotation
from .common import jr
from .state import inject,chart_map
from .partition import POSE,OTHER

def white_hessian(o,z,eps=1e-5):
    u=np.linalg.solve(o.L,z);columns=[]
    for j in range(15):
        d=np.eye(15)[j]*eps
        columns.append(o.L.T@(o.gradient(o.L@(u+d))-o.gradient(o.L@(u-d)))/(2*eps))
    H=np.array(columns).T
    return (H+H.T)/2,float(np.max(abs(H-H.T)))

def force_gradient(o,z,gy):
    x=inject(o.prior,z);g=np.zeros(15);g[:3]=gy.sum(0)
    g[6:9]=np.cross(o.source-o.factor.lever,gy@x.R).sum(0)@jr(z[6:9])
    return g

def sum_sources(si,v,N):
    out=np.zeros((N,)+v.shape[1:]);np.add.at(out,si,v);return out

def decompose(o,z,eps=1e-5):
    cost,g,metrics,p=o.evaluate(z);si=p['source_index'];ti=p['target_ids'];N=len(o.source);n=p['normals'];r=p['residual'];rho=p['rho'];k=p['kernel'];a=p['responsibility']
    x=inject(o.prior,z);T=o.factor.transform(x);y=o.source@T[:3,:3].T+T[:3,3];d=y[si]-o.query.target[ti]
    R=o.c['correspondence_distance_m'];h=o.c['huber_delta_m'];tau=h*h;sigma2=o.c['lidar_sigma_m']**2
    t=np.maximum(0.,1-np.einsum('ni,ni->n',d,d)/R**2);u=-6*t[:,None]**2*d/R**2
    Hk=-6*t[:,None,None]**2*np.eye(3)/R**2+24*t[:,None,None]*np.einsum('ni,nj->nij',d,d)/R**4
    v=np.clip(r,-h,h)[:,None]*n;Hr=(abs(r)<h)[:,None,None]*np.einsum('ni,nj->nij',n,n)
    cap=.5*R*R if R<=h else h*(R-.5*h);B=np.exp(-rho/tau)
    D=1+np.bincount(si,weights=k,minlength=N);Z=np.exp(-cap/tau)+np.bincount(si,weights=k*B,minlength=N)
    b=B/Z[si];V=sum_sources(si,a[:,None]*v,N);U=sum_sources(si,b[:,None]*u,N);gD=sum_sources(si,u,N)
    outer=lambda x,y:np.einsum('ni,nj->nij',x,y)
    Hy={
        'plane_intrinsic':sum_sources(si,a[:,None,None]*Hr,N),
        'residual_responsibility_covariance':-(sum_sources(si,a[:,None,None]*outer(v,v),N)-outer(V,V))/tau,
        'kernel_residual_cross':sum_sources(si,b[:,None,None]*(outer(u,v)+outer(v,u)),N)-(outer(U,V)+outer(V,U)),
        'numerator_kernel_first':tau*outer(U,U),
        'numerator_kernel_second':-tau*sum_sources(si,b[:,None,None]*Hk,N),
        'normalization_density_first':-tau*outer(gD,gD)/D[:,None,None]**2,
        'normalization_density_second':tau*sum_sources(si,Hk,N)/D[:,None,None]}
    body=o.source-o.factor.lever;sk=np.zeros((N,3,3));sk[:,0,1]=-body[:,2];sk[:,0,2]=body[:,1];sk[:,1,0]=body[:,2];sk[:,1,2]=-body[:,0];sk[:,2,0]=-body[:,1];sk[:,2,1]=body[:,0]
    F=np.zeros((N,3,15));F[:,:,:3]=np.eye(3);F[:,:,6:9]=-np.einsum('ab,nbc,cd->nad',x.R,sk,jr(z[6:9]));A=F@o.L
    parts={name:np.einsum('nai,nab,nbj->ij',A,value,A)/sigma2 for name,value in Hy.items()}
    forces={'plane':V/sigma2,'kernel_numerator':-tau*U/sigma2,'normalization_density':tau*gD/D[:,None]/sigma2}
    gy=sum(forces.values());u0=np.linalg.solve(o.L,z);columns=[]
    for j in range(15):
        du=np.eye(15)[j]*eps
        columns.append(o.L.T@(force_gradient(o,o.L@(u0+du),gy)-force_gradient(o,o.L@(u0-du),gy))/(2*eps))
    rawchart=np.array(columns).T;parts['state_chart_curvature']=(rawchart+rawchart.T)/2;parts['prior']=np.eye(15)
    H=sum(parts.values());fd,asym=white_hessian(o,z,eps);forces_white={key:o.L.T@force_gradient(o,z,value) for key,value in forces.items()};forces_white['prior']=np.linalg.solve(o.L,z)
    return {'cost':cost,'metrics':metrics,'H_white':H,'H_white_numeric':fd,'parts_white':parts,'parts_physical':{key:o.Q.T@M@o.Q for key,M in parts.items()},'numeric_asymmetry_inf':asym,'reconstruction_relative':float(np.linalg.norm(H-fd)/max(1.,np.linalg.norm(fd))),'reconstruction_inf':float(np.max(abs(H-fd))),'forces_white':forces_white,'total_gradient_white':o.L.T@g,'force_sum_error':float(np.max(abs(sum(forces_white.values())-o.L.T@g)))}

def curvature_summary(o,z,H):
    H=(H+H.T)/2;ev,V=np.linalg.eigh(H);r={'eigenvalues':ev,'eigenvectors_white':V,'condition_absolute':float(max(abs(ev))/max(min(abs(ev)),1e-300)),'SPD':bool(ev.min()>0),'gradient_white_inf':float(np.max(abs(o.L.T@o.gradient(z))))}
    if ev.min()<=0:r['sigma_status']='UNDEFINED_NONPOSITIVE_CURVATURE';return r
    try:Q=o.L@cho_solve(cho_factor(H),o.L.T)
    except np.linalg.LinAlgError:r['sigma_status']='CHOLESKY_FAILURE';return r
    G=chart_map(z);P=G@Q@G.T;P=(P+P.T)/2;x=inject(o.prior,z)
    E=np.zeros((3,3));step=1e-6
    for j in range(3):
        a=Rotation.from_matrix(x.R@Rotation.from_rotvec(np.eye(3)[j]*step).as_matrix()).as_euler('xyz');b=Rotation.from_matrix(x.R@Rotation.from_rotvec(-np.eye(3)[j]*step).as_matrix()).as_euler('xyz');E[:,j]=((a-b+np.pi)%(2*np.pi)-np.pi)/(2*step)
    K=np.linalg.solve(P[np.ix_(POSE,POSE)],P[np.ix_(OTHER,POSE)].T).T
    aZ=o.L.T[:,2];hz=cho_solve(cho_factor(H),aZ);direction=hz/(aZ@hz)
    r.update(sigma_status='INVERSE_ENERGY_CURVATURE_ONLY',sigma_xyz=np.sqrt(np.diag(P)[:3]),sigma_rpy=np.sqrt(np.diag(E@P[6:9,6:9]@E.T)),P=P,P_chart=Q,K_other_pose=K,K_vz_Z=float(K[2,2]),P_vz_Z=float(P[5,2]),rho_Z_vz=float(P[2,5]/np.sqrt(P[2,2]*P[5,5])),marginal_Z_direction_white=direction,marginal_Z_precision=float(1/Q[2,2]))
    return r

def polish(o,z,max_steps=3):
    from .partition import margins
    from .capture import step_margins
    z=np.array(z,copy=True);hist=[]
    for j in range(max_steps):
        g=o.L.T@o.gradient(z)
        if np.max(abs(g))<=o.c['convergence_whitened_step']:break
        H,asym=white_hessian(o,z);ev=np.linalg.eigvalsh(H)
        if ev.min()<=0:hist.append({'reason':'NONPOSITIVE_HESSIAN'});break
        du=np.linalg.solve(H,-g);used=None;before=o.cost(z)
        for alpha in o.c['line_search_scales']:
            trial=z+alpha*(o.L@du)
            if min(margins(trial))<0 or min(step_margins(trial,z))<0:continue
            after=o.cost(trial)
            if after<=before+1e-10 and np.max(abs(o.L.T@o.gradient(trial)))<np.max(abs(g)):
                hist.append({'alpha':alpha,'actual_cost_change':after-before,'predicted_change':float(g@du+.5*du@H@du),'white_step':float(np.linalg.norm(du))});z=trial;used=alpha;break
        if used is None:break
    return z,hist
