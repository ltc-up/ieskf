"""Plane-distance observations with shared latent surface error, not AD curvature."""
from dataclasses import dataclass
import numpy as np
from scipy.linalg import cho_factor,cho_solve
from .common import C
from .geometry import GeometryFactor
from .state import chart_map,inject
from .posterior import valid_prior
from .partition import margins
from .capture import step_margins

@dataclass
class Plane:
    key:str
    normal:np.ndarray
    anchor:np.ndarray
    tangent:np.ndarray
    covariance:np.ndarray

def tangent_basis(n):
    n=np.asarray(n,float);n=n/np.linalg.norm(n);e=np.eye(3)[np.argmin(abs(n))];u=e-n*(n@e);u/=np.linalg.norm(u)
    return np.column_stack([u,np.cross(n,u)])

def canonical_measurements(points,sample_ids,plane_ids):
    points=np.asarray(points,float);ids=np.asarray(sample_ids);groups=np.asarray(plane_ids)
    if points.shape!=(len(ids),3) or len(groups)!=len(ids) or not np.isfinite(points).all():raise ValueError('INVALID_SOURCE_RECORDS')
    unique,first=np.unique(ids,return_index=True)
    for i in first:
        use=ids==ids[i]
        if not np.all(points[use]==points[i]) or not np.all(groups[use]==groups[i]):raise ValueError('INCONSISTENT_DUPLICATE_SAMPLE_ID')
    order=np.sort(first);return points[order],ids[order],groups[order],order

class SharedPlaneNoise:
    """R=D+B B^T; one3D latent per ORIGINAL physical-plane identity.

    Residual copies must be removed before constructing this object. Repsilon is
    independent ONLY across distinct physical sample identities under the model.
    """
    def __init__(self,plane_ids,Jplane,planes,epsilon_variance):
        self.ids=np.asarray(plane_ids);self.J=np.asarray(Jplane,float);N=len(self.ids)
        self.D=np.broadcast_to(np.asarray(epsilon_variance,float),(N,)).copy()
        if self.J.shape!=(N,3) or not np.isfinite(self.J).all() or not np.isfinite(self.D).all() or np.any(self.D<=0):raise ValueError('INVALID_SHARED_NOISE')
        self.blocks=[]
        for key in np.unique(self.ids):
            ix=np.flatnonzero(self.ids==key);S=np.asarray(planes[str(key)].covariance,float)
            if S.shape!=(3,3) or not np.isfinite(S).all() or not np.allclose(S,S.T,atol=1e-13):raise ValueError('INVALID_PLANE_COVARIANCE')
            L=np.linalg.cholesky(S);B=self.J[ix]@L;M=np.eye(3)+B.T@(B/self.D[ix,None]);self.blocks.append((ix,B,cho_factor(M)))
    def solve(self,V):
        V=np.asarray(V,float);one=V.ndim==1;V=V[:,None] if one else V
        if V.shape[0]!=len(self.ids) or not np.isfinite(V).all():raise ValueError('INVALID_NOISE_RHS')
        out=V/self.D[:,None]
        for ix,B,M in self.blocks:out[ix]-=(B/self.D[ix,None])@cho_solve(M,B.T@out[ix])
        return out[:,0] if one else out
    def quadratic(self,r):return float(r@self.solve(r))
    def covariance_product(self,K):
        out=(K*self.D)@K.T
        for ix,B,M in self.blocks:
            KB=K[:,ix]@B;out+=KB@KB.T
        return out
    def dense(self):
        R=np.diag(self.D)
        for ix,B,M in self.blocks:R[np.ix_(ix,ix)]+=B@B.T
        return R

class SurfaceObservation:
    def __init__(self,points,sample_ids,plane_ids,planes,factor,epsilon_variance=C['lidar_sigma_m']**2):
        self.points,self.sample_ids,self.plane_ids,self.original_rows=canonical_measurements(points,sample_ids,plane_ids);self.planes=planes;self.factor=factor;self.epsilon_variance=epsilon_variance
        for key in np.unique(self.plane_ids):
            p=planes[str(key)]
            if abs(np.linalg.norm(p.normal)-1)>1e-10 or p.tangent.shape!=(3,2) or not np.allclose(p.tangent.T@p.normal,0,atol=1e-10):raise ValueError('INVALID_PLANE_FRAME')
    def linearize(self,x,z_chart=None):
        N=len(self.points);r=np.empty(N);H=np.zeros((N,15));Jplane=np.empty((N,3))
        T=self.factor.transform(x);y=self.points@T[:3,:3].T+T[:3,3]
        for key in np.unique(self.plane_ids):
            ix=np.flatnonzero(self.plane_ids==key);p=self.planes[str(key)];n=np.broadcast_to(p.normal,(len(ix),3));q=np.broadcast_to(p.anchor,(len(ix),3));rr,HH,_=self.factor.linearize(x,self.points[ix],q,n)
            r[ix]=rr;H[ix]=HH;Jplane[ix,:2]=(y[ix]-p.anchor)@p.tangent;Jplane[ix,2]=-1.
        if z_chart is not None:H=H@chart_map(z_chart)
        noise=SharedPlaneNoise(self.plane_ids,Jplane,self.planes,self.epsilon_variance)
        return r,H,Jplane,noise

def linear_surface_update(P,H,r,noise):
    P=valid_prior(P);H=np.asarray(H,float);r=np.asarray(r,float)
    if H.shape!=(len(r),15) or len(r)==0 or not np.isfinite(H).all() or not np.isfinite(r).all():raise ValueError('INVALID_SURFACE_LINEARIZATION')
    L=np.linalg.cholesky(P);A=H@L;RiA=noise.solve(A);M=np.eye(15)+A.T@RiA;M=(M+M.T)/2;Cq=cho_factor(M)
    du=cho_solve(Cq,-A.T@noise.solve(r));delta=L@du;Ppost=L@cho_solve(Cq,L.T);Ppost=(Ppost+Ppost.T)/2
    K=noise.solve(H@Ppost).T;B=np.eye(15)-K@H;Joseph=B@P@B.T+noise.covariance_product(K);Joseph=(Joseph+Joseph.T)/2
    information=H.T@noise.solve(H);b=-H.T@noise.solve(r)
    return {'delta':delta,'P_chart':Ppost,'P_Joseph':Joseph,'Joseph_relative_error':float(np.linalg.norm(Ppost-Joseph)/np.linalg.norm(Ppost)),'information':information,'information_vector':b,'gain':K,'innovation_quadratic':noise.quadratic(r),'model_scope':'FROZEN_LINEAR_GAUSSIAN_SHARED_PLANE_ASSUMPTIONS_ONLY'}

def atomic_surface_update(prior,P,proposal,support_good=True,statistically_calibrated=False):
    """Independent frozen transaction; no AD energy/Hessian and no state clipping."""
    keep=lambda reason:{'state':prior.copy(),'P':np.array(P,copy=True),'accepted':False,'reason':reason,'covariance_updates':0}
    try:
        valid_prior(P);z=np.asarray(proposal['delta']);Q=np.asarray(proposal['P_chart']);valid_prior(Q)
        if z.shape!=(15,) or not np.isfinite(z).all():raise ValueError('NONFINITE_DELTA')
    except (ValueError,np.linalg.LinAlgError,KeyError):return keep('NUMERIC_FAILURE')
    if not support_good:return keep('LOW_SURFACE_SUPPORT')
    if min(margins(z))<0:return keep('CUMULATIVE_ENGINEERING_GUARD')
    if min(step_margins(z,np.zeros(15)))<0:return keep('SINGLE_LINEARIZATION_TRUST_GUARD')
    G=chart_map(z);Q=G@Q@G.T;Q=(Q+Q.T)/2
    try:valid_prior(Q)
    except (ValueError,np.linalg.LinAlgError):return keep('INVALID_RESET_POSTERIOR')
    return {'state':inject(prior,z),'P':Q,'P_chart':proposal['P_chart'],'accepted':True,'reason':'ISOLATED_MODEL_CONDITIONAL_LINEAR_UPDATE','covariance_updates':1,'NUMERIC_POSTERIOR_VALID':True,'STATISTICAL_POSTERIOR_CALIBRATED':bool(statistically_calibrated)}


def plane_registry(records):
    """Only an explicit same physical identity shares a latent; no silent aliases."""
    out={}
    for p in records:
        if p.key in out:
            old=out[p.key]
            if not all(np.array_equal(getattr(old,k),getattr(p,k)) for k in ['normal','anchor','tangent','covariance']):raise ValueError('INCONSISTENT_PHYSICAL_PLANE_FRAME')
        else:out[p.key]=p
    return out

def transported_plane_covariance(S,normal,tangent,anchor_shift):
    """Same plane error at another in-plane anchor: d_new=d_old-alpha dot shift."""
    shift=np.asarray(anchor_shift)
    if abs(np.asarray(normal)@shift)>1e-10:raise ValueError('ANCHOR_SHIFT_NOT_IN_PLANE')
    B=np.eye(3);B[2,:2]=-np.asarray(tangent).T@shift
    return B@S@B.T,B
