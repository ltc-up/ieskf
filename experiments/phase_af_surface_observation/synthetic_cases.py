"""Known physical plane identities, exact surfaces, explicitly generative errors."""
import numpy as np
from geometry_ieskf.common import C
from geometry_ieskf.state import State
from geometry_ieskf.geometry import GeometryFactor
from geometry_ieskf.surface_observation import Plane,tangent_basis,SurfaceObservation

def error_covariance(correlated=False):
    L=np.array([[.01,0,0],[.002,.01,0],[.01,-.005,.1]]) if correlated else np.diag([.01,.01,.1])
    return L@L.T

def prior(correlated=False):
    std=np.repeat(C['initial_std_p_v_theta_bg_ba'],3);L=np.diag(std)
    if correlated:L[5,2]=.7;L[9,6]=.01;L[13,7]=.02
    return State(np.zeros(3),np.zeros(3),np.eye(3),np.zeros(3),np.zeros(3)),L@L.T

def geometry(kind,side=5,correlated=False):
    normals={'horizontal':[[0,0,1]],'two':[[0,0,1],[1,0,0]],'three':[[0,0,1],[1,0,0],[0,1,0]],'finite_local':[[0,0,1],[1,0,0],[0,1,0],[1,1,1]],'finite_coplanar':[[0,0,1]]}[kind]
    planes={};points=[];gids=[]
    for j,n in enumerate(normals):
        n=np.asarray(n,float);n/=np.linalg.norm(n);T=tangent_basis(n);anchor=np.zeros(3) if kind!='finite_local' else n*(.2*j)
        key='plane'+str(j);planes[key]=Plane(key,n,anchor,T,error_covariance(correlated))
        p=np.array([anchor+T@np.array([u,v]) for u in np.linspace(-2,2,side) for v in np.linspace(-2,2,side)])
        if kind=='finite_coplanar':p=np.r_[p+T[:,0]*4,p-T[:,0]*4]
        points.extend(p);gids.extend([key]*len(p))
    points=np.array(points);gids=np.array(gids);return points,np.arange(len(points)),gids,planes

def observation(kind='horizontal',side=5,correlated=False):
    p,ids,gids,planes=geometry(kind,side,correlated);return SurfaceObservation(p,ids,gids,planes,GeometryFactor(np.zeros(3)))
