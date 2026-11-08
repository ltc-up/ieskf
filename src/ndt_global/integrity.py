"""Geometry/provenance diagnostics only: never a statistical pose covariance."""
from dataclasses import dataclass
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from .shadow import se3

@dataclass(frozen=True)
class GeometryConfig:
    distance_m: float = 2.
    neighborhood: int = 20
    cell_m: float = 2.
    rotation_scale_m: float = 10.
    max_source_samples: int = 1500
    planarity_min: float = .2
    relative_rank_tol: float = 1e-6

def directional(points, normals, rotation_scale_m=10., weights=None, tol=1e-6):
    """Left perturbation at base origin; [dt(m), L*dtheta(m)] coordinates.

    Points must be relative to that origin, normals in matching parent frame.
    H=[n, (p cross n)/L]. A unit rotation coordinate is 1/L radians.
    No inversion into covariance. Threshold is a numeric diagnostic, not calibration.
    """
    p,n=np.asarray(points,float),np.asarray(normals,float)
    if rotation_scale_m<=0:raise ValueError('INVALID_SCALE')
    if p.shape!=n.shape or p.ndim!=2 or p.shape[1]!=3:raise ValueError('GEOMETRY_SHAPE')
    if len(p)==0:return dict(rank=0,eigenvalues=[0.]*6,basis=[],normal_eigenvalues=[0.]*3,normalization_m=rotation_scale_m)
    n=n/np.linalg.norm(n,axis=1)[:,None]
    H=np.c_[n,np.cross(p,n)/rotation_scale_m]
    w=np.ones(len(p)) if weights is None else np.asarray(weights)
    if not np.isfinite(H).all() or np.any(w<0) or w.sum()<=0:raise ValueError('INVALID_GEOMETRY')
    M=(H.T*w)@H/w.sum();values,V=np.linalg.eigh(M);keep=values>max(values[-1]*tol,1e-12)
    return dict(rank=int(keep.sum()),eigenvalues=values.tolist(),basis=V[:,keep].T.tolist(),
      normal_eigenvalues=np.linalg.eigvalsh((n.T*w)@n/w.sum()).tolist(),normalization_m=rotation_scale_m,
      covariance='NOT_A_COVARIANCE')

class MapSupport:
    def __init__(self,target,config=GeometryConfig()):
        self.target=np.asarray(target);self.tree=cKDTree(self.target);self.config=config
    def diagnose(self,source,T):
        c=self.config;T=se3(T);s=np.asarray(source);s=s[np.isfinite(s).all(axis=1)]
        if len(s)==0:return dict(classification='NO_CLOUD',source_count=0,sampled=0)
        ids=np.linspace(0,len(s)-1,min(len(s),c.max_source_samples),dtype=int);q=s[ids]
        world=q@T[:3,:3].T+T[:3,3];dist,nn=self.tree.query(world,workers=1)
        good=dist<=c.distance_m;used=nn[good];qrel=(q@T[:3,:3].T)[good]
        if not len(used):return dict(classification='MAP_SUPPORT_MISSING',source_count=len(s),sampled=len(q),nearby_fraction=0.,rank=0,geometry=None)
        k=min(c.neighborhood,len(self.target));_,neighbors=self.tree.query(self.target[used],k=k,workers=1)
        patches=self.target[neighbors];center=patches.mean(axis=1);delta=patches-center[:,None,:]
        eig,V=np.linalg.eigh(np.einsum('nki,nkj->nij',delta,delta)/k)
        planar=(eig[:,1]-eig[:,0])/np.maximum(eig[:,2],1e-12)>c.planarity_min
        cells=np.floor((self.target[used]-T[:3,3])/c.cell_m).astype(int)
        unique,inv,counts=np.unique(cells,axis=0,return_inverse=True,return_counts=True)
        # Equal cell mass avoids point count domination, but cells are not asserted independent surfaces.
        weights=1/counts[inv]
        geo=directional(qrel[planar],V[planar,:,0],c.rotation_scale_m,weights[planar],c.relative_rank_tol)
        rays=np.arctan2(q[:,1],q[:,0]);sector=np.floor((rays+np.pi)/(2*np.pi)*8).astype(int)%8
        covered=len(set(sector[good]));available=len(set(sector))
        return dict(classification='PARTIAL_DIRECTION' if geo['rank']<6 else 'VALID_CANDIDATE',
          source_count=len(s),sampled=len(q),nearby_target_samples=int(good.sum()),nearby_fraction=float(good.mean()),
          unique_target_points=int(len(set(used))),support_cells=len(unique),covered_sectors=covered,source_sectors=available,
          planar_samples=int(planar.sum()),rank=geo['rank'],geometry=geo,
          point_count_is_information=False,map_missing_proven=False,per_lidar_support='UNKNOWN_WITHOUT_POINT_LINEAGE')

def cluster(candidates,translation_m=1.,rotation_rad=.1):
    """Deterministic complete-link clustering; one input group, many seed hypotheses.

    A cluster requires every member to lie within both bounds, avoiding chain collapse.
    Bounds are declared resolution diagnostics, not error probabilities.
    """
    groups=[]
    for i,c in enumerate(candidates):
        if not c['converged'] or not c.get('candidate_valid',True):continue
        T=se3(c['pose'])
        def close(j):
            U=se3(candidates[j]['pose'])
            return np.linalg.norm(T[:3,3]-U[:3,3])<=translation_m and Rotation.from_matrix(T[:3,:3].T@U[:3,:3]).magnitude()<=rotation_rad
        for g in groups:
            if all(close(j) for j in g):g.append(i);break
        else:groups.append([i])
    return dict(clusters=groups,classification='AMBIGUOUS' if len(groups)>1 else 'UNKNOWN' if len(groups)==1 else 'NOT_CONVERGED',
                uniqueness_verified=False,independent_observations=0)

def classify(*,received=True,fresh=True,converged=None,support=None,modes=None,consistent=None):
    if not received:return 'NO_CLOUD'
    if not fresh:return 'STALE'
    if converged is False:return 'NOT_CONVERGED'
    if converged is None:return 'UNKNOWN'
    if support is None:return 'UNKNOWN'
    if support['classification'] in ('NO_CLOUD','MAP_SUPPORT_MISSING'):return support['classification']
    if modes and modes['classification']=='AMBIGUOUS':return 'AMBIGUOUS'
    if consistent is False:return 'INCONSISTENT'
    rank=support['rank']
    if rank==0:return 'WEAK_GEOMETRY'
    if rank<6:return 'PARTIAL_DIRECTION'
    # A single-start fit cannot certify absence of alternative basins.
    return 'UNKNOWN' if modes is None or not modes.get('uniqueness_verified') or consistent is None else 'VALID_CANDIDATE'
