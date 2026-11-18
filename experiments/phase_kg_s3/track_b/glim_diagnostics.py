"""CPU correspondence diagnostics, NOT an alternate state estimator or NDT gate.

GLIM source snapshot 1a197c999e4865f5cab7c9cd25474128413ebbca;
gtsam_points 8e94654558dfbe7ddb0341de300daaf17e25ee02, lookup_voxels.cuh.
Surface check is signed ray/rotated SOURCE normal, not normal-normal agreement.
"""
import numpy as np
from scipy.spatial import cKDTree
from local_odometry import transform, occupied


def normals(points, k=20):
    if len(points)<k:raise ValueError('insufficient normal neighborhood')
    _,idx=cKDTree(points).query(points,k=k,workers=1)
    q=points[idx];q=q-q.mean(axis=1,keepdims=True)
    C=np.einsum('nki,nkj->nij',q,q)/k
    _,V=np.linalg.eigh(C);n=V[:,:,0]
    # Explicit source-facing sign convention; upstream orientation uses an
    # unusual dot>1.0 condition, so this is marked a separately defined variant.
    n[np.sum(points*n,axis=1)>0]*=-1
    return n


def surface_mask(points, source_normals, target_T_source):
    p=transform(target_T_source,points)
    n=source_normals@target_T_source[:3,:3].T
    norm=np.linalg.norm(p,axis=1)
    dot=np.divide(np.sum(p*n,axis=1),norm,out=np.zeros(len(p)),where=norm>0)
    return dot<=.174


def multires_support(source, source_normals, target, target_T_source, resolutions=(1.5,3.)):
    q=transform(target_T_source,source);surface=surface_mask(source,source_normals,target_T_source)
    rows=[]
    for resolution in resolutions:
        keys=occupied(target,resolution)
        mask=np.array([tuple(k) in keys for k in np.floor(q/resolution).astype(np.int64)])
        rows.append(dict(resolution=resolution,source_points=len(source),occupied_target_voxels=len(keys),support_without_surface=int(mask.sum()),support_with_surface=int((mask&surface).sum())))
    return dict(method='GLIM-inspired occupancy/surface diagnostic; not native VGICP or NDT optimization',signed_ray_threshold=.174,normal_sign='source-facing explicitly defined; differs from native dot>1 condition',levels=rows)
