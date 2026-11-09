"""Offline polygon audit. Geometry is caller supplied; no vehicle dimensions assumed."""
import numpy as np

def contains(ring, p):
    ring=np.asarray(ring,float);p=np.asarray(p,float);a=ring;b=np.roll(a,-1,axis=0)
    v=b-a;u=np.clip(np.sum((p-a)*v,axis=1)/np.maximum(np.sum(v*v,axis=1),1e-30),0,1)
    if np.min(np.linalg.norm(p-a-u[:,None]*v,axis=1))<1e-9:return True
    return bool(np.count_nonzero(((a[:,1]>p[1])!=(b[:,1]>p[1])) & (p[0]<(b[:,0]-a[:,0])*(p[1]-a[:,1])/(b[:,1]-a[:,1]+1e-300)+a[:,0]))%2)

def segments_intersect(a,b,c,d):
    def cross(x,y):return x[0]*y[1]-x[1]*y[0]
    u=b-a;v=d-c;den=cross(u,v)
    if abs(den)<1e-12:
        if abs(cross(c-a,u))>1e-9:return False
        return bool(np.all(np.maximum(np.minimum(a,b),np.minimum(c,d))<=np.minimum(np.maximum(a,b),np.maximum(c,d))+1e-9))
    t=cross(c-a,v)/den;s=cross(c-a,u)/den
    return -1e-9<=t<=1+1e-9 and -1e-9<=s<=1+1e-9

def polygon_audit(footprint,outer,inner):
    """Exact segment intersections and ring containment, including enclosed holes.

    clearance is unsigned boundary separation, NOT penetration depth. Touches
    count as intersection. A certified safety margin additionally needs error bounds.
    """
    f=np.asarray(footprint,float);rings=[np.asarray(outer,float),np.asarray(inner,float)]
    if len(f)<3 or not np.isfinite(f).all():raise ValueError('INVALID_FOOTPRINT')
    crossing=False;distance=np.inf
    def pd(p,a,b):
        v=b-a;u=np.clip(np.dot(p-a,v)/max(np.dot(v,v),1e-30),0,1);return np.linalg.norm(p-a-u*v)
    for a,b in zip(f,np.roll(f,-1,axis=0)):
        for ring in rings:
            c=ring;d=np.roll(ring,-1,axis=0);v=d-c;u=b-a;w=c-a
            cross=lambda x,y:x[...,0]*y[...,1]-x[...,1]*y[...,0]
            den=cross(u,v);parallel=abs(den)<1e-12;safe=np.where(parallel,1.,den)
            tt=cross(w,v)/safe;ss=cross(w,u)/safe
            hit=(~parallel)&(tt>=-1e-9)&(tt<=1+1e-9)&(ss>=-1e-9)&(ss<=1+1e-9)
            collinear=parallel&(abs(cross(w,u))<1e-9)
            hit|=collinear&np.all(np.maximum(np.minimum(a,b),np.minimum(c,d))<=np.minimum(np.maximum(a,b),np.maximum(c,d))+1e-9,axis=1)
            crossing|=bool(np.any(hit))
            def point_segments(p,c,d):
                z=d-c;ratio=np.clip(np.sum((p-c)*z,axis=-1)/np.maximum(np.sum(z*z,axis=-1),1e-30),0,1)
                return np.linalg.norm(p-c-ratio[...,None]*z,axis=-1)
            distance=min(distance,0. if np.any(hit) else float(min(point_segments(a,c,d).min(),point_segments(b,c,d).min(),point_segments(c,a,b).min(),point_segments(d,a,b).min())))
    vertices_inside=all(contains(rings[0],p) and not contains(rings[1],p) for p in f)
    encloses_hole=contains(f,rings[1][0])
    return dict(contained=bool(vertices_inside and not crossing and not encloses_hole),boundary_intersection=bool(crossing),encloses_hole=bool(encloses_hole),boundary_separation=float(distance))

def antenna_to_base(antenna,R_map_base,t_base_antenna):
    return np.asarray(antenna)-np.einsum('...ij,j->...i',R_map_base,t_base_antenna)

def project_footprint(points,rotation,translation):
    """Ordered 2D polygon or 3D convex-solid vertices, projected into map XY.

    The 3D case returns the convex silhouette including height, roll and pitch.
    A convex silhouette does not recover unknown physical body/tyre dimensions.
    """
    from scipy.spatial import ConvexHull
    points=np.asarray(points,float);is3d=points.shape[1]==3
    xyz=points if is3d else np.c_[points,np.zeros(len(points))]
    world=(xyz@np.asarray(rotation).T+translation)[:,:2]
    return world[ConvexHull(world).vertices] if is3d else world
