"""Offline qualification utilities; no implied online scheduling or calibrated accuracy."""
import numpy as np
from scipy.spatial.transform import Rotation

def static_transform(transforms,parent,child):
    edges={}
    for r in transforms:
        T=np.eye(4);T[:3,:3]=Rotation.from_quat(r['q']).as_matrix();T[:3,3]=r['p']
        edges.setdefault(r['parent'],[]).append((r['child'],T))
        edges.setdefault(r['child'],[]).append((r['parent'],np.linalg.inv(T)))
    pending=[(parent,np.eye(4))];seen=set()
    while pending:
        name,T=pending.pop()
        if name==child:return T
        if name in seen:continue
        seen.add(name)
        pending.extend((n,T@A) for n,A in edges.get(name,[]) if n not in seen)
    raise ValueError('STATIC_TRANSFORM_UNAVAILABLE')

def closed_traverse(poses):
    """Input segmentation only, never an accuracy criterion or surveyed lap claim."""
    if not poses:return None
    poses=sorted(poses,key=lambda e:e['header_ns']);p=np.array([e['position'] for e in poses]);ts=np.array([e['header_ns'] for e in poses]);s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1))]
    R=Rotation.from_quat([e['quaternion_xyzw'] for e in poses]);yaw=R.as_euler('xyz')[:,2];dy=np.angle(np.exp(1j*(yaw-yaw[0])))
    ok=(ts-ts[0]>60e9)&(s>3000)&(np.linalg.norm(p[:,:2]-p[0,:2],axis=1)<30)&(abs(dy)<np.pi/6)
    ids=np.flatnonzero(ok)
    return None if not len(ids) else dict(begin_ns=int(ts[0]),end_ns=int(ts[ids[0]]),duration_s=float((ts[ids[0]]-ts[0])/1e9),historical_path_m=float(s[ids[0]]))

def percentiles(values):
    return dict(zip(('p50','p95','p99','max'),np.percentile(values,[50,95,99,100]).tolist())) if len(values) else None

def timestamp_qualified(events):
    return bool(events) and all(e['header_ns']>0 for e in events)
