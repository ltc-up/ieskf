"""One association change: nearest among originally admissible plane Targets.

    Original radius, normal fit/predicate, Source support and Target weights unchanged.
    No k-neighbor tuning: exact radius candidates are evaluated only when the raw
    nearest Target is invalid. This is not soft association or a new backend.
"""
import time
import numpy as np
from .geometry import GeometryCorrespondence

class NearestValidCorrespondence(GeometryCorrespondence):
    def warm_normals(self,ids):
        missing=np.unique(ids);missing=missing[~self.cached[missing]]
        if len(missing):
            k=min(self.c['normal_neighbors'],len(self.target));dd,ii=self.tree.query(self.target[missing],k=k)
            if k>=3:
                pts=self.target[ii];dev=pts-pts.mean(1,keepdims=True)
                ev,V=np.linalg.eigh(np.einsum('nki,nkj->nij',dev,dev)/k)
                self.normals[missing]=V[:,:,0]
                self.valid[missing]=(k==self.c['normal_neighbors'])&(dd[:,-1]<=self.c['normal_radius_m'])&(ev[:,0]/np.maximum(ev.sum(1),1e-20)<=self.c['normal_planarity_ratio'])&(ev[:,1]>=self.c['normal_second_eigenvalue_m2'])
            self.cached[missing]=True
        return len(missing)

    def get(self,x,source,factor):
        start=time.perf_counter();T=factor.transform(x);y=source@T[:3,:3].T+T[:3,3]
        distance,ids=self.tree.query(y,k=1);inside=np.flatnonzero(distance<=self.c['correspondence_distance_m']);query_ms=(time.perf_counter()-start)*1000
        tick=time.perf_counter();new=self.warm_normals(ids[inside]);normal_ms=(time.perf_counter()-tick)*1000
        failed=inside[~self.valid[ids[inside]]];rescued=0
        if len(failed):
            tick=time.perf_counter();balls=self.tree.query_ball_point(y[failed],self.c['correspondence_distance_m']);query_ms+=(time.perf_counter()-tick)*1000
            flat=np.concatenate([np.asarray(v,dtype=int) for v in balls]) if any(len(v) for v in balls) else np.empty(0,dtype=int)
            tick=time.perf_counter();new+=self.warm_normals(flat);normal_ms+=(time.perf_counter()-tick)*1000
            for i,candidates in zip(failed,balls):
                candidates=np.asarray(candidates,dtype=int);v=np.sort(candidates[self.valid[candidates]])
                if len(v):
                    ds=np.linalg.norm(self.target[v]-y[i],axis=1);chosen=int(np.argmin(ds))
                    ids[i]=v[chosen];distance[i]=ds[chosen];rescued+=1
        use=inside[self.valid[ids[inside]]];tid=ids[use];p=source[use]
        if len(use):
            _,inv,counts=np.unique(tid,return_inverse=True,return_counts=True);w=1./counts[inv]/self.c['lidar_sigma_m']**2
            blocks=len(np.unique(np.floor(p/self.c['source_block_m']).astype(int),axis=0))
        else:w=np.empty(0);blocks=0
        metrics={'source_n':len(source),'support_n':len(use),'unique_targets':len(np.unique(tid)),
            'source_blocks':blocks,'fraction':len(use)/max(1,len(source)),'new_normal_n':new,
            'query_ms':query_ms,'normal_ms':normal_ms,'correspondence_ms':(time.perf_counter()-start)*1000,
            'raw_nearest_invalid_count':len(failed),'nearest_valid_rescued_count':rescued}
        good=len(use)>=self.c['minimum_support_points'] and metrics['unique_targets']>=self.c['minimum_unique_targets'] and blocks>=self.c['minimum_source_blocks'] and metrics['fraction']>=self.c['minimum_support_fraction']
        return {'p':p,'q':self.target[tid],'n':self.normals[tid],'w':w,'source_index':use,'target_ids':tid,'good':bool(good),'metrics':metrics}
