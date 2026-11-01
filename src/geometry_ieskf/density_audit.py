"""Pure record-density diagnostics; reuses unchanged AD objective."""
import numpy as np
from .valid_correspondence import NearestValidCorrespondence

class FrozenNormalsView(NearestValidCorrespondence):
    def __init__(self,reference,original_ids):
        self.reference=reference;self.original_ids=np.asarray(original_ids,dtype=int)
        super().__init__(reference.target[self.original_ids],reference.c)
    def warm_normals(self,ids):
        ids=np.unique(ids);missing=ids[~self.cached[ids]]
        if len(missing):
            oi=self.original_ids[missing];self.reference.warm_normals(oi)
            self.normals[missing]=self.reference.normals[oi];self.valid[missing]=self.reference.valid[oi];self.cached[missing]=True
        return len(missing)

def density_arms(target,candidate_ids,cell_size=5.):
    cells=np.floor(target/cell_size).astype(np.int64)
    unique,inv=np.unique(cells,axis=0,return_inverse=True)
    order=np.lexsort((np.arange(len(target)),target[:,2],target[:,1],target[:,0],inv))
    counts=np.bincount(inv,minlength=len(unique));starts=np.r_[0,np.cumsum(counts[:-1])]
    rank=np.empty(len(target),int);rank[order]=np.arange(len(target))-np.repeat(starts,counts)
    half=rank%2==0
    pool=np.unique(candidate_ids);occupied,cnt=np.unique(inv[pool],return_counts=True)
    chosen=occupied[np.lexsort((unique[occupied,2],unique[occupied,1],unique[occupied,0],-cnt))[:int(np.ceil(len(occupied)/4))]]
    dense=np.isin(inv,chosen)
    return {'A':np.arange(len(target)),'B':np.repeat(np.arange(len(target)),2),'C':np.flatnonzero(half),'D':np.flatnonzero(~dense|half)}, {'cell_size':cell_size,'selected_dense_cells':unique[chosen],'candidate_pool_n':len(pool),'dense_original_n':int(dense.sum()),'original_n':len(target),'half_n':int(half.sum())}

def block_partition(points,size=20.,inset=3.5):
    cells=np.floor(points[:,:2]/size).astype(np.int64);inside=points[:,:2]-size*cells
    core=np.all((inside>=inset)&(inside<=size-inset),axis=1)
    val=(cells[:,0]+2*cells[:,1])%3==0
    return np.flatnonzero(core&~val),np.flatnonzero(core&val)
