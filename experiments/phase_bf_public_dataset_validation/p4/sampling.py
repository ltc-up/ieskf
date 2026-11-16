"""Source-only nested samplers. No map, pose or reference input."""
import time
import numpy as np
COUNTS=(3000,5000,10000,20000)
SPACE_SEEDS=(2026111501,2026111502,2026111503)
RANDOM_SEED=2026111599
RADIAL_EDGES=np.array([10.,20.,40.,80.])
HEIGHT_EDGES=np.array([-2.,-1.,0.,1.,3.])
def strata(points):
 p=np.asarray(points);angle=np.mod(np.arctan2(p[:,1],p[:,0]),2*np.pi);sector=np.minimum(35,(angle/(2*np.pi)*36).astype(int));rad=np.searchsorted(RADIAL_EDGES,np.linalg.norm(p[:,:2],axis=1),side='right');height=np.searchsorted(HEIGHT_EDGES,p[:,2],side='right')
 return (sector*5+rad)*6+height
def space_order(points,seed):
 p=np.asarray(points);assert p.ndim==2 and p.shape[1]==3 and np.isfinite(p).all();n=len(p);rng=np.random.default_rng(seed);s=strata(p)
 keys=np.column_stack([s,np.floor(p).astype(np.int64)]);_,vi,counts=np.unique(keys,axis=0,return_inverse=True,return_counts=True)
 voxel_priority=rng.random(len(counts));point_tie=rng.random(n);o=np.lexsort((point_tie,vi));starts=np.r_[0,np.cumsum(counts)[:-1]];within=np.empty(n,dtype=np.int64);within[o]=np.arange(n)-np.repeat(starts,counts)
 q=np.lexsort((voxel_priority[vi],within,s));_,si,sc=np.unique(s,return_inverse=True,return_counts=True);ss=np.r_[0,np.cumsum(sc)[:-1]];within_s=np.empty(n,dtype=np.int64);within_s[q]=np.arange(n)-np.repeat(ss,sc)
 priority=rng.random(len(sc));order=np.lexsort((priority[si],within_s));anchors=np.unique(np.r_[np.argmin(p,axis=0),np.argmax(p,axis=0)]);mask=np.ones(n,dtype=bool);mask[anchors]=False
 return np.r_[anchors,order[mask[order]]]
def random_order(points,seed):return np.random.default_rng(seed).permutation(len(points))
def quality(points):
 p=np.asarray(points);rad=np.linalg.norm(p[:,:2],axis=1);s=strata(p);angle=np.mod(np.arctan2(p[:,1],p[:,0]),2*np.pi);sectors=np.minimum(35,(angle/(2*np.pi)*36).astype(int))
 return dict(count=len(p),bbox_min=p.min(axis=0).tolist(),bbox_max=p.max(axis=0).tolist(),xyz_span=np.ptp(p,axis=0).tolist(),radial_bin_counts=np.bincount(np.searchsorted(RADIAL_EDGES,rad,side='right'),minlength=5).tolist(),angle_sector_counts=np.bincount(sectors,minlength=36).tolist(),height_bin_counts=np.bincount(np.searchsorted(HEIGHT_EDGES,p[:,2],side='right'),minlength=6).tolist(),occupied_strata=len(np.unique(s)),occupied_angle_sectors=len(np.unique(sectors)),occupied_voxels_1m=len(np.unique(np.floor(p).astype(np.int64),axis=0)),occupied_voxels_1_5m=len(np.unique(np.floor(p/1.5).astype(np.int64),axis=0)),near_lt10_fraction=float(np.mean(rad<10)),far_ge40_fraction=float(np.mean(rad>=40)))
