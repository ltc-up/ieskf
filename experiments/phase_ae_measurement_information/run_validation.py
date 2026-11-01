"""Spatially disjoint train-only fits and explicit contaminated root diagnostics."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from case_tools import *
from geometry_ieskf.density_audit import FrozenNormalsView,block_partition,density_arms
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output);V=O/'validation';V.mkdir(exist_ok=True)
source,prior,P,factor,reference,roots,ad=load_case(D);T0=factor.transform(prior);predicted_y=source@T0[:3,:3].T+T0[:3,3];st,sv=block_partition(predicted_y);mt,mv=block_partition(reference.target);np.savez_compressed(V/'partition_ids.npz',source_train=st,source_validation=sv,target_train=mt,target_validation=mv)
assert not set(st)&set(sv) and not set(mt)&set(mv)
qtrain=NearestValidCorrespondence(reference.target[mt]);qval=NearestValidCorrespondence(reference.target[mv]);gap=float(cKDTree(qtrain.target).query(qval.target)[0].min());assert gap>2*C['normal_radius_m']
print('fold sizes',len(st),len(sv),len(mt),len(mv),'minimum_target_gap',gap,flush=True)
# Whole validation/training plane banks are computed independently, in bounded batches.
for q in [qtrain,qval]:
 for j in range(0,len(q.target),5000):q.warm_normals(np.arange(j,min(j+5000,len(q.target))))
validtrees={};validids={}
for name,q in [('train',qtrain),('validation',qval)]:
 validids[name]=np.flatnonzero(q.valid);validtrees[name]=cKDTree(q.target[validids[name]])

def score(z,kind):
 indices=st if kind=='train' else sv;q=qtrain if kind=='train' else qval
 T=factor.transform(inject(prior,z));y=source[indices]@T[:3,:3].T+T[:3,3]
 if not len(y) or len(validids[kind])==0:return {'status':'NO_VALIDATION_SUPPORT','n':len(y)}
 distance,local=validtrees[kind].query(y);ids=validids[kind][local];res=np.einsum('ni,ni->n',q.normals[ids],y-q.target[ids]);inside=distance<=C['correspondence_distance_m'];cells=np.floor(predicted_y[indices,:2]/20).astype(int);uc,inv=np.unique(cells,axis=0,return_inverse=True);blocks=[]
 for j in range(len(uc)):
  mask=inv==j;blocks.append({'ordinal':j,'n':int(mask.sum()),'absolute_residual_p50_p90_p99_max':np.percentile(abs(res[mask]),[50,90,99,100]),'distance_p50_p90_p99_max':np.percentile(distance[mask],[50,90,99,100]),'inside_original_radius':int(inside[mask].sum())})
 return {'status':'SCORED_UNTRUNCATED','n':len(y),'absolute_residual_p50_p90_p99_max':np.percentile(abs(res),[50,90,99,100]),'rms':float(np.sqrt(np.mean(res**2))),'distance_p50_p90_p99_max':np.percentile(distance,[50,90,99,100]),'inside_original_radius':int(inside.sum()),'supported_abs_residual_p50_p90':np.percentile(abs(res[inside]),[50,90]) if inside.any() else [],'blocks':blocks,'private_target_ids':ids,'private_residuals':res,'private_distances':distance}
# Build density choices using ONLY training data and original13 seeds; not AD roots.
pool=[]
for s in starting_seeds(P):
 T=factor.transform(inject(prior,s['z']));y=source[st]@T[:3,:3].T+T[:3,3];balls=qtrain.tree.query_ball_point(y,C['correspondence_distance_m']);pool.extend(v for v in balls if len(v))
pool=np.unique(np.concatenate(pool)) if pool else np.array([],int);arms,definition=density_arms(qtrain.target,pool,C['source_block_m']);np.savez_compressed(V/'train_density_ids.npz',**arms);dump(V/'train_density_definition.json',definition)
results=[]
for arm in ['A','B','C','D']:
 q=qtrain if arm=='A' else FrozenNormalsView(qtrain,arms[arm]);o=CompactHuberMixture(prior,P,source[st],factor,q);r=run_fit(o,{'id':'nominal','z':np.zeros(15)},V,'train_'+arm);z=np.array(r['final']['z']);results.append({'arm':arm,'fit':r,'train_score':score(z,'train'),'validation_score':score(z,'validation')})
# Actual plane neighborhoods sampled by held-out validation and train fits use disjoint global IDs.
train_targets=[];val_targets=[]
for row in results:
 z=np.array(row['fit']['final']['z']);ts=row['train_score'].get('private_target_ids',[]);vs=row['validation_score'].get('private_target_ids',[])
 if len(ts):train_targets.extend(mt[qtrain.tree.query(qtrain.target[np.unique(ts)],k=C['normal_neighbors'])[1]].ravel().tolist())
 if len(vs):val_targets.extend(mv[qval.tree.query(qval.target[np.unique(vs)],k=C['normal_neighbors'])[1]].ravel().tolist())
inter=np.intersect1d(train_targets,val_targets);assert len(inter)==0
posthoc=[]
for label,z in roots.items():posthoc.append({'label':label,'independence':'CONTAMINATED_AD_ALL_DATA_FIT','train_score':score(z,'train'),'validation_score':score(z,'validation')})
# If full-data density fits have finished, score every existing endpoint on exactly this same excluded geometry (still contaminated).
for file in sorted((O/'fixed').glob('*.json')):
 if file.name.endswith('_frozen.json'):continue
 r=json.load(open(file));posthoc.append({'label':file.stem,'independence':'CONTAMINATED_FULL_DATA_FIT','validation_score':score(np.array(r['final']['z']),'validation')})
dump(V/'validation_results.json',{'source_train_n':len(st),'source_validation_n':len(sv),'map_train_n':len(mt),'map_validation_n':len(mv),'map_center_min_gap_m':gap,'normal_neighbor_overlap_n':len(inter),'train_valid_planes_n':len(validids['train']),'validation_valid_planes_n':len(validids['validation']),'train_fits':results,'posthoc':posthoc,'independent_absolute_reference':False})
print('validation complete; train fits',len(results),'posthoc',len(posthoc),flush=True)
