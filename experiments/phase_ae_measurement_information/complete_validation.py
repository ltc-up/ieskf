"""Complete posthoc scores without repeating any optimization or changing folds."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from case_tools import *
p=argparse.ArgumentParser();p.add_argument('--data-root',required=True);p.add_argument('--output',required=True);a=p.parse_args();D=Path(a.data_root);O=Path(a.output);V=O/'validation';parts=np.load(V/'partition_ids.npz')
source,prior,P,factor,reference,roots,ad=load_case(D);st=parts['source_train'];sv=parts['source_validation'];mt=parts['target_train'];mv=parts['target_validation'];qtrain=NearestValidCorrespondence(reference.target[mt]);qval=NearestValidCorrespondence(reference.target[mv])
for q in [qtrain,qval]:
 for j in range(0,len(q.target),5000):q.warm_normals(np.arange(j,min(j+5000,len(q.target))))
valid=np.flatnonzero(qval.valid);tree=cKDTree(qval.target[valid]);T0=factor.transform(prior);pred_y=source[sv]@T0[:3,:3].T+T0[:3,3];distance,loc=tree.query(pred_y);keep=distance<=C['correspondence_distance_m'];anchor_si=sv[keep];anchor_ti=valid[loc[keep]]
np.savez_compressed(V/'frozen_validation_witness.npz',source_ids=anchor_si,target_ids=anchor_ti,normals=qval.normals[anchor_ti],target=qval.target[anchor_ti]);common_train=CompactHuberMixture(prior,P,source[st],factor,qtrain)
entries=[]
for arm in ['A','B','C','D']:
 r=json.load(open(V/('train_'+arm+'.json')));entries.append(('TRAIN_ONLY_'+arm,np.array(r['final']['z']),r['status']))
for name,z in roots.items():entries.append(('AD_CONTAMINATED_'+name,z,'AD_CERTIFIED'))
for file in sorted((O/'fixed').glob('*.json')):
 if file.name.endswith('_frozen.json'):continue
 r=json.load(open(file));entries.append(('FULL_DATA_CONTAMINATED_'+file.stem,np.array(r['final']['z']),r['status']))
rows=[]
for label,z,status in entries:
 T=factor.transform(inject(prior,z));y=source[sv]@T[:3,:3].T+T[:3,3];dd,ll=tree.query(y);ti=valid[ll];res=np.einsum('ni,ni->n',qval.normals[ti],y-qval.target[ti]);support=dd<=C['correspondence_distance_m']
 ay=source[anchor_si]@T[:3,:3].T+T[:3,3];ar=np.einsum('ni,ni->n',qval.normals[anchor_ti],ay-qval.target[anchor_ti]);adist=np.linalg.norm(ay-qval.target[anchor_ti],axis=1)
 rows.append({'label':label,'optimization_status':status,'training_source_n':len(st),'validation_source_n':len(sv),'common_A_training_energy':common_train.cost(z),'nearest_validation_rms':float(np.sqrt(np.mean(res**2))),'nearest_validation_abs_quantiles':np.percentile(abs(res),[50,90,99,100]),'nearest_center_distance_quantiles':np.percentile(dd,[50,90,99,100]),'nearest_inside_R':int(support.sum()),'supported_nearest_rms':float(np.sqrt(np.mean(res[support]**2))) if support.any() else None,'fixed_witness_n':len(anchor_si),'fixed_witness_rms':float(np.sqrt(np.mean(ar**2))),'fixed_witness_abs_quantiles':np.percentile(abs(ar),[50,90,99,100]),'fixed_witness_current_distance_quantiles':np.percentile(adist,[50,90,99,100]),'fixed_witness_current_inside_R':int(np.sum(adist<=C['correspondence_distance_m'])),'private_nearest_residuals':res,'private_fixed_residuals':ar})
train=rows[:4];pairs=[]
for i in range(4):
 for j in range(i+1,4):
  a,b=train[i],train[j];low,high=(a,b) if a['common_A_training_energy']<b['common_A_training_energy'] else (b,a)
  pairs.append({'lower_train_energy':low['label'],'higher_train_energy':high['label'],'common_train_energy_delta':low['common_A_training_energy']-high['common_A_training_energy'],'fixed_validation_rms_delta':low['fixed_witness_rms']-high['fixed_witness_rms'],'nearest_validation_rms_delta':low['nearest_validation_rms']-high['nearest_validation_rms'],'nearest_support_delta':low['nearest_inside_R']-high['nearest_inside_R']})
dump(V/'completed_validation.json',{'fixed_witness_selection':'original prediction nearest validation-only valid plane inside original radius, no outcome-based selection','rows':rows,'all6_train_pairs':pairs,'reoptimizations':0})
print('completed',len(rows),'fixed witnesses',len(anchor_si));print([{k:r[k] for k in ['label','common_A_training_energy','nearest_validation_rms','fixed_witness_rms','nearest_inside_R']} for r in rows[:6]],flush=True);print('all train-pair tradeoffs',pairs,flush=True)
