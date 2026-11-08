"""Deidentified geometry/state/causality checks; plots aggregate metrics only."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from ndt_global.integrity import MapSupport

def main(a):
 ledgers={m:json.loads((a.private/m/'transactions.json').read_text()) for m in 'ABCDE'};A,E=ledgers['A'],ledgers['E'];same=True
 for x,y in zip(A,E):
  for k in ('p','R','P'):
   if not np.array_equal(x['prior'][k],y['prior'][k]):same=False
 target=np.fromfile(a.target,dtype='<f4',offset=8).reshape(-1,3);support=MapSupport(target);zdiag=[]
 for r in ledgers['C']:
  n=r['ndt'];path=a.private/'C'/(str(r['ref'])+'_A1.bin');src=np.fromfile(path,dtype='<f4',offset=8).reshape(-1,3);T=np.array(n['final']);g=support.diagnose(src,T);q=src[np.linspace(0,len(src)-1,min(len(src),1500),dtype=int)]@T[:3,:3].T+T[:3,3];dist,idx=support.tree.query(q);idx=idx[dist<=2];_,near=support.tree.query(target[idx],k=20);patch=target[near];delta=patch-patch.mean(axis=1)[:,None,:];val,V=np.linalg.eigh(np.einsum('nki,nkj->nij',delta,delta)/20);planar=(val[:,1]-val[:,0])/np.maximum(val[:,2],1e-12)>.2;norm=V[planar,:,0];zdiag.append(dict(index=r['index'],converged=n['converged'],accepted=r['update']['accepted'],rank=g.get('rank',0),support_fraction=g.get('nearby_fraction',0),normal_z_squared_mean=float(np.mean(norm[:,2]**2)) if len(norm) else None,horizontal_normal_fraction=float(np.mean(abs(norm[:,2])>.8)) if len(norm) else None,upright_normal_fraction=float(np.mean(abs(norm[:,2])<.2)) if len(norm) else None,Z_ndt_minus_prediction=float(T[2,3]-np.array(r['seed'])[2,3])))
 out=dict(A_E_native_prior_full_P_bitwise_equal=same,real_causal_C_frames=len(zdiag),geometry=zdiag,semantic_ground_or_guardrail_labels='NOT_AVAILABLE',R_calibrated=False,covariance_or_accuracy_claim=False,Source_ref_consistency='native scan-end body reference; historical binding integer fields may carry submicrosecond float quantization',init_equal=all(ledgers[m][0]['prior']==A[0]['prior'] for m in 'BCDE'))
 (a.public/'validation_results.json').write_text(json.dumps(out,indent=2)+'\n')
 import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
 results=[json.loads((a.public/(m+'_results.json')).read_text()) for m in 'ABCDE'];fig,ax=plt.subplots(1,2,figsize=(9,3));ax[0].bar(list('ABCDE'),[r['delayed_15D_updates'] for r in results]);ax[0].set_ylabel('accepted pose updates / 40 raw frames');ax[1].bar(list('ABCDE'),[r['Z_span_m'] for r in results]);ax[1].set_ylabel('model Z range (m), NOT accuracy');fig.tight_layout();fig.savefig(a.public/'comparison.png',dpi=150);plt.close(fig)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--private',type=Path,required=True);p.add_argument('--target',type=Path,required=True);p.add_argument('--public',type=Path,required=True);main(p.parse_args())
