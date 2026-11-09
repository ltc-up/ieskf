"""Supplement frozen native-time corner diagnostics and private plots, no fit."""
import argparse,json,csv
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from ndt_global.track_evaluation import paired_errors
from ndt_global.footprint_audit import contains,segments_intersect
from analyze import stats
p=argparse.ArgumentParser();p.add_argument('--private',type=Path,required=True);p.add_argument('--run',type=Path,required=True);p.add_argument('--gnss',type=Path,required=True);p.add_argument('--quality',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--left',type=Path,required=True);p.add_argument('--right',type=Path,required=True);a=p.parse_args();z=np.load(a.private/'causal.npz');tx=json.loads((a.run/'transactions.json').read_text());q={int(x['ref']):x for x in map(json.loads,a.quality.open())};ref=np.array([x['ref'] for x in tx]);c=list(csv.DictReader(a.gnss.open()));origin=ref[0]*1e-9;gt=np.array([float(x['timestamp'])-origin for x in c]);rq=Rotation.from_quat([[float(x['base_q'+k]) for k in 'xyzw'] for x in c]);gR=Slerp(gt,rq)(np.clip(z['t'],gt[0],gt[-1]));pr=Rotation.from_matrix([x['prior']['R'] for x in tx]);po=pr*Rotation.from_rotvec(z['delta'][:,6:9]);nr=Rotation.from_matrix(np.array([x['ndt']['final'] for x in tx])[:,:3,:3]);out={}
import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
for k in [1,3,9]:
 m=(z['corner']==k)&z['valid'];ii=np.flatnonzero(m);quality=[q[int(ref[i])] for i in ii if int(ref[i]) in q];summary={}
 for field in ['fitness','support','rank','geo_lat','geo_long','geo_eigen_ratio','innovation_metric']:
  summary[field]=stats([r[field] for r in quality])
 yerr={n:np.rad2deg(np.angle(np.exp(1j*(R.as_euler('xyz')[:,2]-gR.as_euler('xyz')[:,2])))) for n,R in [('prior',pr),('NDT',nr),('posterior',po)]}
 out[str(k)]=dict(quality_joined=len(quality),quality=summary,yaw_GNSS_INS_difference_deg={n:stats(e[m]) for n,e in yerr.items()},causal_label='prediction discrepancy present before current NDT; physical source UNKNOWN')
 fig,ax=plt.subplots(3,1,figsize=(10,8));
 for n,e in yerr.items():ax[0].plot(z['t'][m],e[m],label=n)
 ax[0].set_ylabel('yaw reference difference deg');ax[0].legend();ax[1].plot(z['t'][m],z['innovation'][m,:3]);ax[1].set_ylabel('NDT XYZ innovation m');ax[2].plot(z['t'][m],z['delta'][m,9:15]);ax[2].set_ylabel('bias update (mixed units)');fig.tight_layout();fig.savefig(a.private/f'corner_{k}_yaw_innovation_bias.png');plt.close(fig)
def ring(path):
 d=np.genfromtxt(path,delimiter=',',names=True);return np.c_[d['X'],d['Y']]
L=ring(a.left);R=ring(a.right)
# Remove duplicated final vertex before testing self intersections.
L=L[:-1] if np.linalg.norm(L[0]-L[-1])<1e-6 else L;R=R[:-1] if np.linalg.norm(R[0]-R[-1])<1e-6 else R
area=lambda p:abs(np.cross(p,np.roll(p,-1,axis=0)).sum());outer,inner=(L,R) if area(L)>area(R) else (R,L)
inter=sum(segments_intersect(x,y,u,v) for x,y in zip(L,np.roll(L,-1,axis=0)) for u,v in zip(R,np.roll(R,-1,axis=0)))
def self_inter(r):
 n=len(r);return sum(segments_intersect(r[i],r[(i+1)%n],r[j],r[(j+1)%n]) for i in range(n) for j in range(i+1,n) if j!=i+1 and (i,j)!=(0,n-1))
out['boundary_topology']=dict(inter_ring_intersections=int(inter),left_self_intersections=int(self_inter(L)),right_self_intersections=int(self_inter(R)),all_inner_vertices_in_outer=all(contains(outer,x) for x in inner))
a.output.write_text(json.dumps(out,indent=2)+'\n')
