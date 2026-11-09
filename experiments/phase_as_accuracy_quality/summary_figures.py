"""Deidentified coarse error summaries; no XY coordinates or raw trajectories."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--evaluation',type=Path,required=True);p.add_argument('--multilap-evaluation',type=Path,required=True);p.add_argument('--laps',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);a=p.parse_args();a.output_dir.mkdir(parents=True,exist_ok=True);laps=json.loads(a.laps.read_text());length=laps['track_length_m'];v=np.load(a.evaluation);b=np.floor(v['s']/length*40).astype(int);fig,ax=plt.subplots(3,1,figsize=(10,8))
 for j,(x,y,title) in enumerate([('ad','bd','lateral reference difference'),('as_','bs','longitudinal reference difference'),('ma','mb','center corridor margin')]):
  for k,label in [(x,'15D'),(y,'EKF')]:
   xs=[];ys=[]
   for i in range(40):
    m=(b==i)&(v['valid'] if j<2 else np.ones(len(b),bool))
    if m.any():xs.append((i+.5)/40);ys.append(np.median(v[k][m]))
   ax[j].scatter(xs,ys,s=15,label=label)
  ax[j].set_ylabel(title+' m');ax[j].legend()
 ax[-1].set_xlabel('normalized progress; coarse40-bin medians only');fig.tight_layout();fig.savefig(a.output_dir/'progress_error_summary.png');plt.close(fig)
 e=np.load(a.multilap_evaluation);laps=json.loads(a.laps.read_text());bins=np.floor(e['s']/length*40).astype(int);fig,ax=plt.subplots(2,1,figsize=(10,6))
 for j,key in enumerate(['track_d_A','track_d_B']):
  for lapid in laps['selected_ids']:
   z=laps['all_laps'][lapid];m=(e['time_ns']>=z['start_ns'])&(e['time_ns']<z['end_ns']);xx=[];yy=[]
   for k in range(40):
    t=m&(bins==k)
    if t.any():xx.append((k+.5)/40);yy.append(np.mean(e[key][t]))
   ax[j].plot(xx,yy,label='lap'+str(lapid))
  ax[j].set_ylabel(('15D' if j==0 else 'EKF')+' coarse lateral mean m');ax[j].legend()
 ax[-1].set_xlabel('coarse normalized progress; driving variation included');fig.tight_layout();fig.savefig(a.output_dir/'multilap_lateral.png');plt.close(fig)
