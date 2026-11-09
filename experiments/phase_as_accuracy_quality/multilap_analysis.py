"""Progress-aligned repeatability is not pure localization error."""
import argparse,json
from pathlib import Path
import numpy as np
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for k in ['evaluation','laps','run','private','output','figures']:p.add_argument('--'+k,type=Path,required=True)
 a=p.parse_args();a.figures.mkdir(parents=True,exist_ok=True);ev=np.load(a.evaluation);laps=json.loads(a.laps.read_text());tx=json.loads((a.run/'transactions.json').read_text());s=ev['s'];ns=ev['time_ns'];rows=[];profiles=[];bprofiles=[]
 # Shared map centerline lateral coordinate from tangent and closest reference geometry.
 from ndt_global.track_evaluation import Polyline
 # Reconstruction uses fixed anchor errors plus reference offset; passed saved A/B/G world arrays.
 # Centerline coordinates are separately supplied through frozen projection in evaluate_reference.
 for lapid in laps['selected_ids']:
  lap=laps['all_laps'][lapid];m=(ns>=lap['start_ns'])&(ns<lap['end_ns']);good=m&ev['valid'];native=[v for v in tx if lap['start_ns']<=v['ref']<lap['end_ns']];bins=np.floor(s/2).astype(int);n=int(np.max(bins))+1;prof=np.full(n,np.nan);bp=np.full(n,np.nan)
  for k in np.unique(bins[m]):
   ix=m&(bins==k);prof[k]=np.mean(ev['track_d_A'][ix]);bp[k]=np.mean(ev['track_d_B'][ix])
  profiles.append(prof);bprofiles.append(bp);rows.append(dict(lap_id=lapid,frames=len(native),accepted=sum(v['update']['accepted'] for v in native),reference_valid=int(sum(good)),A_abs_lateral_P95=float(np.percentile(abs(ev['ad'][good]),95)) if good.any() else None,B_abs_lateral_P95=float(np.percentile(abs(ev['bd'][good]),95)) if good.any() else None,estimated_center_outside=int(sum(ev['ma'][m]<0))))
 P=np.array(profiles);B=np.array(bprofiles);common=np.isfinite(P).all(axis=0)&np.isfinite(B).all(axis=0);spread=np.ptp(P[:,common],axis=0);bspread=np.ptp(B[:,common],axis=0);out=dict(laps=rows,progress_bin_m=2,common_bins=int(sum(common)),A_cross_lap_lateral_range_P50_P95_max=np.percentile(spread,[50,95,100]).tolist(),B_cross_lap_lateral_range_P50_P95_max=np.percentile(bspread,[50,95,100]).tolist(),interpretation='repeatability plus real driving-line differences; not independent localization error',continuous_single_initialization=True)
 import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
 fig,ax=plt.subplots(2,1,figsize=(10,7));grid=np.flatnonzero(common)/len(common)
 for i in range(len(P)):ax[0].plot(grid,P[i,common],label='lap'+str(i+1));ax[1].plot(grid,B[i,common],label='lap'+str(i+1))
 ax[0].set_title('15D centerline lateral coordinate; 2m bins');ax[1].set_title('production; includes actual driving differences');ax[1].set_xlabel('normalized track progress');ax[0].legend();fig.tight_layout();fig.savefig(a.figures/'multilap_lateral.png');plt.close(fig)
 fig,ax=plt.subplots(figsize=(8,9))
 for lapid in laps['selected_ids']:
  l=laps['all_laps'][lapid];m=(ns>=l['start_ns'])&(ns<l['end_ns']);ax.plot(ev['A'][m,0],ev['A'][m,1],label=str(lapid))
 ax.axis('equal');ax.legend();fig.savefig(a.private/'multilap_overlay.png');plt.close(fig);a.output.write_text(json.dumps(out,indent=2)+'\n')
