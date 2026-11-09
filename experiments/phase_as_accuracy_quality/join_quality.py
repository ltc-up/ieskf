"""Exact native-time reference association and deidentified offline diagnostics."""
import argparse,csv,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import spearmanr
from ndt_global.track_evaluation import Polyline,paired_errors,interpolate_reference,offline_quality
from ndt_global.validation import static_transform
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for k in ['input','run','gnss','center','quality','laps','private','output','figures']:p.add_argument('--'+k,type=Path,required=True)
 a=p.parse_args();a.private.mkdir(parents=True,exist_ok=True);a.figures.mkdir(parents=True,exist_ok=True);tx=json.loads((a.run/'transactions.json').read_text());q=[json.loads(l) for l in a.quality.read_text().splitlines()];assert len(q)==len(tx);ns=np.array([v['ref'] for v in tx],np.int64);origin=int(ns[0]);t=(ns-origin)*1e-9;g=list(csv.DictReader(a.gnss.open()));gt=np.array([float(v['timestamp'])-origin*1e-9 for v in g]);gv=np.array([int(v['truth_valid']) and float(v['gps_bracket_dt_sec'])<=.6 for v in g],bool);gp=np.array([[float(v['base_link_'+k]) for k in 'xyz'] for v in g]);G,valid=interpolate_reference(gt,gp,gv,t,.100001);c=np.genfromtxt(a.center,delimiter=',',names=True);line=Polyline(np.c_[c['Xref'],c['Yref']]);project=line.project(G[:,:2]);tan=project['tangent'];obs=np.array([v['ndt']['final'] for v in tx]);data=json.loads(a.input.read_text());lever=static_transform(data['transforms'],'base_link','imu_bosch')[:3,3];base=[]
 for v in tx:
  d=np.array(v['update']['delta']);R=np.array(v['prior']['R'])@Rotation.from_rotvec(d[6:9]).as_matrix();base.append(np.array(v['prior']['p'])+d[:3]-R@lever)
 base=np.array(base);es,ed=paired_errors(obs[:,:3,3],G,tan);_,fd=paired_errors(base,G,tan);xy=np.linalg.norm(obs[:,:2,3]-G[:,:2],axis=1);laps=json.loads(a.laps.read_text());cut=laps['all_laps'][laps['selected_ids'][0]]['end_ns'];teststart=laps['all_laps'][laps['selected_ids'][2]]['start_ns'];train=ns<cut;test=ns>=teststart;threshold=float(np.percentile([v['innovation_metric'] for i,v in enumerate(q) if train[i]],99));median=float(np.median([v['fitness'] for i,v in enumerate(q) if train[i]]));labels=[offline_quality(converged=v['converged'],valid=v['valid'],support=v['support'],rank=v['rank'],innovation_flag=v['innovation_metric']>threshold) for v in q]
 corr={}
 for key in ['fitness','iterations','support','geo_eigen_ratio','geo_lat','geo_long','geo_Z','geo_roll','geo_pitch','geo_yaw','innovation_metric','native_lat_sigma','translation_update','velocity_update']:
  x=np.array([v[key] for v in q]);corr[key]=float(spearmanr(x[valid],abs(ed[valid])).statistic)
 low=np.array([v['fitness']<median for v in q]);out=dict(time_alignment='NDT Source native reference; GNSS/base/geometry/update all at identical timestamp',paired_valid=int(sum(valid)),training_frames=int(sum(train)),heldout_frames=int(sum(test)),heldout_reference_valid=int(sum(valid&test)),innovation_99pct_training_threshold=threshold,offline_labels={k:labels.count(k) for k in set(labels)},heldout_labels={k:int(sum(test[i] and labels[i]==k for i in range(len(q)))) for k in set(labels)},peak_correctness='NOT_PROVEN; no wrong-peak labels or multi-start evidence this stage',correlation_Spearman_abs_NDT_lateral_reference_difference=corr,correlation_inference='descriptive only, autocorrelated samples, not detector accuracy',low_fitness_training_median=median,low_fitness_reference_lateral_gt_half_m=int(sum(valid&low&(abs(ed)>.5))),low_fitness_reference_lateral_gt_one_m=int(sum(valid&low&(abs(ed)>1))),low_fitness_reference_XY_gt_one_m=int(sum(valid&low&(xy>1))),false_positive_or_miss_rate='NOT_VALIDATED',current_R_calibrated=False,conditional_geometry_rank6_count=sum(v['rank']==6 for v in q))
 np.savez(a.private/'joined_quality.npz',native_ns=ns,valid=valid,ndt_lateral=ed,fused_lateral=fd,ndt_XY=xy,progress=project['s'],labels=labels)
 import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
 for key,name in [('fitness','fitness_reference'),('geo_lat','geometry_reference'),('translation_update','update_reference')]:
  fig,ax=plt.subplots();ax.scatter(np.array([v[key] for v in q])[valid],abs(ed[valid]),s=3,alpha=.4);ax.set_xlabel(key);ax.set_ylabel('absolute NDT/GPS1 lateral difference (m)');ax.set_title('engineering reference; not labelled peak correctness');fig.tight_layout();fig.savefig(a.figures/(name+'.png'));plt.close(fig)
 a.output.write_text(json.dumps(out,indent=2)+'\n')
