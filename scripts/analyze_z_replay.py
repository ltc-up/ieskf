"""Separate proposed, committed and replayed vertical motion; no new acceptance gate."""
import argparse,json,csv,collections
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def analyze(path):
 r=json.load(open(path));rows=[]
 for e in r['lidar_updates']:
  b=e['predicted'];a=e['corrected'];P=np.array(b['P']);Q=np.array(a['P']);trace=e.get('solver_trace',{});props=trace.get('proposals',[]);queries=trace.get('queries',[])
  raw=np.array(props[-1]['absolute_frozen_delta']) if props else np.zeros(15)
  sets=[set(q['target_ids']) for q in queries];turn=[1-len(x&y)/max(1,len(x|y)) for x,y in zip(sets,sets[1:])]
  rows.append({'time_ns':e['header_ns'],'accepted':e['accepted'],'reason':e['reason'],'true_converged':e['reason']=='CONVERGED','predicted_z':b['p_imu_map'][2],'corrected_z':a['p_imu_map'][2],'accepted_delta_z':a['p_imu_map'][2]-b['p_imu_map'][2],'proposed_delta_z':float(raw[2]),'predicted_vz':b['v_imu_map'][2],'replayed_vz':a['v_imu_map'][2],'accepted_delta_vz':a['v_imu_map'][2]-b['v_imu_map'][2],'proposed_delta_vz':float(raw[5]),'sigma_z_before':np.sqrt(P[2,2]),'sigma_z_after':np.sqrt(Q[2,2]),'sigma_vz_before':np.sqrt(P[5,5]),'sigma_vz_after':np.sqrt(Q[5,5]),'P_vz_Z':P[5,2],'rho_Z_vz':P[5,2]/np.sqrt(P[5,5]*P[2,2]),'K_vz_Z':P[5,2]/P[2,2],'target_set_turnover_max':max(turn,default=0.),'support_n':e.get('support',{}).get('support_n',0),'residual_p90_initial':queries[0]['abs_residual_quantiles_m'][1] if queries and queries[0]['abs_residual_quantiles_m'] else None,'residual_p90_final':queries[-1]['abs_residual_quantiles_m'][1] if queries and queries[-1]['abs_residual_quantiles_m'] else None,'vertical_proxy_initial':queries[0]['vertical_static_plane_proxy_quantiles_m'] if queries else [],'vertical_proxy_final':queries[-1]['vertical_static_plane_proxy_quantiles_m'] if queries else [],'bg_delta':(np.array(a['bg_body_rad_s'])-np.array(b['bg_body_rad_s'])).tolist(),'ba_delta':(np.array(a['ba_body_m_s2'])-np.array(b['ba_body_m_s2'])).tolist()})
 return r,rows

def distribution(x):return dict(zip(['p0','p50','p90','p95','p99','max'],map(float,np.percentile(x,[0,50,90,95,99,100]))))
def main():
 p=argparse.ArgumentParser();p.add_argument('--new',required=True);p.add_argument('--legacy');p.add_argument('--legacy-frozen');p.add_argument('--output',required=True);a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
 r,rows=analyze(a.new);allsets={'corrected':(r,rows)}
 if a.legacy:allsets['legacy']=analyze(a.legacy)
 summary={}
 for label,(run,rr) in allsets.items():
  with open(out/(label+'_events.csv'),'w') as f:
   w=csv.DictWriter(f,fieldnames=list(rr[0]));w.writeheader();w.writerows(rr)
  summary[label]={'counts':run['counts'],'reasons':dict(collections.Counter(x['reason'] for x in rr)),'accepted_abs_delta_vz':distribution([abs(x['accepted_delta_vz']) for x in rr]),'proposed_abs_delta_vz':distribution([abs(x['proposed_delta_vz']) for x in rr]),'accepted_abs_delta_z':distribution([abs(x['accepted_delta_z']) for x in rr]),'continuous_abs_vz':distribution([abs(x['v_imu_map'][2]) for x in run['outputs']]),'accepted_over_2':sum(abs(x['accepted_delta_vz'])>2 for x in rr),'accepted_over_4':sum(abs(x['accepted_delta_vz'])>4 for x in rr),'proposed_over_2':sum(abs(x['proposed_delta_vz'])>2 for x in rr),'proposed_over_4':sum(abs(x['proposed_delta_vz'])>4 for x in rr),'update_ms':run['diagnostics']['timings_ms']['lidar_update'],'top_accepted_events':sorted(rr,key=lambda x:abs(x['accepted_delta_vz']),reverse=True)[:10],'top_proposed_events':sorted(rr,key=lambda x:abs(x['proposed_delta_vz']),reverse=True)[:10],'first_event':rr[0],'second_event':rr[1],'all_finite':all(x['finite'] for x in run['outputs']),'minimum_P_eigen':min(x['P_eigen_min'] for x in run['outputs'])}
 t0=rows[0]['time_ns'];t=np.array([(e['time_ns']-t0)*1e-9 for e in rows]);get=lambda k:np.array([e[k] for e in rows])
 def figure(name,groups,zoom=False):
  fig,axes=plt.subplots(len(groups),1,figsize=(11,3*len(groups)),sharex=True,squeeze=False)
  for ax,(keys,title) in zip(axes[:,0],groups):
   for k in keys:ax.plot(t,get(k),'.-',ms=2,label=k)
   ax.set_ylabel(title);ax.grid(alpha=.25);ax.legend(fontsize=8)
   if zoom:ax.set_xlim(-.025,.35)
  axes[-1,0].set_xlabel('Seconds from first LiDAR (offline diagnostic)');fig.tight_layout();fig.savefig(out/(name+'.png'),dpi=150);plt.close(fig)
 figure('z_position',[(['predicted_z','corrected_z'],'Map Z [m]'),(['proposed_delta_z','accepted_delta_z'],'Z correction [m]')])
 figure('vertical_velocity',[(['predicted_vz','replayed_vz'],'Map vertical speed [m/s]'),(['proposed_delta_vz','accepted_delta_vz'],'Frozen-prior correction [m/s]')])
 figure('first_events',[(['proposed_delta_vz','accepted_delta_vz'],'Correction [m/s]'),(['sigma_z_before','sigma_z_after'],'sigma Z [m]'),(['rho_Z_vz'],'Correlation'),(['K_vz_Z'],'Coefficient [1/s]')],True)
 figure('covariance',[(['sigma_z_before','sigma_z_after'],'sigma Z [m]'),(['sigma_vz_before','sigma_vz_after'],'sigma vz [m/s]'),(['rho_Z_vz'],'Correlation'),(['K_vz_Z'],'Conditional coefficient [1/s]')])
 fig,ax=plt.subplots(figsize=(11,3));st={'MAX_ITERATIONS':1,'CONVERGED':2,'LINE_SEARCH_OR_TRUST_LIMIT':0,'LOW_SUPPORT':-1};ax.scatter(t,[st.get(e['reason'],-2) for e in rows],s=14);ax.set_yticks([-1,0,1,2],['Low support','Rejected','Partial accepted','Converged']);ax.set_xlabel('Seconds from first LiDAR');fig.tight_layout();fig.savefig(out/'acceptance_timeline.png',dpi=150);plt.close(fig)
 fig,ax=plt.subplots(figsize=(11,4))
 for label,(run,rr) in allsets.items():
  ax.plot([(e['state_time_ns']-t0)*1e-9 for e in run['outputs']],[e['v_imu_map'][2] for e in run['outputs']],label=label)
 ax.set_xlabel('Seconds from corrected first LiDAR');ax.set_ylabel('Actual replayed map vz [m/s]');ax.legend();ax.grid(alpha=.25);fig.tight_layout();fig.savefig(out/'continuous_vz.png',dpi=150);plt.close(fig)
 if a.legacy_frozen:
  old=json.load(open(a.legacy_frozen));pairs=[]
  for new,oldrow in zip(rows,old):
   ps=oldrow['solver_trace']['proposals'];prop=ps[-1]['absolute_frozen_delta'] if ps else [0]*15
   qs=oldrow['solver_trace']['queries'];pairs.append({'time_ns':new['time_ns'],'new_proposed_delta_z':new['proposed_delta_z'],'old_proposed_delta_z':prop[2],'new_proposed_delta_vz':new['proposed_delta_vz'],'old_proposed_delta_vz':prop[5],'new_initial_residual_p90':new['residual_p90_initial'],'old_initial_residual_p90':qs[0]['abs_residual_quantiles_m'][1] if qs[0]['abs_residual_quantiles_m'] else None,'new_vertical_proxy':new['vertical_proxy_initial'],'old_vertical_proxy':qs[0]['vertical_static_plane_proxy_quantiles_m']})
  (out/'frozen_common_prior_comparison.json').write_text(json.dumps(pairs,indent=2));summary['common_prior']={k:distribution([abs(x[k]) for x in pairs if x[k] is not None]) for k in ['new_proposed_delta_z','old_proposed_delta_z','new_proposed_delta_vz','old_proposed_delta_vz','new_initial_residual_p90','old_initial_residual_p90']}
  fig,axes=plt.subplots(2,1,figsize=(11,6),sharex=True)
  for prefix in ['new','old']:
   axes[0].plot(t,[x[prefix+'_proposed_delta_z'] for x in pairs],label=prefix);axes[1].plot(t,[x[prefix+'_proposed_delta_vz'] for x in pairs],label=prefix)
  axes[0].set_ylabel('Frozen proposal delta Z [m]');axes[1].set_ylabel('Frozen proposal delta vz [m/s]');axes[1].set_xlabel('Seconds from first LiDAR');axes[0].legend();fig.tight_layout();fig.savefig(out/'common_prior_source_comparison.png',dpi=150);plt.close(fig)
 (out/'statistics.json').write_text(json.dumps(summary,indent=2));print(json.dumps({k:{q:v for q,v in s.items() if q not in ['top_accepted_events','top_proposed_events','first_event','second_event']} for k,s in summary.items()},indent=2))
if __name__=='__main__':main()
