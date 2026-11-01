"""Plot only published relative aggregates, never vehicle trajectories."""
import argparse,json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--results',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();v=json.load(open(a.results));events=[e['alias'] for e in v['events']];x=np.arange(len(events));fig,ax=plt.subplots(2,2,figsize=(11,8),constrained_layout=True)
 for k,g in enumerate(['B','C']):
  p=[next(r for r in v['pointcloud'] if r['alias']==e and r['group']==g) for e in events];n=[next(r for r in v['NDT'] if r['alias']==e and r['group']==g) for e in events];ax[0,0].bar(x+(k-.5)*.3,[r['abs_Z_quantiles_m'][2] for r in p],width=.3,label=g+' minus A');ax[0,1].bar(x+(k-.5)*.3,[r['delta_Z_vs_A_m']*1000 for r in n],width=.3,label=g+' minus A')
 for k,g in enumerate(['A','B','C']):
  u=next(r for r in v['IESKF'] if r['alias']=='E2' and r['group']==g);ax[1,0].bar(k-.15,u['raw_last_PROPOSED_delta_vz_m_s'],width=.3,color='tab:orange',label='Raw proposed' if k==0 else None);ax[1,0].bar(k+.15,u['LEGACY_FROZEN_ACCEPTED_delta_vz_m_s'],width=.3,color='tab:blue',label='Old frozen accepted = 0' if k==0 else None)
  q=next(r for r in v['IESKF'] if r['alias']=='E1' and r['group']==g);ax[1,1].bar(k,q['legacy_conditional_posterior']['sigma_Z_m'],label=g)
 ax[0,0].set(title='Same raw point IDs: Z difference P99',xticks=x,xticklabels=events,ylabel='Absolute Z coordinate difference (m)');ax[0,0].legend()
 ax[0,1].set(title='Full6D CUDA-NDT, same per-event seed',xticks=x,xticklabels=events,ylabel='Matched Z difference (mm)');ax[0,1].legend()
 ax[1,0].set(title='E2 rejected proposal, NOT a velocity trajectory',xticks=[0,1,2],xticklabels=['A','B','C'],ylabel='delta vZ (m/s)');ax[1,0].legend(fontsize=8)
 ax[1,1].set(title='E1 old MAX_ITERATIONS partial posterior',xticks=[0,1,2],xticklabels=['A','B','C'],ylabel='Conditional sigma Z (m)');ax[1,1].text(.5,.9,'Prior sigmaZ = 1.077 m; no strict submissions',transform=ax[1,1].transAxes,ha='center',fontsize=8)
 fig.savefig(a.output,dpi=160)
if __name__=='__main__':main()
