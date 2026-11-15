"""Render existing scalar diagnostics only. No matcher/filter calls."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main(a):
 root=Path(a.analysis);summary=json.loads((root/'summary.json').read_text());fig,axes=plt.subplots(2,3,figsize=(13,7))
 for row in summary['frames']:
  rs=json.loads((root/f"frame_{row['frame']}_trajectory.json").read_text());k=list(range(len(rs)+1));label=str(row['frame']);score=[rs[0]['score_before']]+[x['score_after'] for x in rs];pairs=[rs[0]['pairs_before']]+[x['pairs_after'] for x in rs];long=[rs[0]['longitudinal_error_before']]+[x['longitudinal_error_after'] for x in rs]
  axes[0,0].plot(k,long,label=label);axes[0,1].plot(k,[x/score[0] for x in score],label=label);axes[0,2].plot(k,[x/pairs[0] for x in pairs],label=label)
  axes[1,0].plot(k[1:],[x['accepted_step'] for x in rs],label=label);axes[1,1].semilogy(k[1:],[x['relative_change'] for x in rs],label=label)
 axes[1,0].axhline(.01,color='black',ls='--',lw=1);axes[1,1].axhline(.001,color='black',ls='--',lw=1)
 for ax,title in zip(axes.flat,['Longitudinal reference error (m)','Score / initial score','Valid pairs / initial pairs','Accepted step (parameter norm)','Relative objective change','Align wall time (nested intervals)']):ax.set_title(title);ax.grid(alpha=.25)
 ax=axes[1,2];fs=[str(r['frame']) for r in summary['frames']];pcl=[r['timing']['p2_pcl_fit_ms'] for r in summary['frames']];rest=[r['timing']['align_minus_nested_pcl_ms'] for r in summary['frames']];ax.bar(fs,pcl,label='PCL Fitness wall+sync');ax.bar(fs,rest,bottom=pcl,label='Other align incl. trace');ax.set_ylabel('ms');ax.legend(fontsize=8)
 for ax in list(axes.flat)[:5]:ax.set_xlabel('Iteration');ax.legend(fontsize=8)
 fig.suptitle('BF1-P2: exactly five fixed inputs; diagnostics, not real-time qualification');fig.tight_layout();fig.savefig(a.out,dpi=140);plt.close(fig)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--analysis',required=True);p.add_argument('--out',required=True);main(p.parse_args())
