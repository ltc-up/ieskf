"""Only deidentified optimization traces from published aggregate results."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--results',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();v=json.load(open(a.results));fig,ax=plt.subplots(2,2,figsize=(11,8),constrained_layout=True)
 for r in v['capture']:
  h=r['outer_history'];x=[p['index']+1 for p in h];label=str(r['side_m'])+' m groups'
  ax[0,0].plot(x,[p['source_plane_jaccard'] for p in h],'.-',label=label)
  ax[0,1].semilogy(x,[max(1e-12,p['local_gn_white_norm']) for p in h],'.--',label=label+' fixed')
  ax[0,1].semilogy(x,[max(1e-12,p['reassociated_gn_white_norm']) for p in h],'-',label=label+' requery')
  ax[1,0].plot(x,[p['source_support_after'] for p in h],'.-',label=label)
  ax[1,1].plot(x,[(p['fixed_factor_cost_start']-p['fixed_factor_cost_end'])/p['fixed_factor_cost_start'] for p in h],'.-',label=label)
 ax[0,0].set(title='Factor retention, not raw nearest-record retention',xlabel='Outer query',ylabel='Source-plane Jaccard',ylim=(0,1.03));ax[0,0].legend(fontsize=8)
 ax[0,1].set(title='Fixed local vs freshly associated optimality',xlabel='Outer query',ylabel='Untruncated GN prior-white norm');ax[0,1].legend(fontsize=7,ncol=2)
 ax[1,0].set(title='Surface support under declared hypotheses',xlabel='Outer query',ylabel='Source residual count');ax[1,0].legend(fontsize=8)
 ax[1,1].set(title='Decrease WITHIN each frozen-factor problem',xlabel='Outer query',ylabel='(local start - local end) / local start');ax[1,1].legend(fontsize=8)
 fig.savefig(Path(a.output),dpi=160)
if __name__=='__main__':main()
