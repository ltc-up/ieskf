"""Plot saved scalar results only."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main(a):
 s=json.loads(Path(a.results).read_text());rows=s['results'];fig,axes=plt.subplots(2,5,figsize=(15,6),sharex=True)
 for j,frame in enumerate([60,64,65,66,76]):
  for method,color in [('space','tab:blue'),('random','tab:orange'),('full','black')]:
   rs=[r for r in rows if r['frame']==frame and r['method']==method and (method!='space' or r['stage']=='A')];rs.sort(key=lambda r:r['n'])
   for ax,key in [(axes[0,j],'final_long_m'),(axes[1,j],'final_yaw_deg')]:ax.plot([r['n'] for r in rs],[r[key] for r in rs],'.-',color=color,label=method)
  sample=next(r for r in rows if r['frame']==frame)
  axes[0,j].axhline(sample['seed_long_m'],ls='--',color='gray',label='seed');axes[1,j].axhline(sample['seed_yaw_deg'],ls='--',color='gray');axes[0,j].set_title(f'Frame {frame}')
  for ax in axes[:,j]:ax.set_xscale('log');ax.grid(alpha=.3)
  axes[1,j].set_xlabel('Source points (log scale)')
 axes[0,0].set_ylabel('Longitudinal reference error (m)');axes[1,0].set_ylabel('Yaw reference error (deg)');axes[0,0].legend(fontsize=8);fig.suptitle('P4: fixed seeds/configuration; first spatial seed; reference-assisted diagnostics only');fig.tight_layout();fig.savefig(a.out,dpi=140);plt.close(fig)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--results',required=True);p.add_argument('--out',required=True);main(p.parse_args())
