"""De-identified scientific plots from saved outputs; no estimation or replay."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main(a):
    root=Path(a.root);out=Path(a.out);d=json.loads((root/'results/analysis/comparison.json').read_text());cases=json.loads((root/'results/analysis/cases.json').read_text())
    fig,ax=plt.subplots(2,2,figsize=(10,6.5))
    for j,name in enumerate(['normal8','right3']):
        w=d['windows'][name];c=[v for v in cases if v['window']==name];x=[v['index'] for v in c]
        ax[0,j].plot(x,[v['xy_proxy'] for v in c],'.-',ms=3,label='FAR registered candidate')
        ax[0,j].set_title(name+' (returned scored candidates)');ax[0,j].set_ylabel('XY engineering proxy error [m]');ax[0,j].legend(fontsize=8)
        ax[1,j].plot(x,[v['after_available_ms'] for v in c],'.-',ms=3,label='Input available to LiDAR delivery')
        ax[1,j].axhline(50,color='r',ls='--',label='50 ms');ax[1,j].set_ylabel('Delivery latency [ms]');ax[1,j].set_xlabel('Frozen frame index');ax[1,j].legend(fontsize=8)
        for axis in ax[:,j]:axis.grid(alpha=.25)
    fig.suptitle('Limited-window FAR-LIO: no full-lap or independent-GT qualification',fontsize=11);fig.tight_layout();fig.savefig(out,dpi=140);plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--out',required=True);main(p.parse_args())
