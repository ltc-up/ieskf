"""Static, exportable research figures; no online imports or fitted alignment."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from log_io import open_text
from provenance import require_run_output


def plot(run):
    run=Path(run);cfg=json.loads((run/'config.frozen.json').read_text());origin=cfg['start_ns']-130*10**9
    names=['B0']+['KINEMATIC_BK__'+x for x in ('NONE','PREVIOUS','ROLLING','KEYFRAME','KEYFRAME_MR')]
    labels=['Legacy B0','Kinematic only','Previous scan','Rolling','Keyframe','Keyframe multires.']
    times=[];xy={n:[] for n in names}
    for line in open_text(run/'kinematic_evaluation/errors.jsonl'):
        r=json.loads(line);times.append(r['bag_s']);data=r['by_lag'].get('0.075',{})
        for n in names:xy[n].append(data.get(n+'/final',{}).get('xy',np.nan))
    early={n:[] for n in names[1:]};t0=[];first={};G=None
    for line in open_text(run/'kinematic_events.jsonl'):
        r=json.loads(line)
        if r['status']!='COMMITTED':continue
        t=(r['ref_ns']-origin)*1e-9
        if t>140:break
        t0.append(t)
        if G is None:G=np.asarray(r['T_map_from_O'])
        for n in names[1:]:
            z=float((G@np.asarray(r['variants'][n]['O_T_base']))[2,3]);first.setdefault(n,z);early[n].append(z-first[n])
    summary=json.loads((run/'kinematic_relative_evaluation/summary.json').read_text())
    selected={(r['window'],r['variant']):r['metrics']['xy']['rmse'] for r in summary['summary'] if r['lag']=='0.075' and r['label']=='final' and r['horizon']=='previous'}
    fig,ax=plt.subplots(1,3,figsize=(15,4.2));colors=plt.cm.tab10(np.arange(len(names)))
    for i,(n,label) in enumerate(zip(names,labels)):
        ax[0].plot(times,xy[n],lw=.85,color=colors[i],label=label)
        if n in early:ax[1].plot(t0,early[n],lw=1,color=colors[i],label=label)
    ax[0].set(yscale='log',xlabel='Bag time (s)',ylabel='XY discrepancy (m)',title='Continuous local chains: engineering reference')
    ax[1].set(xlabel='Bag time (s)',ylabel='Map-axis height change from first scan (m)',title='Early height drift: no reference datum')
    windows=['all','normal8','right3','fault515_525'];xx=np.arange(len(windows));width=.12
    for i,(n,label) in enumerate(zip(names,labels)):
        ax[2].bar(xx+(i-2.5)*width,[selected.get((w,n),np.nan) for w in windows],width,color=colors[i],label=label)
    ax[2].set(yscale='log',xticks=xx,xticklabels=['All','Normal','Right turn','515–525s'],ylabel='Strict adjacent XY RMSE (m)',title='Prediction improvement is not matching gain')
    for a in ax:a.grid(True,alpha=.2)
    ax[0].legend(fontsize=7);fig.suptitle(f"KINEMATIC_BK map {cfg['map']}: full causal serial replay; reference is not independent truth",fontsize=11)
    fig.tight_layout();out=run/'kinematic_evaluation';require_run_output(out)
    for suffix in ('png','pdf'):fig.savefig(out/f'kinematic_diagnostics.{suffix}',dpi=180,bbox_inches='tight')
    plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');plot(p.parse_args().run)
