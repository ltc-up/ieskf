"""Paired engineering-reference comparison of full G2 closed-loop trajectories."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

def load_run(path, lever):
    states = json.loads((path / 'states.json').read_text())
    tx = json.loads((path / 'transactions.json').read_text())
    p = np.asarray([s['p'] for s in states])
    rot = np.asarray([s['R'] for s in states])
    base = p - np.einsum('nij,j->ni', rot, lever)
    return tx, base

def stats(x):
    x = np.asarray(x)
    return dict(n=len(x), rmse=float(np.sqrt(np.mean(x*x))), p50=float(np.percentile(x,50)),
                p95=float(np.percentile(x,95)), p99=float(np.percentile(x,99)), max=float(np.max(x)))

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--candidate', type=Path, required=True)
    p.add_argument('--legacy', type=Path, required=True)
    p.add_argument('--fixed', type=Path, required=True)
    p.add_argument('--reference', type=Path, required=True)
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=True)
    from ndt_global.validation import static_transform
    motion=json.loads(a.input.read_text())
    lever=static_transform(motion['transforms'],'base_link','imu_bosch')[:3,3]
    runs={name:load_run(path,lever) for name,path in [('production_gicp',a.candidate),('ndt_legacy',a.legacy),('ndt_fixed',a.fixed)]}
    ref=np.load(a.reference)
    header=ref['header']; G=ref['G_0.075']; tangent=ref['tangent']; mask=ref['historical_valid_0.075'].astype(bool)
    rows=[]; report={}
    for name,(tx,base) in runs.items():
        assert len(tx)==len(header)==len(base) and np.array_equal([r['ref'] for r in tx],header)
        delta=base-G; along=np.einsum('ij,ij->i',delta[:,:2],tangent)
        lateral=np.cross(tangent,delta[:,:2]); xy=np.linalg.norm(delta[:,:2],axis=1)
        accepted=np.asarray([r['update']['accepted'] for r in tx])
        report[name]=dict(frames=len(tx),accepted=int(accepted.sum()),rejected=int((~accepted).sum()),
                          xy=stats(xy[mask]),abs_lateral=stats(abs(lateral[mask])),
                          signed_lateral_mean=float(lateral[mask].mean()),
                          abs_longitudinal=stats(abs(along[mask])),
                          max_accept_gap_s=float(max(np.diff(header[accepted])*1e-9)) if accepted.sum()>1 else None,
                          matcher_align_ms=stats(np.asarray([r['ndt']['align_wall_ms'] for r in tx])))
        for i in range(len(tx)):
            rows.append(dict(backend=name,frame=i,ref_ns=int(header[i]),reference_valid=int(mask[i]),
                             x=float(base[i,0]),y=float(base[i,1]),xy_error_m=float(xy[i]),
                             signed_lateral_m=float(lateral[i]),longitudinal_m=float(along[i]),
                             accepted=int(accepted[i]),reason=tx[i]['update']['reason']))
    with (a.output/'per_frame.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    (a.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    fig,ax=plt.subplots(figsize=(10,10))
    ax.plot(G[mask,0],G[mask,1],'.',ms=1,label='temporary GNSS reference')
    for name,(_,base) in runs.items():ax.plot(base[:,0],base[:,1],lw=.8,label=name)
    ax.set_aspect('equal');ax.legend();fig.tight_layout();fig.savefig(a.output/'trajectory.png',dpi=160)
    plt.close(fig)
    fig,ax=plt.subplots(figsize=(14,5))
    for name,(_,base) in runs.items():
        lateral=np.cross(tangent,(base-G)[:,:2]);ax.plot(np.flatnonzero(mask),lateral[mask],lw=.7,label=name)
    ax.set_ylabel('signed lateral error, m');ax.set_xlabel('frame');ax.legend();fig.tight_layout()
    fig.savefig(a.output/'lateral_error.png',dpi=160);plt.close(fig)

if __name__=='__main__':main()
