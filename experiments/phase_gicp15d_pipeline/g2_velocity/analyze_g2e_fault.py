"""Private controlled fault comparison; publish aggregate error and recovery only."""
import argparse
import collections
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from analyze_g2v import load
from ndt_global.validation import static_transform


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--reference', type=Path, required=True)
    p.add_argument('--run', action='append', nargs=2, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    data = json.loads(a.input.read_text())
    ref = np.load(a.reference)
    lever = static_transform(data['transforms'], 'base_link', 'imu_bosch')[:3, 3]
    imu_R = static_transform(data['transforms'], 'base_link', 'imu_bosch')[:3, :3]
    imu = [e for e in data['events'] if e['kind'] == 'imu']
    times = np.asarray([e['header_ns'] for e in imu])
    omega = np.asarray([imu_R @ e['omega'] for e in imu])
    n = 900
    tangent = ref['tangent'][:n]
    normal = np.c_[-tangent[:,1], tangent[:,0]]
    valid = ref['raw_valid_0.075'][:n].astype(bool)
    result = dict(fault='one frame +3m x and +4deg yaw at 800; 45 forced missing poses at 820..864', runs={})
    fig, ax = plt.subplots(figsize=(8,4))
    for name, path in a.run:
        path = Path(path)
        run = load(path, ref['G_0.075'][:n], normal, tangent, lever, times, omega)
        tx = json.loads((path/'transactions.json').read_text())
        health = [json.loads(x) for x in (path/'g2e_health.jsonl').open()]
        summary = json.loads(Path(str(path)+'-summary.json').read_text())
        post = run['posterior']
        def xy(lo,hi):
            x = post[lo:hi][valid[lo:hi]]
            return float(np.sqrt(np.mean(np.sum(x*x,axis=1))))
        item = dict(frames=summary['frames'], stop_reason=summary['stop_reason'],
                    injected_frame_accepted=bool(tx[800]['update']['accepted']),
                    injected_frame_reason=tx[800]['update']['reason'],
                    accepted_after_loss=int(run['accepted'][865:900].sum()),
                    first_reaccepted_frame=next((i for i in range(865,900) if run['accepted'][i]),None),
                    max_accepted_gap_s=summary['max_accepted_gap_s'],
                    xy_rmse_before=xy(750,800), xy_at_fault=xy(800,801),
                    xy_rmse_gap=xy(820,865), xy_rmse_after=xy(865,900),
                    health_states=dict(collections.Counter(x['state'] for x in health)),
                    gate_reasons=dict(collections.Counter(x['reason'] for x in health)),
                    R_factors=dict(collections.Counter(str(x.get('R_factor',1.)) for x in health)))
        result['runs'][name]=item
        x=np.linalg.norm(post[800:900],axis=1)
        ax.plot(np.arange(1,11),[float(np.mean(z)) for z in np.array_split(x,10)],marker='o',label=name)
    ax.set(xlabel='fault-window normalized decile',ylabel='mean XY difference to engineering reference m')
    ax.grid(alpha=.25);ax.legend();fig.tight_layout();fig.savefig(a.output/'fault_recovery_curve.png',dpi=150)
    plt.close(fig)
    (a.output/'fault_aggregate.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
