"""Aggregate G2-E private runs; export only de-identified error curves."""
import argparse
import collections
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def metric(values):
    x = np.asarray(values, float)
    return dict(n=len(x), rmse=float(np.sqrt(np.mean(x*x))),
                abs_p95=float(np.percentile(abs(x), 95)),
                abs_max=float(np.max(abs(x))))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--analysis', type=Path, required=True)
    p.add_argument('--reference', type=Path, required=True)
    p.add_argument('--run', action='append', nargs=2, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    base = json.loads((a.analysis/'summary.json').read_text())
    rows = list(csv.DictReader((a.analysis/'per_frame.csv').open()))
    ref = np.load(a.reference)
    valid = ref['raw_valid_0.075'].astype(bool)
    corner = ref['corner']
    index = np.arange(len(valid))
    out = dict(reference='75ms raw GNSS engineering reference, not independent truth',
               gate='6D chi2 .999 22.458; marginal 3D 16.266 diagnostic only; translation 5m, rotation 5deg', runs={})
    plot_groups = [('normal8', valid & (corner == 8)),
                   ('bend_800_900', valid & (index >= 800) & (index < 900)),
                   ('bend_1254_1405', valid & (index >= 1254) & (index < 1406))]
    fig, axes = plt.subplots(3, 2, figsize=(11, 10))
    for name, path in a.run:
        run_rows = [r for r in rows if r['run'] == name]
        assert len(run_rows) == len(valid)
        h = [json.loads(line) for line in (Path(path)/'g2e_health.jsonl').open()]
        assert len(h) == len(valid)
        summary = json.loads(Path(str(path)+'-summary.json').read_text())
        tx = json.loads((Path(path)/'transactions.json').read_text())
        yaw_innovation = [abs(np.rad2deg(r['update']['innovation'][5])) for r in tx
                          if r['update']['accepted']]
        yaw_delta = [abs(np.rad2deg(r['update']['delta'][8])) for r in tx
                     if r['update']['accepted']]
        def pct(x): return dict(median=float(np.median(x)), p95=float(np.percentile(x,95)))
        item = dict(accepted=base['runs'][name]['accepted'], rejected=base['runs'][name]['rejected'],
                    ranges=base['runs'][name]['ranges'], stages=base['runs'][name]['stages'],
                    health_reasons=dict(collections.Counter(r['reason'] for r in h)),
                    health_states=dict(collections.Counter(r['state'] for r in h)),
                    max_nis6=max((r.get('nis6', 0.) for r in h)),
                    nis6=pct([r['nis6'] for r in h if 'nis6' in r]),
                    yaw_innovation_abs_deg=pct(yaw_innovation),
                    yaw_update_abs_deg=pct(yaw_delta),
                    R_factors=dict(collections.Counter(str(r.get('R_factor',1.)) for r in h)),
                    max_observation_gap_s=summary['max_accepted_gap_s'],
                    p99_wall_ms=summary['wall_ms']['p99'])
        item['normal8'] = {}
        for field in ('post_lateral', 'post_along'):
            x = np.asarray([float(r[field]) for r in run_rows])
            item['normal8'][field] = metric(x[valid & (corner == 8)])
        out['runs'][name] = item
        for k, (label, mask) in enumerate(plot_groups):
            for c, field in enumerate(('post_lateral', 'post_along')):
                values = np.asarray([float(r[field]) for r in run_rows])
                if label == 'normal8':
                    ix = np.flatnonzero(corner == 8)
                    groups = np.split(ix, np.flatnonzero(np.diff(ix) > 1)+1)
                    curves = [[np.mean(z) for z in np.array_split(abs(values[g][valid[g]]),10)]
                              for g in groups]
                    curve = np.mean(curves, axis=0)
                else:
                    curve = [np.mean(z) for z in np.array_split(abs(values[mask]),10)]
                axes[k,c].plot(np.arange(1,11), curve, marker='o', label=name)
                axes[k,c].set(title=label+' '+field, xlabel='normalized decile', ylabel='mean absolute error m')
                axes[k,c].grid(alpha=.25)
    axes[0,0].legend()
    fig.tight_layout()
    fig.savefig(a.output/'error_curves.png', dpi=150)
    plt.close(fig)
    (a.output/'aggregate.json').write_text(json.dumps(out, indent=2)+'\n')


if __name__ == '__main__': main()
