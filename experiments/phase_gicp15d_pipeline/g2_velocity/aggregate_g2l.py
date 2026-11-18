"""Publish only de-identified G2-L aggregates and a normalized segment-8 curve."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--diagnostics', type=Path, required=True)
    p.add_argument('--per-frame', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    d = json.loads(a.diagnostics.read_text())
    rows = list(csv.DictReader(a.per_frame.open()))
    assert len(rows) == 4134
    curves = {key: [] for key in ('seed_lateral', 'match_lateral', 'post_lateral')}
    for lap in d['normal8_laps'].values():
        lo, hi = lap['frame_start'], lap['frame_end']
        for key in curves:
            values = np.array([abs(float(rows[i][key])) for i in range(lo, hi+1)])
            bins = np.array_split(values, 10)
            curves[key].append([float(np.mean(chunk)) for chunk in bins])
    summarized = {key: np.mean(curves[key], axis=0).tolist() for key in curves}
    out = dict(reference=d['reference'], full=d['full'], corners=d['corners'],
               normal8_laps=d['normal8_laps'], peak_800_900=d['peak_800_900'],
               bend_1254_1405=d['bend_1254_1405'],
               normal8_normalized_mean_abs_lateral_10bins=summarized)
    # Strip frame positions so the public result cannot locate private samples.
    for lap in out['normal8_laps'].values():
        lap.pop('frame_start')
        lap.pop('frame_end')
    (a.output / 'aggregate.json').write_text(json.dumps(out, indent=2) + '\n')
    x = np.arange(1, 11)
    fig, ax = plt.subplots(figsize=(8, 4))
    for key, label in [('seed_lateral', 'Seed'), ('match_lateral', 'raw GICP'),
                       ('post_lateral', 'posterior')]:
        ax.plot(x, summarized[key], marker='o', label=label)
    ax.set(xlabel='normal segment 8, normalized decile', ylabel='mean absolute lateral error (m)')
    ax.grid(alpha=.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(a.output / 'normal8_lateral_curve.png', dpi=150)
    plt.close(fig)
    # The short bend exposes the matcher-to-posterior response without map coordinates.
    bend = {}
    for key in curves:
        values = np.array([abs(float(rows[i][key])) for i in range(1254, 1406)])
        bend[key] = [float(np.mean(chunk)) for chunk in np.array_split(values, 10)]
    fig, ax = plt.subplots(figsize=(8, 4))
    for key, label in [('seed_lateral', 'Seed'), ('match_lateral', 'raw GICP'),
                       ('post_lateral', 'posterior')]:
        ax.plot(x, bend[key], marker='o', label=label)
    ax.set(xlabel='1254–1405 bend, normalized decile',
           ylabel='mean absolute lateral error (m)')
    ax.grid(alpha=.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(a.output / 'bend_feedback_curve.png', dpi=150)


if __name__ == '__main__':
    main()
