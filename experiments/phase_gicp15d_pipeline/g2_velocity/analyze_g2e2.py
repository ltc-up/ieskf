"""Export only relative, de-identified G2-E2 health and fault aggregates."""
import argparse
import collections
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
from ndt_global.validation import static_transform


def metrics(values):
    values = np.asarray(values, float)
    return dict(n=len(values), rmse=float(np.sqrt(np.mean(values**2))),
                p95=float(np.percentile(np.abs(values), 95)))


def run_metrics(path, ref, truth, valid, normal, tangent, lever):
    tx = json.loads((path/'transactions.json').read_text())
    assert len(tx) <= len(truth)
    errors = []
    for i, row in enumerate(tx):
        assert row['index'] == i and row['ref'] == int(ref[i])
        prior, update = row['prior'], row['update']
        correction = np.asarray(update.get('delta', np.zeros(15)))
        rotation = np.asarray(prior['R']) @ Rotation.from_rotvec(correction[6:9]).as_matrix()
        pose = np.asarray(prior['p']) + correction[:3] - rotation @ lever
        delta = pose[:2] - truth[i, :2]
        errors.append((normal[i]@delta, tangent[i]@delta))
    errors = np.asarray(errors)
    mask = valid[:len(errors)]
    xy = np.linalg.norm(errors, axis=1)
    health_path = path/'g2e2_sensor_health.jsonl'
    health = [json.loads(line) for line in health_path.open()] if health_path.exists() else []
    summary = json.loads(Path(str(path)+'-summary.json').read_text())
    result = dict(frames=len(tx), accepted=sum(bool(row['update']['accepted']) for row in tx),
                  lateral=metrics(errors[mask, 0]), longitudinal=metrics(errors[mask, 1]),
                  xy=metrics(xy[mask]), p99_wall_ms=summary['wall_ms']['p99'])
    if health:
        result['health_states'] = dict(collections.Counter(h['status'] for h in health))
        result['vx_sources'] = dict(collections.Counter(h['vx_source'] for h in health))
        result['ellipse_lateral_3sigma_m'] = metrics([h['ellipse_lateral_3sigma_m'] for h in health])
        result['ellipse_major_3sigma_m'] = metrics([h['ellipse_major_3sigma_m'] for h in health])
        if 'ellipse_longitudinal_3sigma_m' in health[0]:
            result['ellipse_longitudinal_3sigma_m'] = metrics(
                [h['ellipse_longitudinal_3sigma_m'] for h in health])
        result['max_no_update_frames'] = {k:max(h['no_update_frames'][k] for h in health)
                                          for k in health[0]['no_update_frames']}
    for lo, hi, name in ((800, 900, 'bend_800_900'), (1254, 1406, 'bend_1254_1405'),
                         (865, 900, 'recovery_35')):
        if hi <= len(tx):
            m = mask[lo:hi]
            result[name] = dict(lateral=metrics(errors[lo:hi, 0][m]),
                                longitudinal=metrics(errors[lo:hi, 1][m]), xy=metrics(xy[lo:hi][m]))
    return result, errors, xy, health


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--reference', type=Path, required=True)
    p.add_argument('--run', action='append', nargs=2, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    data = json.loads(a.input.read_text())
    lever = static_transform(data['transforms'], 'base_link', 'imu_bosch')[:3, 3]
    ref = np.load(a.reference)
    stamp = ref['header']
    truth = ref['G_0.075']
    valid = ref['raw_valid_0.075'].astype(bool)
    normal = np.stack((-ref['tangent'][:, 1], ref['tangent'][:, 0]), axis=1)
    tangent = ref['tangent']
    result = dict(reference='75ms raw GNSS engineering reference; not independent ground truth', runs={})
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig_h, health_axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for name, path_text in a.run:
        path = Path(path_text)
        summary, errors, xy, health = run_metrics(path, stamp, truth, valid, normal, tangent, lever)
        result['runs'][name] = summary
        if len(errors) >= 900:
            x = np.arange(760, 900)
            axes[0].plot(x, errors[x, 0], label=name)
            axes[1].plot(x, xy[x], label=name)
            if health:
                health_axes[0].plot(x, [health[i]['ellipse_lateral_3sigma_m'] for i in x], label=name)
                health_axes[1].plot(x, [health[i]['vx_source'] != 'none' for i in x], label=name)
    axes[0].set(ylabel='lateral error (m)')
    axes[1].set(xlabel='frame within three-lap replay', ylabel='XY error (m)')
    for ax in axes:
        ax.grid(alpha=.3)
        ax.legend()
    fig.tight_layout()
    fig.savefig(a.output/'fault_relative_error.png', dpi=150)
    plt.close(fig)
    health_axes[0].set(ylabel='lateral 3-sigma (m)')
    health_axes[1].set(xlabel='frame within replay', ylabel='vx source valid', ylim=(-.1, 1.1))
    for ax in health_axes:
        ax.grid(alpha=.3)
        ax.legend()
    fig_h.tight_layout()
    fig_h.savefig(a.output/'health_uncertainty_curve.png', dpi=150)
    plt.close(fig_h)
    (a.output/'aggregate.json').write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
