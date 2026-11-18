"""Private per-frame G2-V comparison using an explicitly temporary reference."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation

from ndt_global.validation import static_transform


def metrics(values):
    values = np.asarray(values)
    return dict(n=int(len(values)), mean=float(np.mean(values)),
                rmse=float(np.sqrt(np.mean(values**2))),
                abs_p95=float(np.percentile(abs(values), 95)),
                abs_max=float(np.max(abs(values))))


def longest_run(values, threshold=.3):
    longest = current = 0
    for value in values:
        current = current + 1 if abs(value) > threshold else 0
        longest = max(longest, current)
    return longest


def load(path, reference, normal, tangent, lever, omega_times, omega_values):
    rows = json.loads((path / 'transactions.json').read_text())
    assert len(rows) == len(reference)
    prior = []
    match = []
    posterior = []
    gain = []
    yaw_gain = []
    yaw_match = []
    yaw_post = []
    bias_g = []
    bias_a = []
    body_velocity = []
    yaw_rate = []
    prior_lateral_variance = []
    yaw_variance = []
    accepted = []
    for i, row in enumerate(rows):
        assert row['index'] == i
        seed = np.asarray(row['seed'])
        fit = np.asarray(row['ndt']['final'])
        pr = row['prior']
        p = np.asarray(pr['p'])
        rotation = np.asarray(pr['R'])
        omega = omega_values[np.searchsorted(omega_times, row['ref'], side='right') - 1]
        gyro_bias = np.asarray(pr.get('bg', [np.nan]*3))
        body_velocity.append(rotation.T @ np.asarray(pr.get('v', [np.nan]*3)) -
                             np.cross(omega - gyro_bias, lever))
        yaw_rate.append(float(omega[2] - gyro_bias[2]))
        covariance = np.asarray(pr['P'])
        jacobian = np.zeros((3, 15))
        jacobian[:, :3] = np.eye(3)
        jacobian[:, 6:9] = rotation @ np.array([
            [0, -lever[2], lever[1]], [lever[2], 0, -lever[0]],
            [-lever[1], lever[0], 0]])
        projection = normal[i] @ jacobian[:2]
        prior_lateral_variance.append(float(projection @ covariance @ projection))
        yaw_variance.append(float(covariance[8, 8]))
        update = row['update']
        delta = np.asarray(update.get('delta', np.zeros(15)))
        new_rotation = rotation @ Rotation.from_rotvec(delta[6:9]).as_matrix()
        post = p + delta[:3] - new_rotation @ lever
        def lat(x): return float(normal[i] @ (x[:2] - reference[i, :2]))
        def along(x): return float(tangent[i] @ (x[:2] - reference[i, :2]))
        prior.append((lat(seed[:3, 3]), along(seed[:3, 3])))
        match.append((lat(fit[:3, 3]), along(fit[:3, 3])))
        posterior.append((lat(post), along(post)))
        K = np.asarray(update.get('K', np.zeros((15, 6))))
        gain.append(float(normal[i] @ K[:2, :2] @ normal[i]))
        yaw_gain.append(float(normal[i] @ K[:2, 5]))
        yaw_match.append(float(Rotation.from_matrix(seed[:3, :3].T @ fit[:3, :3]).as_rotvec()[2]))
        yaw_post.append(float(delta[8]))
        bias_g.append(np.asarray(pr.get('bg', [np.nan]*3)))
        bias_a.append(np.asarray(pr.get('ba', [np.nan]*3)))
        accepted.append(bool(update['accepted']))
    return dict(prior=np.asarray(prior), match=np.asarray(match), posterior=np.asarray(posterior),
                gain=np.asarray(gain), yaw_gain=np.asarray(yaw_gain),
                yaw_match=np.asarray(yaw_match), yaw_post=np.asarray(yaw_post),
                bias_g=np.asarray(bias_g), bias_a=np.asarray(bias_a),
                body_velocity=np.asarray(body_velocity), yaw_rate=np.asarray(yaw_rate),
                prior_lateral_variance=np.asarray(prior_lateral_variance),
                yaw_variance=np.asarray(yaw_variance),
                accepted=np.asarray(accepted), rows=rows)


def source_counts(path):
    log = path / 'velocity_source.jsonl'
    if not log.exists():
        return {}
    records = [json.loads(line) for line in log.open()]
    from collections import Counter
    count = Counter((r['kind'], r['source'], r['update'].get('reason')) for r in records)
    out = {f'{kind}:{source}:{reason}': number for (kind, source, reason), number in count.items()}
    for axis in ('wheel', 'optical'):
        innovation = [float(r['update']['innovation'] if axis == 'wheel' else r['update']['innovation'][-1])
                      for r in records if r['kind'] == axis and r['update'].get('accepted')
                      and 'innovation' in r['update']]
        if innovation:
            out[axis + '_innovation'] = metrics(innovation)
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--reference', type=Path, required=True)
    p.add_argument('--run', action='append', nargs=2, metavar=('NAME', 'PATH'), required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    data = json.loads(a.input.read_text())
    lever = static_transform(data['transforms'], 'base_link', 'imu_bosch')[:3, 3]
    imu_rotation = static_transform(data['transforms'], 'base_link', 'imu_bosch')[:3, :3]
    imu_events = [e for e in data['events'] if e['kind'] == 'imu']
    omega_times = np.asarray([e['header_ns'] for e in imu_events])
    omega_values = np.asarray([imu_rotation @ e['omega'] for e in imu_events])
    reference = np.load(a.reference)
    position = reference['G_0.075']
    tangent = reference['tangent']
    normal = np.c_[-tangent[:, 1], tangent[:, 0]]
    valid = reference['historical_valid_0.075'].astype(bool)
    heading = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
    arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(position[:, :2], axis=0), axis=1))]
    curvature = gaussian_filter1d(np.gradient(heading, arc), 5)
    magnitude = abs(curvature)
    turn = magnitude > np.percentile(magnitude[valid], 50)
    apex = magnitude >= np.percentile(magnitude[valid], 85)
    growth = np.gradient(gaussian_filter1d(magnitude, 5))
    stage = np.where(~turn, 'straight', np.where(apex, 'apex', np.where(growth >= 0, 'entry', 'exit')))
    runs = {name: load(Path(path), position, normal, tangent, lever,
                       omega_times, omega_values) for name, path in a.run}
    result = dict(reference='temporary_G_0.075_historical_valid_only',
                  stage_rule='Gaussian sigma 5 frames; straight below valid median absolute curvature; apex above valid p85; entry/exit by smoothed curvature slope',
                  curvature_quantiles=np.percentile(magnitude[valid], [50, 85]).tolist(), runs={})
    ranges = {'full': (0, len(position)), 'peak_800_900': (800, 900),
              'corner_1254_1297': (1254, 1297), 'offset_1317_1406': (1317, 1406),
              'corner_1254_1406': (1254, 1406)}
    table = []
    for name, run in runs.items():
        item = dict(accepted=int(run['accepted'].sum()),
                    rejected=int((~run['accepted']).sum()), source=source_counts(Path(dict(a.run)[name])),
                    ranges={}, stages={})
        for label, (lo, hi) in ranges.items():
            m = valid[lo:hi]
            seed = run['prior'][lo:hi][m]
            match = run['match'][lo:hi][m]
            post = run['posterior'][lo:hi][m]
            item['ranges'][label] = dict(n=int(m.sum()), seed_lateral=metrics(seed[:, 0]),
                match_lateral=metrics(match[:, 0]), posterior_lateral=metrics(post[:, 0]),
                posterior_along=metrics(post[:, 1]),
                posterior_xy_rmse=float(np.sqrt(np.mean(np.sum(post**2, axis=1)))),
                longest_abs_lateral_gt_0p3=longest_run(post[:, 0]),
                mean_prediction_lateral_step=float(np.mean(run['prior'][lo+1:hi, 0] - run['posterior'][lo:hi-1, 0])),
                prediction_lateral_step_rmse=float(np.sqrt(np.mean((run['prior'][lo+1:hi, 0] - run['posterior'][lo:hi-1, 0])**2))),
                mean_update_lateral_step=float(np.mean(run['posterior'][lo:hi, 0] - run['prior'][lo:hi, 0])),
                update_lateral_step_rmse=float(np.sqrt(np.mean((run['posterior'][lo:hi, 0] - run['prior'][lo:hi, 0])**2))),
                median_lateral_gain=float(np.median(run['gain'][lo:hi][run['accepted'][lo:hi]])),
                mean_match_yaw_correction_deg=float(np.rad2deg(np.mean(run['yaw_match'][lo:hi]))),
                mean_post_yaw_correction_deg=float(np.rad2deg(np.mean(run['yaw_post'][lo:hi]))),
                mean_body_vx_m_s=float(np.mean(run['body_velocity'][lo:hi, 0])),
                mean_body_vy_m_s=float(np.mean(run['body_velocity'][lo:hi, 1])),
                mean_yaw_rate_deg_s=float(np.rad2deg(np.mean(run['yaw_rate'][lo:hi]))),
                mean_gyro_bias_z_deg_s=float(np.rad2deg(np.nanmean(run['bias_g'][lo:hi, 2]))),
                mean_accel_bias_y_m_s2=float(np.nanmean(run['bias_a'][lo:hi, 1])),
                mean_prior_lateral_variance_m2=float(np.mean(run['prior_lateral_variance'][lo:hi])),
                mean_prior_yaw_variance_deg2=float(np.rad2deg(np.sqrt(np.mean(run['yaw_variance'][lo:hi])))**2))
        for label in ('straight', 'entry', 'apex', 'exit'):
            m = valid & (stage == label)
            post = run['posterior'][m]
            item['stages'][label] = dict(n=int(m.sum()), lateral=metrics(post[:, 0]),
                                          along=metrics(post[:, 1]),
                                          xy_rmse=float(np.sqrt(np.mean(np.sum(post**2, axis=1)))))
        result['runs'][name] = item
        for i in range(len(position)):
            table.append(dict(run=name, frame=i, valid=int(valid[i]), stage=stage[i],
                              curvature=float(curvature[i]), seed_lateral=float(run['prior'][i, 0]),
                              match_lateral=float(run['match'][i, 0]),
                              post_lateral=float(run['posterior'][i, 0]),
                              post_along=float(run['posterior'][i, 1]),
                              body_vx=float(run['body_velocity'][i, 0]),
                              body_vy=float(run['body_velocity'][i, 1]),
                              yaw_rate=float(run['yaw_rate'][i]),
                              accepted=int(run['accepted'][i])))
    (a.output / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    with (a.output / 'per_frame.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(table[0]))
        writer.writeheader()
        writer.writerows(table)
    figure, axes = plt.subplots(4, 1, figsize=(15, 12), sharex=True)
    for name, run in runs.items():
        post = run['posterior']
        axes[0].plot(np.where(valid)[0], post[valid, 0], lw=.8, label=name)
        axes[1].plot(np.where(valid)[0], post[valid, 1], lw=.8, label=name)
        axes[2].plot(np.where(valid)[0], np.linalg.norm(post[valid], axis=1), lw=.8, label=name)
    for axis, ylabel in zip(axes, ('lateral m', 'along m', 'XY m')):
        axis.axvspan(800, 900, color='grey', alpha=.15)
        axis.axvspan(1254, 1406, color='grey', alpha=.15)
        axis.set_ylabel(ylabel)
        axis.grid(alpha=.25)
    axes[0].legend()
    for label, color in (('straight', 'tab:grey'), ('entry', 'tab:orange'),
                         ('apex', 'tab:red'), ('exit', 'tab:green')):
        selected = valid & (stage == label)
        axes[3].scatter(np.flatnonzero(selected), magnitude[selected],
                        s=3, color=color, label=label)
    axes[3].set_ylabel('abs curvature, 1/m')
    axes[3].legend(ncol=4)
    axes[-1].set_xlabel('frame')
    figure.tight_layout()
    figure.savefig(a.output / 'error_curves.png', dpi=160)
    figure.savefig(a.output / 'stage_error_curves.png', dpi=160)
    plt.close(figure)


if __name__ == '__main__':
    main()
