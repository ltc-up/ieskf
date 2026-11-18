"""G2-L private diagnostics at the native LiDAR reference time."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from analyze_g2v import load, metrics
from ndt_global.validation import static_transform


def subset(run, mask):
    seed = run['prior'][mask, 0]
    match = run['match'][mask, 0]
    post = run['posterior'][mask, 0]
    along = run['posterior'][mask, 1]
    accepted = run['accepted'][mask]
    return dict(n=int(mask.sum()), accepted=int(accepted.sum()),
                xy_rmse=float(np.sqrt(np.mean(post**2 + along**2))),
                lateral=metrics(post), along=metrics(along),
                seed_lateral=metrics(seed), match_lateral=metrics(match),
                innovation_lateral=metrics(match-seed),
                update_lateral=metrics(post-seed),
                gain_median=float(np.median(run['gain'][mask & run['accepted']])),
                prior_lateral_sigma_m=float(np.sqrt(np.mean(run['prior_lateral_variance'][mask]))),
                yaw_match_correction_deg=float(np.rad2deg(np.mean(run['yaw_match'][mask]))),
                yaw_update_deg=float(np.rad2deg(np.mean(run['yaw_post'][mask]))),
                vx_mean=float(np.mean(run['body_velocity'][mask, 0])),
                vy_mean=float(np.mean(run['body_velocity'][mask, 1])),
                yaw_rate_mean_deg_s=float(np.rad2deg(np.mean(run['yaw_rate'][mask]))),
                bg_z_mean_deg_s=float(np.rad2deg(np.mean(run['bias_g'][mask, 2]))),
                ba_y_mean=float(np.mean(run['bias_a'][mask, 1])))


def main():
    parser = argparse.ArgumentParser()
    for name in ('input', 'reference', 'run', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    data = json.loads(args.input.read_text())
    ref = np.load(args.reference)
    lever = static_transform(data['transforms'], 'base_link', 'imu_bosch')[:3, 3]
    imu_R = static_transform(data['transforms'], 'base_link', 'imu_bosch')[:3, :3]
    imu = [e for e in data['events'] if e['kind'] == 'imu']
    times = np.array([e['header_ns'] for e in imu])
    omega = np.array([imu_R @ e['omega'] for e in imu])
    tangent = ref['tangent']
    normal = np.c_[-tangent[:, 1], tangent[:, 0]]
    run = load(args.run, ref['G_0.075'], normal, tangent, lever, times, omega)
    assert np.array_equal(ref['header'], [row['ref'] for row in run['rows']])
    valid = ref['raw_valid_0.075'].astype(bool)
    corner = ref['corner']
    result = dict(reference='temporary_G_0.075_raw_valid_native_header',
                  full=subset(run, valid), corners={}, normal8_laps={})
    for number in sorted(set(corner)):
        mask = valid & (corner == number)
        if mask.any():
            result['corners'][str(number)] = subset(run, mask)
    indices = np.flatnonzero(corner == 8)
    groups = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
    for lap, group in enumerate(groups, 1):
        mask = valid & np.isin(np.arange(len(corner)), group)
        result['normal8_laps'][str(lap)] = dict(frame_start=int(group[0]),
                                                   frame_end=int(group[-1]), **subset(run, mask))
    for label, lo, hi in [('peak_800_900', 800, 900),
                          ('bend_1254_1405', 1254, 1406)]:
        if hi <= len(valid):
            mask = np.zeros(len(valid), bool)
            mask[lo:hi] = valid[lo:hi]
            result[label] = subset(run, mask)
    (args.output / 'diagnostics.json').write_text(json.dumps(result, indent=2) + '\n')
    fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
    x = np.arange(len(valid))
    for ax, key in zip(axes[:3], ('prior', 'match', 'posterior')):
        ax.plot(x[valid], run[key][valid, 0], lw=.65)
        ax.set_ylabel(key + ' lateral m')
        ax.grid(alpha=.25)
        for group in groups:
            ax.axvspan(group[0], group[-1], color='orange', alpha=.15)
    axes[3].plot(x[valid], run['gain'][valid], lw=.6, label='lateral gain')
    axes[3].plot(x[valid], run['body_velocity'][valid, 1], lw=.6, label='body vy m/s')
    axes[3].legend()
    axes[3].set_xlabel('three-lap frame')
    axes[3].grid(alpha=.25)
    fig.tight_layout()
    fig.savefig(args.output / 'lateral_diagnostics.png', dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
