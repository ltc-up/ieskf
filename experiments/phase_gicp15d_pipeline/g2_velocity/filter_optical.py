"""Offline port of ekf_converter::velocity_sensor_filter for existing Kistler events.

The output keeps the original status decision and engineering measurement variance.
This applies the production filter to every raw measurement before 10 ms selection.
"""
import argparse
import json
from pathlib import Path

import numpy as np


class VelocitySensorFilter:
    def __init__(self):
        self.state = np.zeros(3)
        self.covariance = np.eye(3)
        self.last_time = 0.
        self.initialized = False

    def push(self, time, velocity):
        if time - self.last_time > 1.:
            self.initialized = False
        if not self.initialized:
            self.last_time = time
            self.state[:] = (velocity, 0., 0.)
            self.initialized = True
            return float(self.state[0])
        dt = time - self.last_time
        if dt < 0:
            raise ValueError('KISTLER_TIMESTAMP_ROLLBACK')
        transition = np.array([[1, dt, .5*dt*dt], [0, 1, dt], [0, 0, 1.]])
        process = np.array([
            [dt**5/20, dt**4/8, dt**3/6],
            [dt**4/8, dt**3/3, dt**2/2],
            [dt**2/6, dt**2/2, dt],
        ])
        self.state = transition @ self.state
        self.covariance = transition @ self.covariance @ transition.T + process
        gain = self.covariance[:, 0] / (self.covariance[0, 0] + .01**2)
        self.state = self.state + (velocity - self.state[0]) * gain
        self.covariance = (np.eye(3) - np.outer(gain, [1., 0., 0.])) @ self.covariance
        self.last_time = time
        return float(self.state[0])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--audit', type=Path, required=True)
    args = parser.parse_args()
    raw = json.loads(args.input.read_text())
    filters = [VelocitySensorFilter(), VelocitySensorFilter()]
    filtered = {}
    for event in raw['raw_topics']['/sensor/kistler/measurement']:
        time = event['header_ns'] * 1e-9
        filtered[event['message_id']] = [f.push(time, value) for f, value in zip(filters, event['raw'])]
    selected = []
    delta = []
    for event in raw['events']:
        item = {key: value for key, value in event.items() if key != 'status'}
        item['raw_velocity'] = item['velocity']
        item['velocity'] = filtered[item['message_id']]
        delta.append(np.asarray(item['velocity']) - np.asarray(item['raw_velocity']))
        selected.append(item)
    args.output.write_text(json.dumps(dict(events=selected, filter='production_velocity_sensor_filter'),
                                      separators=(',', ':')))
    delta = np.asarray(delta)
    audit = dict(raw_measurements=len(filtered), selected=len(selected),
                 good=sum(event['status_good'] for event in selected),
                 mean_abs_filter_minus_raw=np.mean(abs(delta), axis=0).tolist(),
                 p95_abs_filter_minus_raw=np.percentile(abs(delta), 95, axis=0).tolist(),
                 source='ekf_converter velocity_sensor_filter: sigma_a=1, sigma_r=0.01, 1 s reset',
                 note='Filter evaluated at each raw measurement header; no 10 ms ROS timer interpolation.')
    args.audit.write_text(json.dumps(audit, indent=2) + '\n')
    print(audit)


if __name__ == '__main__':
    main()
