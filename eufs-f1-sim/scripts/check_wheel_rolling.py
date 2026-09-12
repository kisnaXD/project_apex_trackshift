#!/usr/bin/env python3
"""Read-only wheel rolling and steering verification for a running EUFS car.

This node subscribes only to odometry and joint states.  It never publishes a
command or changes simulator state.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


RADIUS_M = 0.346
PAIR_TOLERANCE_S = 0.03
WHEELS = (
    'left_front_wheel_joint', 'right_front_wheel_joint',
    'left_rear_wheel_joint', 'right_rear_wheel_joint',
)


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def wrap_delta(value):
    return math.atan2(math.sin(value), math.cos(value))


class WheelChecker(Node):
    def __init__(self):
        super().__init__('check_wheel_rolling')
        self.odometry = deque(maxlen=5000)
        self.odom_first = None
        self.odom_latest = None
        self.odom_max_speed = 0.0
        self.joints = {name: [] for name in WHEELS}
        self.steering = {}
        self.gaps = {name: 0 for name in WHEELS}
        self.missing_velocity = {name: 0 for name in WHEELS}
        self.create_subscription(Odometry, '/eufs/odom', self._odom, 20)
        self.create_subscription(JointState, '/eufs/joint_states', self._joint, 20)

    def _odom(self, msg):
        sample = (stamp_seconds(msg.header.stamp),
                  float(msg.twist.twist.linear.x),
                  float(msg.pose.pose.position.x))
        self.odometry.append(sample)
        self.odom_first = self.odom_first or sample
        self.odom_latest = sample
        self.odom_max_speed = max(self.odom_max_speed, abs(sample[1]))

    def _nearest_odom(self, stamp):
        if not self.odometry:
            return None
        candidate = min(self.odometry, key=lambda item: abs(item[0] - stamp))
        return candidate if abs(candidate[0] - stamp) < PAIR_TOLERANCE_S else None

    def _joint(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        paired = self._nearest_odom(stamp)
        if paired is None:
            return
        _, vx, _ = paired
        values = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        for name in WHEELS:
            if name not in values:
                continue
            position = float(values[name])
            samples = self.joints[name]
            if samples and stamp > samples[-1]['t']:
                dt = stamp - samples[-1]['t']
                if dt > 0.1:
                    self.gaps[name] += 1
                    # Establish a fresh baseline after a telemetry gap;
                    # retain the failure but do not poison all later samples.
                    samples.append({'t': stamp, 'position': position, 'vx': vx})
                    continue
                if name not in velocities:
                    self.missing_velocity[name] += 1
                    continue
                measured = float(velocities[name])
                previous_omega = samples[-1].get('omega', measured)
                prediction = 0.5 * (previous_omega + measured) * dt
                raw_delta = position - samples[-1]['position']
                # Joint positions may wrap at 2π. Choose the equivalent delta
                # nearest the expected increment, while retaining large-jump
                # samples as failures instead of hiding them with modulo math.
                turns = round((raw_delta - prediction) / (2.0 * math.pi))
                unwrapped_delta = raw_delta - turns * 2.0 * math.pi
                residual = unwrapped_delta - prediction
                samples.append({
                    't': stamp, 'position': position, 'vx': vx,
                    'omega': measured, 'dt': dt, 'residual': residual,
                    'delta': unwrapped_delta,
                })
            elif not samples:
                if name not in velocities:
                    self.missing_velocity[name] += 1
                samples.append({'t': stamp, 'position': position, 'vx': vx})
        steer = [(name, float(value)) for name, value in values.items()
                 if 'steer' in name]
        for name, value in steer:
            self.steering.setdefault(name, []).append((stamp, value))

    def report(self):
        wheel_report = {}
        all_samples = []
        failures = []
        stopped_durations = []
        for name, samples in self.joints.items():
            measured = [s for s in samples if 'omega' in s]
            moving = [s for s in measured if abs(s['vx']) > 1.0]
            errors = [abs(s['omega'] - s['vx'] / RADIUS_M) for s in moving]
            residuals = [abs(s['residual']) for s in measured]
            backwards = [s for s in measured if s['vx'] > 0.3 and s['delta'] < -0.01]
            suffix = []
            for sample in reversed(measured):
                if abs(sample['vx']) >= 0.05:
                    break
                if suffix and suffix[-1]['t'] - sample['t'] > 0.1:
                    break
                suffix.append(sample)
            suffix.reverse()
            stopped_duration = suffix[-1]['t'] - suffix[0]['t'] if len(suffix) > 1 else 0.0
            settled = [s for s in suffix if s['t'] >= suffix[0]['t'] + 0.3]
            phase = 0.0
            phases = [phase]
            for sample in settled[1:]:
                phase += sample['delta']
                phases.append(phase)
            phase_range = max(phases, default=0.0) - min(phases, default=0.0)
            stopped_omega = max((abs(s['omega']) for s in settled), default=None)
            stopped_durations.append(stopped_duration)
            wheel_report[name] = {
                'samples': len(measured),
                'moving_samples': len(moving),
                'max_omega_error_rad_s': max(errors, default=None),
                'max_position_residual_rad': max(residuals, default=None),
                'negative_steps_at_positive_speed': len(backwards),
                'final_stopped_segment_s': stopped_duration,
                'final_stopped_phase_range_rad': phase_range,
                'final_stopped_max_abs_omega_rad_s': stopped_omega,
            }
            all_samples.extend(measured)
            if not measured:
                failures.append(f'{name}: no paired rolling samples')
            elif not moving:
                failures.append(f'{name}: no sample with |vx| > 1 m/s')
            elif max(errors) > 0.15:
                failures.append(f'{name}: omega error {max(errors):.3f} rad/s > 0.15')
            if self.missing_velocity[name]:
                failures.append(f'{name}: {self.missing_velocity[name]} joint samples missing velocity')
            if self.gaps[name]:
                failures.append(f'{name}: {self.gaps[name]} sample gaps > 0.1 sim s')
            if residuals and max(residuals) > 0.02:
                failures.append(f'{name}: position residual {max(residuals):.3f} rad > 0.02')
            if backwards:
                failures.append(f'{name}: negative rolling step at positive speed')
            if stopped_duration < 1.0:
                failures.append(f'{name}: final stopped segment is {stopped_duration:.2f} sim s; need >= 1.00 s')
            if phase_range > 0.005:
                failures.append(f'{name}: stopped phase range {phase_range:.4f} rad > 0.005')
            if stopped_omega is not None and stopped_omega > 0.05:
                failures.append(f'{name}: stopped omega {stopped_omega:.3f} rad/s > 0.05')

        steer_report = {}
        for name, samples in self.steering.items():
            values = [value for _, value in samples]
            steer_report[name] = {'samples': len(values), 'min_rad': min(values), 'max_rad': max(values)}
            if max(abs(value) for value in values) > 0.02:
                failures.append(f'{name}: steering angle exceeds 0.02 rad')
        displacement = (self.odom_latest[2] - self.odom_first[2]
                        if self.odom_first and self.odom_latest else None)
        max_speed = self.odom_max_speed if self.odom_latest else None
        return {
            'wheel_radius_m': RADIUS_M,
            'paired_samples': len(all_samples),
            'odom_displacement_x_m': displacement,
            'odom_max_abs_speed_m_s': max_speed,
            'final_stopped_segment_s': min(stopped_durations, default=0.0),
            'steering': steer_report,
            'wheels': wheel_report,
            'failures': failures,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=22.0,
                        help='wall-clock collection duration (default: 22 s)')
    parser.add_argument('--json', metavar='PATH', help='write the report JSON to PATH')
    args = parser.parse_args()
    rclpy.init()
    node = WheelChecker()
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))
        report = node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    encoded = json.dumps(report, indent=2, sort_keys=True)
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as stream:
            stream.write(encoded + '\n')
    print(encoded)
    if report['failures']:
        print('wheel verification FAILED: ' + '; '.join(report['failures']))
        return 1
    print('wheel verification PASSED')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
