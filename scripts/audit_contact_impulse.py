"""Compare recorded IMU delta velocity with native and pose-derived velocity changes."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from pyulog import ULog

GRAVITY_NED = np.array([0.0, 0.0, 9.80665])


def normalized_quaternion(value):
    q = np.asarray(value, dtype=float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('invalid quaternion')
    return q / norm


def quaternion_rotation(quaternion):
    """Rotate a body-FRD vector into local NED using a Hamilton quaternion."""
    w, x, y, z = normalized_quaternion(quaternion)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def validate_stamps(stamps):
    stamps = np.asarray(stamps, dtype=float)
    if (stamps.ndim != 1 or not np.all(np.isfinite(stamps))
            or np.any(np.diff(stamps) <= 0)):
        raise ValueError('timestamps must be finite and strictly increasing')
    return stamps


def interpolate(stamps, values, stamp, quaternion=False, max_gap_us=50000):
    """Interpolate bracketed samples; never extrapolate across missing coverage."""
    stamps = validate_stamps(stamps)
    values = np.asarray(values, dtype=float)
    if values.ndim != 2 or len(values) != len(stamps) or not np.isfinite(stamp):
        raise ValueError('invalid interpolation series')
    index = int(np.searchsorted(stamps, stamp))
    if index < len(stamps) and stamps[index] == stamp:
        value = values[index]
        if not np.all(np.isfinite(value)):
            raise ValueError('nonfinite interpolation sample')
        return normalized_quaternion(value) if quaternion else value.copy()
    if (index == 0 or index == len(stamps)
            or stamps[index] - stamps[index - 1] > max_gap_us):
        raise ValueError('missing interpolation coverage')
    left, right = values[index - 1], values[index]
    if not np.all(np.isfinite([left, right])):
        raise ValueError('nonfinite interpolation sample')
    fraction = (stamp - stamps[index - 1]) / (stamps[index] - stamps[index - 1])
    if not quaternion:
        return (1 - fraction)*left + fraction*right
    left, right = normalized_quaternion(left), normalized_quaternion(right)
    dot = float(left @ right)
    if dot < 0:
        right, dot = -right, -dot
    dot = float(np.clip(dot, -1, 1))
    if dot > .9995:
        return normalized_quaternion((1 - fraction)*left + fraction*right)
    angle = np.arccos(dot)
    return (np.sin((1 - fraction)*angle)*left + np.sin(fraction*angle)*right) / np.sin(angle)


def sample_stamps(topic):
    key = 'timestamp_sample' if 'timestamp_sample' in topic else 'timestamp'
    return validate_stamps(topic[key]), key


def integrate(imu, attitude, begin_us, end_us):
    """Integrate interval-average specific force at the clipped interval midpoint.

    Missing intervals remain explicit. Their partial integral cannot be compared
    with a full-window velocity change. Nonzero accelerometer-relative timing
    requires separate alignment and is rejected.
    """
    if begin_us < 0 or end_us <= begin_us:
        raise ValueError('require 0 <= begin-us < end-us')
    stamps = validate_stamps(imu['timestamp'])
    durations = np.asarray(imu['accelerometer_integral_dt'], dtype=float)
    if durations.shape != stamps.shape or not np.all(np.isfinite(durations)):
        raise ValueError('invalid IMU durations')
    attitude_stamps, attitude_clock = sample_stamps(attitude)
    quaternion = np.stack([attitude[f'q[{i}]'] for i in range(4)], axis=1)
    delta = np.zeros(3)
    covered_us = samples = clipping = 0
    last_end = begin_us
    gaps = []
    selected = np.flatnonzero((stamps > begin_us) & (
        (stamps - durations < end_us) | (stamps <= end_us)))
    for i in selected:
        start = max(begin_us, int(stamps[i] - durations[i]))
        end = min(end_us, int(stamps[i]))
        if durations[i] <= 0 or end <= start or start < last_end:
            raise ValueError('invalid or overlapping IMU integration interval')
        if int(imu['accelerometer_timestamp_relative'][i]) != 0:
            raise ValueError('nonzero accelerometer relative timestamp requires separate alignment')
        if start > last_end:
            gaps.append([last_end, start])
        q = interpolate(attitude_stamps, quaternion, (start + end)/2, quaternion=True)
        force = np.array([imu[f'accelerometer_m_s2[{axis}]'][i] for axis in range(3)])
        if not np.all(np.isfinite(force)):
            raise ValueError('nonfinite IMU sample')
        delta += (quaternion_rotation(q) @ force + GRAVITY_NED) * ((end - start)*1e-6)
        covered_us += end - start
        samples += 1
        clipping += int(imu['accelerometer_clipping'][i] != 0)
        last_end = end
    if last_end < end_us:
        gaps.append([last_end, end_us])
    return dict(
        available=True, complete=covered_us == end_us - begin_us,
        delta_velocity_ned_mps=delta.tolist(), covered_us=covered_us,
        window_us=end_us - begin_us, coverage_fraction=covered_us/(end_us - begin_us),
        missing_intervals_us=gaps, samples=samples, accelerometer_clipping_samples=clipping,
        attitude_time_field=attitude_clock,
    )


def velocity_change(topic, begin_us, end_us, require_valid=False):
    stamps, clock = sample_stamps(topic)
    vectors = np.stack([topic[axis] for axis in ('vx', 'vy', 'vz')], axis=1)
    endpoints = []
    for stamp in (begin_us, end_us):
        value = interpolate(stamps, vectors, stamp)
        index = int(np.searchsorted(stamps, stamp))
        indices = [index] if stamps[index] == stamp else [index - 1, index]
        if require_valid:
            for field in ('v_xy_valid', 'v_z_valid'):
                if field not in topic or not all(topic[field][i] for i in indices):
                    raise ValueError('invalid native velocity endpoint')
        endpoints.append(value)
    return dict(
        available=True, time_field=clock,
        begin_velocity_ned_mps=endpoints[0].tolist(),
        end_velocity_ned_mps=endpoints[1].tolist(),
        delta_velocity_ned_mps=(endpoints[1] - endpoints[0]).tolist(),
    )


def analyze(ulog_path, begin_us, end_us):
    topics = ['sensor_combined', 'vehicle_attitude', 'vehicle_attitude_groundtruth',
              'vehicle_local_position', 'vehicle_local_position_groundtruth']
    ulog_path = Path(ulog_path)
    ulog = ULog(str(ulog_path), message_name_filter_list=topics)
    data = {d.name: d.data for d in ulog.data_list if d.multi_id == 0}
    result = dict(begin_us=begin_us, end_us=end_us,
                  ulog_sha256=hashlib.sha256(ulog_path.read_bytes()).hexdigest())
    for name, output in [('vehicle_attitude', 'estimated_attitude'),
                         ('vehicle_attitude_groundtruth', 'truth_attitude')]:
        try:
            result[output] = integrate(data['sensor_combined'], data[name], begin_us, end_us)
        except (ValueError, KeyError) as error:
            result[output] = dict(available=False, reason=str(error))
    for name, output in [('vehicle_local_position', 'native'),
                         ('vehicle_local_position_groundtruth', 'gazebo_pose_velocity')]:
        try:
            change = velocity_change(data[name], begin_us, end_us, require_valid=output == 'native')
            result[output] = change
            delta = np.array(change['delta_velocity_ned_mps'])
            for kind in ('estimated_attitude', 'truth_attitude'):
                integrated = result[kind]
                if integrated.get('available') and integrated['complete']:
                    error = np.array(integrated['delta_velocity_ned_mps']) - delta
                    integrated[output + '_difference_ned_mps'] = error.tolist()
                    integrated[output + '_difference_norm_mps'] = float(np.linalg.norm(error))
        except (ValueError, KeyError) as error:
            result[output] = dict(available=False, reason=str(error))
    result['available'] = all(result[key].get('available') for key in (
        'estimated_attitude', 'truth_attitude', 'native', 'gazebo_pose_velocity'))
    result['complete'] = result['available'] and all(result[key]['complete'] for key in (
        'estimated_attitude', 'truth_attitude'))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ulog', type=Path)
    parser.add_argument('--begin-us', type=int, required=True)
    parser.add_argument('--end-us', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.begin_us < 0 or args.end_us <= args.begin_us:
        parser.error('require 0 <= begin-us < end-us')
    result = analyze(args.ulog, args.begin_us, args.end_us)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2) + '\n',
                           encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()
