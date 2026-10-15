"""核对单实例 EKF2 回放所索引的传感器时间戳是否存在于 ULog。"""

import argparse
import hashlib
import json
import math
from pathlib import Path

from pyulog import ULog


RELATIVE_TOPICS = {
    'airspeed_timestamp_rel': 'airspeed',
    'distance_sensor_timestamp_rel': 'distance_sensor',
    'optical_flow_timestamp_rel': 'vehicle_optical_flow',
    'vehicle_air_data_timestamp_rel': 'vehicle_air_data',
    'vehicle_magnetometer_timestamp_rel': 'vehicle_magnetometer',
    'visual_odometry_timestamp_rel': 'vehicle_visual_odometry',
}


def audit(path, begin_s=0.0):
    names = ['ekf2_timestamps', 'sensor_combined', *RELATIVE_TOPICS.values()]
    log = ULog(str(path), message_name_filter_list=names)
    groups = {}
    for data in log.data_list:
        groups.setdefault(data.name, []).append(data)
    clocks = groups.get('ekf2_timestamps', [])
    if len(clocks) != 1 or clocks[0].multi_id != 0:
        raise ValueError('需要包含单个 0 号实例的 ekf2_timestamps')
    clock = clocks[0].data
    stamps = clock['timestamp']
    if not len(stamps) or not 0 <= begin_s < int(stamps[-1]) / 1e6:
        raise ValueError('起始时间必须在日志时间范围内')
    begin_us = begin_s * 1e6
    topics = {}
    for field, topic in [(None, 'sensor_combined'), *RELATIVE_TOPICS.items()]:
        records = groups.get(topic, [])
        requests = []
        for index, stamp in enumerate(stamps):
            # 相对时间为有符号的 0.1 ms；先转 Python int，避免 uint64 加负数溢出。
            relative = int(clock[field][index]) if field else 0
            if relative != 32767:
                requests.append((int(stamp), int(stamp) // 100 + relative))
        if requests and (len(records) > 1 or records and records[0].multi_id != 0):
            raise ValueError(f'{topic} 包含非单实例输入，无法确定原生回放选择的实例')
        available = set(int(t) // 100 for t in records[0].data['timestamp']) if records else set()
        missing = [{'filter_time_us': stamp, 'requested_time_100us': target}
                   for stamp, target in requests if target not in available]
        topics[topic] = {
            'logged_samples': len(records[0].data['timestamp']) if records else 0,
            'requested_total': len(requests),
            'requested_in_window': sum(stamp >= begin_us for stamp, _ in requests),
            'missing_total': len(missing),
            'missing_in_window': sum(item['filter_time_us'] >= begin_us for item in missing),
            'missing_references': missing,
        }
    requested = sum(item['requested_in_window'] for item in topics.values())
    if not requested or not topics['sensor_combined']['requested_in_window']:
        raise ValueError('分析区间没有可检查的 IMU 输入请求')
    complete = all(item['missing_in_window'] == 0 for item in topics.values())
    return {
        'input': str(path.resolve()), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'sdlog_profile': log.initial_parameters.get('SDLOG_PROFILE'),
        'begin_s': begin_s, 'end_s': int(stamps[-1]) / 1e6,
        'check_scope': 'EKF2 timestamp-indexed sensors; excludes directly replayed status, landing and GPS',
        'timestamp_inputs_complete': complete, 'topics': topics,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ulog', type=Path)
    parser.add_argument('--begin-s', type=float, default=0.0, help='PX4 日志时钟的分析起点，单位 s')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.begin_s) or args.begin_s < 0:
        parser.error('begin-s 必须是有限非负数')
    report = audit(args.ulog, args.begin_s)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({
        'timestamp_inputs_complete': report['timestamp_inputs_complete'],
        'begin_s': report['begin_s'],
        'missing_in_window': {k: v['missing_in_window'] for k, v in report['topics'].items()},
    }))
    return 0 if report['timestamp_inputs_complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
