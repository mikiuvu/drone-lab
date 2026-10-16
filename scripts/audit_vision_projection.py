"""Compare recorded vision projection poses with independent Gazebo position truth."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np


class ScalarSeries:
    """Bracket scalar truth samples without extrapolating or bridging long gaps."""

    def __init__(self, stamps, values, max_gap_us=50000):
        self.stamps = np.asarray(stamps, dtype=float)
        self.values = np.asarray(values, dtype=float)
        self.max_gap_us = max_gap_us
        if (self.stamps.ndim != 1 or self.values.shape != self.stamps.shape
                or not np.all(np.isfinite(self.stamps)) or not np.all(np.isfinite(self.values))
                or np.any(np.diff(self.stamps) <= 0)):
            raise ValueError('invalid or unordered truth series')

    def at(self, stamp):
        if not np.isfinite(stamp):
            raise ValueError('invalid query time')
        index = int(np.searchsorted(self.stamps, stamp))
        if index < len(self.stamps) and self.stamps[index] == stamp:
            return float(self.values[index])
        if (index == 0 or index == len(self.stamps)
                or self.stamps[index] - self.stamps[index - 1] > self.max_gap_us):
            raise ValueError('missing truth coverage')
        fraction = (stamp - self.stamps[index - 1]) / (self.stamps[index] - self.stamps[index - 1])
        return float((1 - fraction)*self.values[index - 1] + fraction*self.values[index])


def sample_error(projection, observation, ego, target):
    """Subtract ego translation bias; the residual still includes rotation and image errors."""
    stamp = observation['sim_time_us']
    if projection['sim_time_us'] != stamp or not projection['pose_valid']:
        raise ValueError('invalid projection association')
    pose_stamp = projection['pose_time_us']
    used_y, observed_y, variance = (float(projection['pose_y_m']), float(observation['y_m']),
                                  float(projection['pose_variance_y_m2']))
    if not np.all(np.isfinite([used_y, observed_y, variance])) or variance < 0:
        raise ValueError('invalid projection values')
    true_ego_y, true_pose_y, true_target_y = ego.at(stamp), ego.at(pose_stamp), target.at(stamp)
    ego_error = used_y - true_ego_y
    observed_error = observed_y - true_target_y
    return dict(
        sim_time_us=stamp, pose_time_us=pose_stamp, pose_time_offset_us=pose_stamp - stamp,
        measurement=observation['measurement'], accepted=bool(observation['accepted']),
        world_y_m=observed_y, true_target_y_m=true_target_y, used_pose_y_m=used_y,
        true_ego_y_at_exposure_m=true_ego_y, true_ego_y_at_pose_m=true_pose_y,
        ego_translation_error_y_m=ego_error, ego_state_error_y_m=used_y - true_pose_y,
        ego_time_shift_y_m=true_pose_y - true_ego_y, measurement_error_y_m=observed_error,
        residual_after_ego_translation_y_m=observed_error - ego_error,
        pose_sigma_y_m=float(np.sqrt(variance)),
    )


def summarize(rows):
    result = dict(samples=len(rows))
    if not rows:
        return result
    for name in ('ego_translation_error_y_m', 'ego_state_error_y_m', 'ego_time_shift_y_m',
                 'measurement_error_y_m', 'residual_after_ego_translation_y_m'):
        values = np.array([r[name] for r in rows])
        result[name] = dict(rmse=float(np.sqrt(np.mean(values**2))),
                            abs_p95=float(np.percentile(np.abs(values), 95)))
    ego = np.array([r['ego_translation_error_y_m'] for r in rows])
    observed = np.array([r['measurement_error_y_m'] for r in rows])
    result['ego_measurement_error_correlation'] = (
        float(np.corrcoef(ego, observed)[0, 1])
        if len(rows) >= 3 and np.std(ego) > 1e-12 and np.std(observed) > 1e-12 else None)
    result['pose_time_offset_abs_max_us'] = max(abs(r['pose_time_offset_us']) for r in rows)
    result['future_pose_samples'] = sum(r['pose_time_offset_us'] > 0 for r in rows)
    return result


def unique_projections(rows):
    projections = {}
    duplicates = set()
    for row in rows:
        if row.get('kind') == 'vision_projection_input':
            stamp = row['sim_time_us']
            if stamp in projections:
                duplicates.add(stamp)
            projections[stamp] = row
    for stamp in duplicates:
        del projections[stamp]
    return projections, duplicates


def groups(rows):
    result = {}
    for measurement in ('blue_pnp', 'green_rail_plane'):
        selected = [r for r in rows if r['measurement'] == measurement]
        result[measurement] = summarize(selected)
        result[measurement + '_accepted_precommit'] = summarize(
            [r for r in selected if r['accepted'] and r['precommit']])
    return result


def analyze(run):
    run = Path(run)
    sources = {}

    def read(name, lines=False):
        path = run / name
        content = path.read_bytes()
        sources[name] = hashlib.sha256(content).hexdigest()
        text = content.decode('utf-8-sig')
        return [json.loads(line) for line in text.splitlines() if line.strip()] if lines else json.loads(text)

    config = read('map.json')
    body_truth = read('truth_evaluation_only.jsonl', True)
    armor_truth = read('armor_truth.jsonl', True)
    center_y = float(config['team_mission']['target']['y'])
    target = ScalarSeries([r['sim_time_us'] for r in armor_truth],
                          [r['y_m'] + center_y for r in armor_truth])
    vehicle_ids = sorted({r['vehicle'] for r in body_truth})
    all_samples, members = [], []
    for vehicle in vehicle_ids:
        rows = read(f'companion_{vehicle}.jsonl', True)
        truth = [r for r in body_truth if r['vehicle'] == vehicle]
        ego = ScalarSeries([r['sim_time_us'] for r in truth], [r['y_m'] for r in truth])
        projections, duplicates = unique_projections(rows)
        commits = [r['sim_time_us'] for r in rows if r.get('kind') == 'dive_aim' and r.get('committed')]
        samples, skipped = [], Counter()
        for row in rows:
            if row.get('kind') != 'visual_pose':
                continue
            stamp = row['sim_time_us']
            if stamp in duplicates:
                skipped['ambiguous_projection'] += 1
                continue
            if stamp not in projections:
                skipped['missing_projection'] += 1
                continue
            try:
                sample = sample_error(projections[stamp], row, ego, target)
            except (ValueError, KeyError) as error:
                skipped[str(error)] += 1
                continue
            sample.update(vehicle=vehicle, precommit=any(0 <= t - stamp <= 1000000 for t in commits))
            samples.append(sample)
        members.append(dict(vehicle=vehicle, projection_inputs=len(projections),
                            ambiguous_stamps=len(duplicates), skipped=dict(skipped), groups=groups(samples)))
        all_samples.extend(samples)
    return dict(run=str(run.resolve()), source_sha256=sources, vehicles=members,
                groups=groups(all_samples), samples=all_samples)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.run)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps(result['groups'], indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
