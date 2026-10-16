"""核验整队触压、真实归位、机间碰撞和原生停机反馈。"""

import argparse
import json
import math
from pathlib import Path


def read_rows(path):
    with Path(path).open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def validate(run):
    run = Path(run)
    config = json.loads((run / 'map.json').read_text(encoding='utf-8'))
    status = json.loads((run / 'team_status.json').read_text(encoding='utf-8'))
    contacts = read_rows(run / 'truth_evaluation_only_contacts.jsonl')
    truth = read_rows(run / 'truth_evaluation_only.jsonl')
    homes = {s['id']: s for s in config['spawns']}
    mission = config['team_mission']
    minimum_force = mission['armor_pressure']['min_force_n']
    qualified = {vehicle: [] for vehicle in homes}
    collisions = []
    for row in contacts:
        if (row.get('kind') == 'armor_pressure' and row.get('source') == 'SIM_ARMOR_PRESSURE_SENSOR'
                and row.get('qualified') and row.get('vehicle') in homes
                and math.isfinite(row.get('force_n', float('nan'))) and row['force_n'] >= minimum_force):
            qualified[row['vehicle']].append(row['force_n'])
        for pair in row.get('collision_pairs', []):
            if pair['first'].startswith('micro_') and pair['second'].startswith('micro_'):
                collisions.append({'time_s': row['sim_time_us'] / 1e6, **pair})
    final_truth = {}
    for row in truth:
        if row.get('kind') == 'truth_evaluation_only' and row.get('vehicle') in homes:
            old = final_truth.get(row['vehicle'])
            if old is None or row['sim_time_us'] > old['sim_time_us']:
                final_truth[row['vehicle']] = row
    native = {row['vehicle']: row for row in status['vehicles']}
    failures, landings, protections = [], [], []
    if not status.get('complete') or status.get('aborted') or status.get('skipped'):
        failures.append('mission_not_complete')
    if set(status.get('completed', [])) != set(homes):
        failures.append('mission_contact_members_incomplete')
    if collisions:
        failures.append('drone_collision')
    for vehicle, home in homes.items():
        if not qualified[vehicle]:
            failures.append(f'vehicle_{vehicle}_pressure_not_qualified')
        feedback = native.get(vehicle, {})
        if not (feedback.get('fresh') and feedback.get('landed') and not feedback.get('armed', True)):
            failures.append(f'vehicle_{vehicle}_native_not_stopped')
        actual = final_truth.get(vehicle)
        ground_center = mission['home_ground_z_m'] + mission['body_clearance_m']
        xy_error = math.hypot(actual['x_m'] - home['x'], actual['y_m'] - home['y']) if actual else None
        z_error = abs(actual['z_m'] - ground_center) if actual else None
        age = abs(status['sim_time_us'] - actual['sim_time_us']) / 1e6 if actual else None
        landed_at_home = actual is not None and xy_error <= .05 and z_error <= .035 and age <= .2
        if not landed_at_home:
            failures.append(f'vehicle_{vehicle}_truth_home_not_confirmed')
        landings.append({'vehicle': vehicle, 'xy_error_m': xy_error, 'z_error_m': z_error,
                         'truth_age_s': age, 'passed': landed_at_home})
        protections.extend(row for row in read_rows(run / f'companion_{vehicle}.jsonl')
                           if row.get('kind') == 'unrecoverable')
    if protections:
        failures.append('unrecoverable_protection')
    return {'run': run.name, 'passed': not failures, 'failures': failures,
            'mission_phase': status['phase'], 'flight_elapsed_s': status['flight_elapsed_s'],
            'qualified_pressure_n': {str(k): values for k, values in qualified.items()},
            'truth_home_xy_tolerance_m': .05, 'truth_home_z_tolerance_m': .035,
            'truth_max_age_s': .2, 'landings': landings,
            'drone_contact_pair_samples': len(collisions), 'drone_contacts': collisions,
            'unrecoverable_events': protections}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = validate(args.run)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps({'run': report['run'], 'passed': report['passed'], 'failures': report['failures']}, ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
