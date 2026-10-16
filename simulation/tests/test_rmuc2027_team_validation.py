"""任务终态与物理触压、归位证据必须一致。"""

import json

from simulation.tests.rmuc2027_team_validation import validate


def write_run(path):
    def write(name, value):
        (path / name).write_text(json.dumps(value), encoding='utf-8')
    write('map.json', {'spawns': [{'id': 0, 'x': 0, 'y': 0}],
                      'team_mission': {'home_ground_z_m': 1.6, 'body_clearance_m': .016,
                                       'armor_pressure': {'min_force_n': 50}}})
    write('team_status.json', {'complete': True, 'aborted': False, 'skipped': [], 'completed': [0],
                              'phase': 'COMPLETE', 'sim_time_us': 1000000, 'flight_elapsed_s': 1,
                              'vehicles': [{'vehicle': 0, 'fresh': True, 'landed': True, 'armed': False}]})
    write('truth_evaluation_only_contacts.jsonl', {'kind': 'armor_pressure', 'vehicle': 0,
                                                 'source': 'SIM_ARMOR_PRESSURE_SENSOR', 'qualified': True, 'force_n': 300})
    write('truth_evaluation_only.jsonl', {'kind': 'truth_evaluation_only', 'vehicle': 0,
                                       'sim_time_us': 1000000, 'x_m': 0, 'y_m': 0, 'z_m': 1.616})
    (path / 'companion_0.jsonl').write_text('', encoding='utf-8')


def test_complete_without_pressure_or_true_home_is_rejected(tmp_path):
    write_run(tmp_path)
    assert validate(tmp_path)['passed']
    (tmp_path / 'truth_evaluation_only_contacts.jsonl').write_text('', encoding='utf-8')
    assert 'vehicle_0_pressure_not_qualified' in validate(tmp_path)['failures']
    write_run(tmp_path)
    truth = json.loads((tmp_path / 'truth_evaluation_only.jsonl').read_text(encoding='utf-8'))
    truth['z_m'] = .3
    (tmp_path / 'truth_evaluation_only.jsonl').write_text(json.dumps(truth), encoding='utf-8')
    assert 'vehicle_0_truth_home_not_confirmed' in validate(tmp_path)['failures']


def test_collision_and_native_armed_feedback_cannot_be_hidden_by_complete(tmp_path):
    write_run(tmp_path)
    with (tmp_path / 'truth_evaluation_only_contacts.jsonl').open('a', encoding='utf-8') as stream:
        stream.write('\n' + json.dumps({'kind': 'physical_contact_evaluation', 'sim_time_us': 900000,
                     'collision_pairs': [{'first': 'micro_0::body', 'second': 'micro_1::body'}]}))
    report = validate(tmp_path)
    assert 'drone_collision' in report['failures']
    status = json.loads((tmp_path / 'team_status.json').read_text(encoding='utf-8'))
    status['vehicles'][0]['armed'] = True
    (tmp_path / 'team_status.json').write_text(json.dumps(status), encoding='utf-8')
    assert 'vehicle_0_native_not_stopped' in validate(tmp_path)['failures']


def test_missing_or_old_truth_and_unqualified_force_fail_closed(tmp_path):
    write_run(tmp_path)
    (tmp_path / 'truth_evaluation_only.jsonl').write_text('', encoding='utf-8')
    report = validate(tmp_path)
    assert 'vehicle_0_truth_home_not_confirmed' in report['failures']
    json.dumps(report, allow_nan=False)
    write_run(tmp_path)
    path = tmp_path / 'truth_evaluation_only_contacts.jsonl'
    pressure = json.loads(path.read_text(encoding='utf-8'))
    pressure['force_n'] = 30
    path.write_text(json.dumps(pressure), encoding='utf-8')
    assert 'vehicle_0_pressure_not_qualified' in validate(tmp_path)['failures']
    path = tmp_path / 'truth_evaluation_only.jsonl'
    truth = json.loads(path.read_text(encoding='utf-8'))
    truth['sim_time_us'] = 1
    path.write_text(json.dumps(truth), encoding='utf-8')
    assert 'vehicle_0_truth_home_not_confirmed' in validate(tmp_path)['failures']
