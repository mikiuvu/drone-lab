"""三角网格的航段求交与机体净空。"""

import json
from pathlib import Path

import numpy as np

from simulation.check_map import (
    TriangleScene,
    check_map,
    approach_waiting_separation,
    formation_separation,
    landing_waiting_separation,
)


def test_segment_returns_nearest_wall_and_ignores_geometry_outside_segment():
    face = np.array([[[1, -1, -1], [1, 1, -1], [1, 0, 1]]], dtype=float)
    scene = TriangleScene(np.concatenate([face + [1, 0, 0], face]))
    hit = scene.segment_hit([0, 0, 0], [3, 0, 0])
    np.testing.assert_allclose(hit['point_m'], [1, 0, 0])
    assert scene.segment_hit([0, 0, 0], [.5, 0, 0]) is None
    assert scene.segment_hit([0, 2, 0], [3, 2, 0]) is None


def test_body_envelope_detects_obstacle_missed_by_center_ray():
    face = np.array([[[1, .04, -.1], [1, .08, -.1], [1, .06, .1]]])
    scene = TriangleScene(face)
    assert scene.segment_hit([0, 0, 0], [2, 0, 0]) is None
    assert scene.corridor_hit([0, 0, 0], [2, 0, 0], .06, .01) is not None


def test_hangar_departure_preserves_vehicle_order_and_body_separation():
    path = Path(__file__).resolve().parents[1] / 'maps/rmuc_2027.json'
    config = json.loads(path.read_text(encoding='utf-8'))
    report = formation_separation(config)
    assert report['passed']
    assert report['minimum_distance_m'] >= .139
    offsets = config['team_mission']['formation_offsets']
    offsets[0].update(x=.35, y=0)
    offsets[3].update(x=-.35, y=0)
    crossing = formation_separation(config)
    assert not crossing['passed']
    assert crossing['minimum_distance_m'] < .087


def test_front_row_lands_before_rear_row_to_keep_waiting_positions_clear():
    path = Path(__file__).resolve().parents[1] / 'maps/rmuc_2027.json'
    config = json.loads(path.read_text(encoding='utf-8'))
    assert landing_waiting_separation(config)['passed']
    config['team_mission']['landing_order'] = [0, 1, 2, 3]
    crossing = landing_waiting_separation(config)
    assert not crossing['passed']
    assert crossing['minimum_distance_m'] < crossing['body_diameter_m']


def test_completed_holding_rows_leave_wide_vertical_clearance():
    path = Path(__file__).resolve().parents[1] / 'maps/rmuc_2027.json'
    config = json.loads(path.read_text(encoding='utf-8'))
    holding = {point['id']: point for point in config['team_mission']['completed_holding']}
    assert holding[2]['y'] - holding[0]['y'] >= .79
    assert holding[3]['y'] - holding[1]['y'] >= .79


def test_alignment_descent_clears_tray_rim_with_position_tolerance(tmp_path):
    path = Path(__file__).resolve().parents[1] / 'maps/rmuc_2027.json'
    config = json.loads(path.read_text(encoding='utf-8'))
    config['terrain_triangles_file'] = str((path.parent / config['terrain_triangles_file']).resolve())
    candidate = tmp_path / 'map.json'
    config['team_mission']['landing_alignment_height_m'] = .10
    candidate.write_text(json.dumps(config), encoding='utf-8')
    clear = check_map(candidate)
    assert clear['passed'], clear['failures']
    assert len(clear['landing_alignment_segments']) == 4

    config['team_mission']['landing_alignment_height_m'] = .06
    candidate.write_text(json.dumps(config), encoding='utf-8')
    blocked = check_map(candidate)
    assert not blocked['passed']
    assert any(row['kind'] == 'landing_alignment' for row in blocked['failures'])


def test_contact_approach_does_not_cross_waiting_members():
    path = Path(__file__).resolve().parents[1] / 'maps/rmuc_2027.json'
    config = json.loads(path.read_text(encoding='utf-8'))
    report = approach_waiting_separation(config)
    assert report['passed']
    assert report['minimum_distance_m'] >= .28
    assert len(report['segments']) == 12
    assert any(row['waiting_phase'] == 'completed_holding' for row in report['segments'])

    for point in config['team_mission']['assembly']:
        point.update(x=8.6 if point['id'] % 2 == 0 else 8.95,
                     y=-1.85 if point['id'] < 2 else -1.5)
    blocked = approach_waiting_separation(config)
    assert not blocked['passed']
    assert any(row['vehicle'] == 0 and row['waiting_vehicle'] == 3 and not row['passed']
               for row in blocked['segments'])
