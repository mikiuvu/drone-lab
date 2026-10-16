"""三角网格的航段求交与机体净空。"""

import json
from pathlib import Path

import numpy as np

from simulation.check_map import (
    TriangleScene,
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
    offsets[1].update(x=-.35, y=.35)
    offsets[2].update(x=.35, y=-.35)
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
    assert crossing['minimum_distance_m'] < .05
