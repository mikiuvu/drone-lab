"""三角网格的航段求交与机体净空。"""

import numpy as np

from simulation.check_map import TriangleScene


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
