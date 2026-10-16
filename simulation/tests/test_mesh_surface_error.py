"""压缩网格的表面误差距离。"""

import numpy as np
import pytest

from simulation.mesh_simplify import (
    connected_meshes,
    simplify_components,
    surface_distances,
)

pytest.importorskip('scipy')
pytest.importorskip('rtree')


def test_distances_cover_face_interior_edge_and_vertex():
    triangle = np.array([[[0, 0, 0], [1, 0, 0], [0, 1, 0]]], dtype=float)
    points = np.array([[.2, .2, .3], [.5, -.2, 0], [-.3, -.4, 0]])
    np.testing.assert_allclose(surface_distances(points, triangle), [.3, .2, .5])


def test_large_near_face_is_not_hidden_by_small_far_centroids():
    large = np.array([[[0, 0, 0], [100, 0, 0], [0, 100, 0]]])
    small = np.repeat([[[.1, .1, .1], [.11, .1, .1], [.1, .11, .1]]], 80, axis=0)
    np.testing.assert_allclose(surface_distances(np.array([[.1, .1, .001]]),
        np.concatenate([large, small])), [.001])


def test_separated_closed_parts_survive_mesh_reduction():
    pytest.importorskip('fast_simplification')
    vertices = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]], dtype=float)
    faces = np.array([[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],
                      [1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]], dtype=np.int32)
    meshes = connected_meshes(np.concatenate([vertices, vertices+[10,0,0]]), np.concatenate([faces, faces+8]))
    assert len(meshes) == 2
    output = simplify_components(meshes, .01)
    assert (output[:, :, 0] < 2).any() and (output[:, :, 0] > 9).any()
