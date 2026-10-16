"""地图高度栅格的坐标范围与重叠表面。"""

import numpy as np
import pytest

from simulation.import_sentry26 import height_grid


def test_nonlegacy_bounds_and_top_surface():
    square = np.array([[[20, 30, 0], [21, 30, 0], [20, 31, 0]],
                       [[21, 30, 0], [21, 31, 0], [20, 31, 0]]], dtype=float)
    top = square + [0, 0, 0.5]
    x, y, z = height_grid(np.concatenate([top, square]), 0.25, (20, 21, 30, 31))
    np.testing.assert_allclose(x, [20, 20.25, 20.5, 20.75, 21])
    np.testing.assert_allclose(y, [30, 30.25, 30.5, 30.75, 31])
    np.testing.assert_allclose(z, 0.5)


@pytest.mark.parametrize('step,bounds', [(0, None), (float('nan'), None),
    (0.1, (1, 0, 0, 1)), (0.1, (0, 1, 0, float('inf')))])
def test_invalid_grid_rejected(step, bounds):
    with pytest.raises(ValueError):
        height_grid(np.empty((0, 3, 3)), step, bounds)
