"""新场地的高度色标和地形缓存。"""

import numpy as np

from app.simulation_reader import SimulationReader, terrain_image


def test_color_mapping_uses_map_ceiling():
    first = terrain_image(np.array([[1.0]]), 2)
    second = terrain_image(np.array([[2.0]]), 4)
    assert first.pixelColor(0, 0) == second.pixelColor(0, 0)


def test_color_scale_change_invalidates_image_cache(tmp_path):
    source = tmp_path / 'terrain.npz'
    np.savez_compressed(source, z=np.array([[1.0]]))
    config = dict(terrain_height_grid=str(source), terrain_height_max_m=2)
    reader = SimulationReader(tmp_path)
    first, error = reader.read_terrain(config)
    assert not error
    config['terrain_height_max_m'] = 4
    second, error = reader.read_terrain(config)
    assert not error
    assert first.pixelColor(0, 0) != second.pixelColor(0, 0)
