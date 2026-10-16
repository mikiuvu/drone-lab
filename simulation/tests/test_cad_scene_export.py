"""CAD 场景导出的材质、碰撞网格与定位先验一致性。"""

import xml.etree.ElementTree as ET

import numpy as np

from simulation.import_rmuc2027 import export_armor, export_scene
from simulation.import_sentry26 import read_stl


def test_static_collision_visuals_and_prior_share_identical_vertices(tmp_path):
    triangles = np.array([[[20, 30, 0], [21, 30, 0], [20, 31, 0]],
                          [[21, 30, 0], [21, 31, 0], [20, 31, 0]]], dtype='<f4')
    parts = [{'id': 0, 'rgb': [1., 0., 0.]}, {'id': 1, 'rgb': [0., 0., 1.]}]
    config = {'name': 'field', 'version': '1.0.0', 'cad_import': {},
                  'bounds': {'x_min': 20, 'x_max': 21, 'y_min': 30, 'y_max': 31}}
    report = export_scene(triangles, np.array([0, 1]), parts, config, tmp_path)
    collision = read_stl(tmp_path / 'terrain.stl')['vertices']
    prior = np.fromfile(tmp_path / 'terrain_triangles.f32', dtype='<f4').reshape(-1, 3, 3)
    np.testing.assert_array_equal(prior, triangles)
    np.testing.assert_array_equal(collision, triangles)
    visuals = np.concatenate([read_stl(path)['vertices'] for path in sorted(tmp_path.glob('visual_*.stl'))])
    np.testing.assert_array_equal(visuals, triangles)
    model = ET.parse(tmp_path / 'model.sdf')
    assert model.findtext('.//collision/geometry/mesh/uri') == 'model://field/terrain.stl'
    assert len(model.findall('.//visual')) == report['visual_material_count'] == 2
    with np.load(tmp_path / 'terrain_height_grid.npz', allow_pickle=False) as data:
        np.testing.assert_allclose(data['z'], 0)
        np.testing.assert_allclose(data['x'][[0, -1]], [20, 21])


def test_fixed_armor_has_only_original_front_pressure_surface_and_no_dart_guide(tmp_path):
    panel = np.array([[[2, -.15, .35], [2, .15, .35], [2, -.15, .45]],
                      [[2, .15, .35], [2, .15, .45], [2, -.15, .45]]], dtype='<f4')
    triangles = np.concatenate([panel, panel + [.02, 0, 0]])
    (tmp_path / 'terrain_triangles.f32').write_bytes(b'static')
    config = {'moving_armor': {'target_type': 'base_armor'}, 'cad_import': {'armor': {
        'center_m': [2, 0, .4], 'normal': [-1, 0, 0], 'panel_up': [0, 0, 1], 'panel_part_ids': [0],
        'strip_height_m': .055, 'strip_width_m': .005, 'strip_separation_m': .135}}}
    export_armor(triangles, np.array([0, 0, 1, 1]), np.ones(4, dtype=bool), config, tmp_path)
    folder = tmp_path / 'moving'
    np.testing.assert_array_equal(read_stl(folder / 'armor_hit_surface.stl')['vertices'], panel)
    assert not (folder / 'carriage_green_guide.stl').exists()
    body, blue = [read_stl(folder / name)['vertices'] for name in ['carriage_body_surfaces.stl', 'carriage_blue_surfaces.stl']]
    def area(faces):
        return np.linalg.norm(np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0]), axis=1).sum() / 2
    np.testing.assert_allclose(area(body) + area(blue), area(triangles), rtol=1e-6)


def test_fixed_target_region_remains_in_localization_prior(tmp_path):
    floor = np.array([[[0, -1, 0], [3, -1, 0], [0, 1, 0]],
                      [[3, -1, 0], [3, 1, 0], [0, 1, 0]]])
    panel = np.array([[[2, -.15, .35], [2, .15, .35], [2, -.15, .45]],
                      [[2, .15, .35], [2, .15, .45], [2, -.15, .45]]])
    triangles = np.concatenate([floor, panel, panel + [.02, 0, 0]]).astype('<f4')
    config = {'name': 'field', 'version': '1.0.0',
        'bounds': {'x_min': 0, 'x_max': 3, 'y_min': -1, 'y_max': 1},
        'moving_armor': {'target_type': 'base_armor', 'motion_profile': 'fixed'},
        'cad_import': {'moving_part_ids': [0], 'target_bounds_m': [[1.999, -.16, .34], [2.021, .16, .46]],
            'armor': {'center_m': [2, 0, .4], 'normal': [-1, 0, 0], 'panel_up': [0, 0, 1], 'panel_part_ids': [0],
                       'strip_height_m': .055, 'strip_width_m': .005, 'strip_separation_m': .135}}}
    report = export_scene(triangles, np.zeros(6, dtype=int), [{'id': 0, 'rgb': [.5, .5, .5]}], config, tmp_path)
    assert report['static_triangle_count'] == 6
    assert report['terrain_mesh_triangle_count'] == 2
    assert report['target_triangle_count'] == 4 and report['moving_triangle_count'] == 0
    prior = np.fromfile(tmp_path / 'terrain_triangles.f32', dtype='<f4').reshape(-1, 3, 3)
    np.testing.assert_array_equal(prior, triangles)
    np.testing.assert_array_equal(read_stl(tmp_path / 'terrain.stl')['vertices'], floor)
