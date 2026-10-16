"""2027 场地坐标、固定基地装甲及完整定位先验。"""

import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from simulation.import_sentry26 import read_stl
from simulation.launch_linux import prepare_world

ROOT = Path(__file__).resolve().parents[1]
MAP = ROOT / 'maps/rmuc_2027.json'
ASSET = ROOT / 'models/terrain/rmuc_2027'


def test_collision_parts_reconstruct_static_localization_prior():
    prior = np.fromfile(ASSET / 'terrain_triangles.f32', dtype='<f4').reshape(-1, 3, 3)
    fixed = read_stl(ASSET / 'terrain.stl')['vertices']
    target = read_stl(ASSET / 'moving/base_original_carriage.stl')['vertices']
    def rows(array):
        return np.sort(np.ascontiguousarray(array).reshape(-1, 9).view('V36').ravel())
    np.testing.assert_array_equal(rows(prior), rows(np.concatenate([fixed, target])))
    assert 0 < len(prior) <= 1_000_000


def test_new_world_uses_cad_base_armor_and_four_hangar_homes(tmp_path):
    name, config = prepare_world(tmp_path / 'world.sdf', MAP)
    assert name == 'rmuc_2027' and config['season'] == 2027
    world = ET.parse(tmp_path / 'world.sdf').getroot().find('world')
    terrain = [item.findtext('uri') for item in world.findall('include') if 'terrain' not in item.attrib]
    assert terrain.count('model://rmuc_2027') == 1
    assert not any('2026' in uri for uri in terrain)
    armor = world.find('model[@name="enemy_base_moving_armor"]')
    assert armor.find('joint[@name="armor_slide"]').get('type') == 'fixed'
    assert not armor.findall('plugin')
    assert config['moving_armor']['target_type'] == 'base_armor'
    assert config['team_mission']['target']['z'] < .5
    assert not config['team_mission']['green_guide']['enabled']
    assert config['team_mission']['stranded_ground_source'] == 'terrain'
    assert len(config['spawns']) == 4
    assert all(len(points) >= 5 for points in config['vehicle_waypoints'].values())
    assert all(1.6 < home['z'] < 1.7 for home in config['spawns'])
    camera = [float(v) for v in world.findtext('model[@name="home_overview"]/pose').split()]
    np.testing.assert_allclose(camera[:2], np.mean([[s['x'], s['y']] for s in config['spawns']], axis=0))


def test_pressure_surface_matches_original_cad_faces():
    from simulation.cad_field import transform_triangles
    cache = ROOT.parent / 'build/rmuc2027/cache/cad_mesh.npz'
    if not cache.exists():
        pytest.skip('CAD extraction cache required')
    config = json.loads(MAP.read_text(encoding='utf-8'))
    spec = config['cad_import']
    with np.load(cache, allow_pickle=False) as data:
        original = data['triangles_mm'][data['part_ids'] == 79]
    original = transform_triangles(original, spec['world_from_cad_m'])
    low, high = np.asarray(spec['target_bounds_m'])
    mask = (original.min(axis=1) >= low - 1e-6).all(axis=1) & (original.max(axis=1) <= high + 1e-6).all(axis=1)
    armor = spec['armor']
    mask &= (np.abs((original - armor['center_m']) @ np.asarray(armor['normal'])) < armor['surface_tolerance_m']).all(axis=1)
    actual = read_stl(ASSET / 'moving/armor_hit_surface.stl')['vertices']
    np.testing.assert_array_equal(actual, original[mask])
