"""CAD 基地装甲固定安装与资源版本校验。"""

import hashlib
import json
import xml.etree.ElementTree as ET

import pytest

from simulation.moving_armor import add_moving_armor


def scene(tmp_path, digest=None):
    source = tmp_path / 'terrain_triangles.f32'
    source.write_bytes(b'geometry')
    folder = tmp_path / 'moving'
    folder.mkdir()
    report = dict(static_sha256=digest or hashlib.sha256(source.read_bytes()).hexdigest(),
                  target_center_m=[7, 2, .35], max_half_travel_m=0,
                  material_faces=dict(green=0))
    (folder / 'partition.json').write_text(json.dumps(report), encoding='utf-8')
    world = ET.Element('world')
    ET.SubElement(ET.SubElement(world, 'include'), 'uri').text = 'model://rmuc_2027'
    config = dict(moving_armor=dict(enabled=True, source='cad_mesh_partition', motion_profile='fixed'))
    return world, config, source


def test_fixed_base_armor_keeps_new_terrain_and_has_no_rail_motion(tmp_path):
    world, config, source = scene(tmp_path)
    armor = add_moving_armor(world, config, tmp_path / 'runtime', source)
    assert world.findtext('include/uri') == 'model://rmuc_2027'
    assert len(world.findall('model')) == 1
    assert armor.find('joint[@name="armor_slide"]').get('type') == 'fixed'
    assert not armor.findall('plugin')
    assert armor.findtext('link[@name="armor"]/inertial/pose') == '7 2 0.35 0 0 0'
    assert armor.find('link[@name="armor"]/visual[@name="green_dart_guide"]') is None
    assert armor.findtext('.//sensor/contact/collision') == 'armor_plate'
    assert config['terrain_triangles_file'] == str(source.resolve())


def test_stale_cad_partition_rejected_before_world_mutation(tmp_path):
    world, config, source = scene(tmp_path, digest='wrong')
    with pytest.raises(ValueError, match='CAD'):
        add_moving_armor(world, config, tmp_path / 'runtime', source)
    assert not world.findall('model')
