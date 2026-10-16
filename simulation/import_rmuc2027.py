"""从 RMUC 2027 STEP 装配体生成 Gazebo 场地与定位三角网格。"""

import argparse
import json
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

try:
    from simulation.cad_field import extract_step, sha256, transform_triangles
    from simulation.import_sentry26 import height_grid
except ModuleNotFoundError:
    from cad_field import extract_step, sha256, transform_triangles
    from import_sentry26 import height_grid

ROOT = Path(__file__).resolve().parent


def write_stl(path, triangles):
    triangles = np.asarray(triangles, dtype='<f4')
    if not len(triangles):
        raise ValueError(f'Empty mesh: {path}')
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-20)
    records = np.zeros(len(triangles), dtype=[('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])
    records['normal'], records['v'] = normals, triangles
    Path(path).write_bytes(b'RMUC2027 V1.0.0 CAD assembly; units m'.ljust(80, b' ') + struct.pack('<I', len(triangles)) + records.tobytes())


def add(parent, tag, text=None, **attributes):
    result = ET.SubElement(parent, tag, attributes)
    if text is not None:
        result.text = str(text)
    return result


def mesh_node(parent, kind, name, uri, rgb=None):
    element = add(parent, kind, name=name)
    add(add(add(element, 'geometry'), 'mesh'), 'uri', uri)
    if rgb is not None:
        material = add(element, 'material')
        for key in ('ambient', 'diffuse'):
            add(material, key, ' '.join(map(str, [*rgb, 1])))
    return element


def export_scene(triangles, part_ids, parts, config, output):
    """按 CAD 实例分离固定场地与活动部件，所有资源使用相同米制坐标。"""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    specification = config['cad_import']
    moving_ids = specification.get('moving_part_ids', [])
    excluded_ids = specification.get('excluded_part_ids', [])
    if set(moving_ids) & set(excluded_ids):
        raise ValueError('Moving and excluded CAD parts overlap')
    available = {p['id'] for p in parts}
    if not set(moving_ids + excluded_ids) <= available:
        raise ValueError('Unknown CAD part index')
    kept = ~np.isin(part_ids, excluded_ids)
    moving = kept & np.isin(part_ids, moving_ids)
    if 'target_bounds_m' in specification:
        low, high = np.asarray(specification['target_bounds_m'])
        moving &= ((triangles.min(axis=1) >= low - 1e-6).all(axis=1)
                   & (triangles.max(axis=1) <= high + 1e-6).all(axis=1))
    fixed = kept & ~moving
    stationary = config.get('moving_armor', {}).get('motion_profile') == 'fixed'
    prior = kept if stationary else fixed
    if not 0 < prior.sum() <= 1_000_000:
        raise ValueError(f'Static terrain has {prior.sum()} faces; supported range is 1..1000000')
    write_stl(output / 'terrain.stl', triangles[fixed])
    triangles[prior].astype('<f4').tofile(output / 'terrain_triangles.f32')
    tree = ET.ElementTree(ET.Element('sdf', version='1.9'))
    model = add(tree.getroot(), 'model', name=config['name'])
    add(model, 'static', 'true')
    link = add(model, 'link', name='terrain')
    collision = mesh_node(link, 'collision', 'terrain_collision', f'model://{config["name"]}/terrain.stl')
    contact = add(add(collision, 'surface'), 'contact')
    add(contact, 'collide_bitmask', 1)
    palette = {}
    for part in parts:
        key = tuple(round(c, 6) for c in part['rgb'])
        palette.setdefault(key, []).append(part['id'])
    visual_count = 0
    for rgb, indices in palette.items():
        selection = fixed & np.isin(part_ids, indices)
        if not selection.any():
            continue
        name = f'visual_{visual_count:03d}'
        write_stl(output / f'{name}.stl', triangles[selection])
        mesh_node(link, 'visual', name, f'model://{config["name"]}/{name}.stl', rgb)
        visual_count += 1
    ET.indent(tree)
    tree.write(output / 'model.sdf', encoding='utf-8', xml_declaration=True)
    (output / 'model.config').write_text(
        f'<model><name>{config["name"]}</name><version>{config["version"]}</version><sdf version="1.9">model.sdf</sdf></model>', encoding='utf-8')
    if moving.any():
        export_armor(triangles, part_ids, moving, config, output)
    bounds = config['bounds']
    x, y, z = height_grid(triangles[prior], step=.1, bounds=[bounds[k] for k in ('x_min', 'x_max', 'y_min', 'y_max')])
    np.savez_compressed(output / 'terrain_height_grid.npz', x=x, y=y, z=z)
    return dict(original_triangle_count=len(triangles), retained_triangle_count=int(kept.sum()),
                static_triangle_count=int(prior.sum()), terrain_mesh_triangle_count=int(fixed.sum()),
                target_triangle_count=int(moving.sum()), moving_triangle_count=0 if stationary else int(moving.sum()),
                excluded_triangle_count=int((~kept).sum()), visual_material_count=visual_count,
                world_bounds_m=[triangles[kept].min(axis=(0, 1)).tolist(), triangles[kept].max(axis=(0, 1)).tolist()],
                derived_sha256=sha256(output / 'terrain.stl'), static_sha256=sha256(output / 'terrain_triangles.f32'))


def export_armor(triangles, part_ids, moving, config, output):
    """沿原板面分配装甲压力面与灯条材质，保留原滑台壳体碰撞。"""
    try:
        from simulation.base_mesh import target_surfaces
    except ModuleNotFoundError:
        from base_mesh import target_surfaces
    spec = dict(config['cad_import']['armor'])
    spec.setdefault('guide_center_m', [0, 0, 0])
    spec.setdefault('guide_normal', [1, 0, 0])
    spec.setdefault('guide_up', [0, 0, 1])
    spec.setdefault('guide_diameter_m', .055)
    center, normal = np.asarray(spec['center_m']), np.asarray(spec['normal'])
    panel = moving & np.isin(part_ids, spec['panel_part_ids'])
    distances = (triangles - center) @ normal
    panel &= (np.abs(distances) < spec.get('surface_tolerance_m', .0001)).all(axis=1)
    guide_center, guide_normal = np.asarray(spec['guide_center_m']), np.asarray(spec['guide_normal'])
    guide = moving & np.isin(part_ids, spec.get('guide_part_ids', []))
    guide &= (np.abs((triangles - guide_center) @ guide_normal) < spec.get('surface_tolerance_m', .0001)).all(axis=1)
    if not panel.any() or (spec.get('guide_part_ids') and not guide.any()):
        raise ValueError('CAD target or guide surface is missing')
    surfaces = dict(spec, panel_surface_faces=np.flatnonzero(panel).tolist(), guide_surface_faces=np.flatnonzero(guide).tolist())
    blue, panel_body, green, guide_body = target_surfaces(triangles, surfaces)
    body = np.concatenate([triangles[moving & ~(panel | guide)], panel_body, guide_body])
    folder = output / 'moving'
    folder.mkdir(exist_ok=True)
    for name, faces in [('base_original_carriage', triangles[moving]), ('carriage_body_surfaces', body),
                        ('carriage_blue_surfaces', blue), ('carriage_green_guide', green),
                        ('armor_hit_surface', triangles[panel]), ('carriage_housing', triangles[moving & ~panel])]:
        if len(faces):
            write_stl(folder / f'{name}.stl', faces)
    report = dict(source_faces=len(triangles), fixed_faces=int((~moving).sum()), moving_faces=int(moving.sum()),
                  target_center_m=spec['center_m'], moving_bounds_m=[triangles[moving].min(axis=(0, 1)).tolist(), triangles[moving].max(axis=(0, 1)).tolist()],
                  target_type=config['moving_armor']['target_type'], max_half_travel_m=spec.get('max_half_travel_m', 0),
                  static_sha256=sha256(output / 'terrain_triangles.f32'),
                  zero_pose_geometry_identical=True,
                  material_faces=dict(body=len(body), blue=len(blue), green=len(green)))
    (folder / 'partition.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')


def export_world(config):
    tree = ET.parse(ROOT / 'worlds/rmuc_2026_sentry26.sdf')
    world = tree.getroot().find('world')
    world.set('name', config['name'])
    world.find('include/uri').text = f'model://{config["name"]}'
    tree.write(ROOT / 'worlds' / f'{config["name"]}.sdf', encoding='utf-8', xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, default=ROOT / 'maps/rmuc_2027.json')
    parser.add_argument('--source', type=Path)
    parser.add_argument('--cache', type=Path, default=ROOT.parent / 'build/rmuc2027/cache')
    parser.add_argument('--extract', action='store_true', help='重新展开源 STEP 装配体')
    parser.add_argument('--extract-only', action='store_true')
    args = parser.parse_args()
    config = json.loads(args.map.read_text(encoding='utf-8'))
    spec = config['cad_import']
    source = args.source or ROOT.parent / spec['source_file']
    if sha256(source) != spec['source_sha256']:
        raise ValueError('STEP source hash does not match the map specification')
    if args.extract or args.extract_only or not (args.cache / 'manifest.json').exists():
        extract_step(source, args.cache, spec['deflection_m'], healing=spec.get('healing', True))
    if args.extract_only:
        return
    cache_manifest = json.loads((args.cache / 'manifest.json').read_text(encoding='utf-8'))
    if (cache_manifest['source_sha256'] != spec['source_sha256']
            or cache_manifest['deflection_m'] != spec['deflection_m']
            or cache_manifest.get('healing', True) != spec.get('healing', True)):
        raise ValueError('CAD cache does not match the source or mesh tolerance')
    parts = json.loads((args.cache / 'parts.json').read_text(encoding='utf-8'))
    with np.load(args.cache / 'cad_mesh.npz', allow_pickle=False) as data:
        triangles = transform_triangles(data['triangles_mm'], spec['world_from_cad_m'])
        part_ids = data['part_ids']
    output = (args.map.parent / config['terrain_triangles_file']).resolve().parent
    metadata = export_scene(triangles, part_ids, parts, config, output)
    metadata.update(season=2027, version=config['version'], source_asset=spec['source_file'],
                    source_sha256=spec['source_sha256'], occt_transfer_unit='mm',
                    world_from_cad_m=spec['world_from_cad_m'], deflection_m=spec['deflection_m'], healing=spec.get('healing', True),
                    excluded_part_ids=spec.get('excluded_part_ids', []), moving_part_ids=spec.get('moving_part_ids', []),
                    triangle_format='little-endian float32 Nx3x3, 36 bytes/triangle')
    (output / 'provenance.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    (ROOT / 'docs/rmuc2027-import.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    export_world(config)
    print(json.dumps(metadata, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
