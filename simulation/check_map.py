"""检查静态地图中的起降净空、编队航路与装甲目标坐标。"""

import argparse
import itertools
import json
from pathlib import Path

import numpy as np


class TriangleScene:
    """按线段包围盒筛选候选三角形，再用重心坐标求交。"""

    def __init__(self, triangles, indices=None):
        self.triangles = np.asarray(triangles, dtype=float)
        self.low, self.high = self.triangles.min(axis=1), self.triangles.max(axis=1)
        self.indices = np.arange(len(triangles)) if indices is None else indices

    def segment_hit(self, start, end):
        start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
        ray = end - start
        length = np.linalg.norm(ray)
        if length < 1e-9:
            return None
        ray /= length
        low, high = np.minimum(start, end) - 1e-7, np.maximum(start, end) + 1e-7
        selected = (self.high >= low).all(axis=1) & (self.low <= high).all(axis=1)
        indices = np.flatnonzero(selected)
        tri = self.triangles[selected]
        e1, e2 = tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
        p = np.cross(ray, e2)
        determinant = np.einsum('ij,ij->i', e1, p)
        valid = np.abs(determinant) > 1e-12
        inverse = np.zeros_like(determinant)
        inverse[valid] = 1 / determinant[valid]
        s = start - tri[:, 0]
        u = np.einsum('ij,ij->i', s, p) * inverse
        q = np.cross(s, e1)
        v = q @ ray * inverse
        depth = np.einsum('ij,ij->i', e2, q) * inverse
        valid &= (u >= -1e-7) & (v >= -1e-7) & (u + v <= 1 + 1e-7) & (depth > 1e-6) & (depth <= length + 1e-7)
        if not valid.any():
            return None
        index = np.flatnonzero(valid)[np.argmin(depth[valid])]
        return {'triangle': int(self.indices[indices[index]]), 'distance_m': float(depth[index]), 'point_m': (start + ray * depth[index]).tolist()}

    def corridor_hit(self, start, end, radius, half_height, bounded=False):
        """沿机体中心、上下表面和圆周采样线检查航段。"""
        if not bounded:
            padding = np.array([radius, radius, half_height]) + 1e-7
            low, high = np.minimum(start, end) - padding, np.maximum(start, end) + padding
            selected = (self.high >= low).all(axis=1) & (self.low <= high).all(axis=1)
            return TriangleScene(self.triangles[selected], self.indices[selected]).corridor_hit(start, end, radius, half_height, bounded=True)
        offsets = [[0, 0, 0]]
        for z in (-half_height, half_height):
            offsets.append([0, 0, z])
            offsets.extend([[radius * np.cos(a), radius * np.sin(a), z] for a in np.linspace(0, 2 * np.pi, 12, endpoint=False)])
        for offset in offsets:
            hit = self.segment_hit(np.asarray(start) + offset, np.asarray(end) + offset)
            if hit:
                return hit
        return None


def formation_separation(config):
    """检查等比例推进的编队航段中心距；圆形机体使用地图接触半径。"""
    mission = config['team_mission']
    cruise = mission['cruise_z_m']
    paths = {}
    for spawn in config['spawns']:
        vehicle = spawn['id']
        offset = next(p for p in mission['formation_offsets'] if p['id'] == vehicle)
        assembly = next(p for p in mission['assembly'] if p['id'] == vehicle)
        paths[vehicle] = np.asarray(
            [[spawn['x'], spawn['y']]]
            + [[p['x'] + offset['x'], p['y'] + offset['y']]
               for p in mission['transit_waypoints']]
            + [[assembly['x'], assembly['y']]], dtype=float)
    checks = []
    diameter = 2 * mission['contact_radius_m']
    for first, second in itertools.combinations(paths, 2):
        a, b = paths[first], paths[second]
        for segment in range(len(a) - 1):
            relative = a[segment] - b[segment]
            change = (a[segment + 1] - a[segment]) - (b[segment + 1] - b[segment])
            norm = float(change @ change)
            fraction = float(np.clip(-(relative @ change) / norm, 0, 1)) if norm > 1e-12 else 0.0
            distance = float(np.linalg.norm(relative + fraction * change))
            checks.append({'vehicles': [first, second], 'segment': segment,
                           'fraction': fraction, 'distance_m': distance,
                           'body_gap_m': distance - diameter, 'passed': distance >= diameter})
    return {'scope': 'synchronous_linear_transit', 'height_m': cruise, 'body_diameter_m': diameter,
            'minimum_distance_m': min((row['distance_m'] for row in checks), default=None),
            'segments': checks, 'passed': all(row['passed'] for row in checks)}


def landing_waiting_separation(config):
    """检查逐架返航的巡航高度接近线段与其余等候机的中心距。"""
    mission = config['team_mission']
    order = mission.get('landing_order', mission['order']) if mission.get('sequential_landing') else []
    homes = {p['id']: np.array([p['x'], p['y']]) for p in config['spawns']}
    holds = {}
    for vehicle in order:
        if mission['transit_waypoints']:
            point = mission['transit_waypoints'][0]
            offset = next(p for p in mission['formation_offsets'] if p['id'] == vehicle)
            holds[vehicle] = np.array([point['x'] + offset['x'], point['y'] + offset['y']])
        else:
            point = next(p for p in mission['assembly'] if p['id'] == vehicle)
            holds[vehicle] = np.array([point['x'], point['y']])
    diameter = 2 * mission['contact_radius_m']
    checks = []
    for slot, vehicle in enumerate(order):
        change = homes[vehicle] - holds[vehicle]
        norm = float(change @ change)
        for waiting in order[slot + 1:]:
            relative = holds[vehicle] - holds[waiting]
            fraction = float(np.clip(-(relative @ change) / norm, 0, 1)) if norm > 1e-12 else 0.0
            distance = float(np.linalg.norm(relative + fraction * change))
            checks.append({'vehicle': vehicle, 'waiting_vehicle': waiting, 'fraction': fraction,
                           'distance_m': distance, 'passed': distance >= diameter})
    return {'scope': 'sequential_cruise_approach', 'order': order, 'body_diameter_m': diameter,
            'minimum_distance_m': min((row['distance_m'] for row in checks), default=None),
            'segments': checks, 'passed': all(row['passed'] for row in checks)}


def check_map(path, margin=.02):
    path = Path(path)
    config = json.loads(path.read_text(encoding='utf-8'))
    triangles = np.fromfile(path.parent / config['terrain_triangles_file'], dtype='<f4').reshape(-1, 3, 3)
    scene = TriangleScene(triangles)
    mission = config['team_mission']
    radius = mission['contact_radius_m'] + margin
    half_height = mission['body_clearance_m'] + margin
    cruise = mission['cruise_z_m']
    failures, homes, routes, alignment_routes = [], [], [], []
    for spawn in config['spawns']:
        i = spawn['id']
        start = np.array([spawn[k] for k in ('x', 'y', 'z')])
        ground = scene.segment_hit(start, start - [0, 0, 10])
        result = {'id': i, 'ground_hit': ground}
        if not ground or abs(ground['point_m'][2] - mission['home_ground_z_m']) > .005:
            failures.append(dict(kind='home_ground', **result))
        support = []
        for angle in np.linspace(0, 2 * np.pi, 12, endpoint=False):
            point = start + [mission['contact_radius_m'] * np.cos(angle),
                             mission['contact_radius_m'] * np.sin(angle), 0]
            hit = scene.segment_hit(point, point - [0, 0, 10])
            support.append(hit)
        result['support_hits'] = support
        if any(hit is None or abs(hit['point_m'][2] - mission['home_ground_z_m']) > .005 for hit in support):
            failures.append({'kind': 'home_body_support', 'id': i})
        up = start.copy()
        up[2] = cruise
        hit = scene.corridor_hit(start, up, radius, half_height)
        result['takeoff_hit'] = hit
        if hit:
            failures.append({'kind': 'takeoff', 'id': i, 'hit': hit})
        alignment_height = mission.get('landing_alignment_height_m', 0)
        if mission.get('sequential_landing') and alignment_height > 0:
            alignment = [start[0], start[1], mission['home_ground_z_m'] + alignment_height]
            # 水平对准误差计入包络，避免中心线通过而机体擦到托盘边框。
            alignment_radius = radius + mission.get('landing_alignment_tolerance_m', 0)
            hit = scene.corridor_hit(up, alignment, alignment_radius, half_height)
            alignment_routes.append({'id': i, 'start_m': up.tolist(), 'end_m': alignment,
                                     'radius_m': alignment_radius, 'half_height_m': half_height,
                                     'hit': hit})
            if hit:
                failures.append({'kind': 'landing_alignment', 'id': i, 'hit': hit})
        homes.append(result)
        offset = next(p for p in mission['formation_offsets'] if p['id'] == i)
        points = [up]
        points += [[p['x'] + offset['x'], p['y'] + offset['y'], p.get('z', cruise)] for p in mission['transit_waypoints']]
        assembly = next(p for p in mission['assembly'] if p['id'] == i)
        points.append([assembly['x'], assembly['y'], assembly.get('z', cruise)])
        for number, (a, b) in enumerate(itertools.pairwise(points)):
            hit = scene.corridor_hit(a, b, radius, half_height)
            routes.append({'id': i, 'segment': number, 'start_m': list(a), 'end_m': list(b), 'hit': hit})
            if hit:
                failures.append({'kind': 'transit', 'id': i, 'segment': number, 'hit': hit})
    target = np.array([mission['target'][k] for k in ('x', 'y', 'z')])
    approach = np.array([mission['approach'][k] for k in ('x', 'y', 'z')])
    direction = target - approach
    direction /= np.linalg.norm(direction)
    target_hit = scene.segment_hit(approach, target + direction * .03)
    if not target_hit or np.linalg.norm(np.asarray(target_hit['point_m']) - target) > .01:
        failures.append({'kind': 'target_surface', 'hit': target_hit})
    retreat_routes = []
    if 'retreat_exit' in mission:
        exit_point = [mission['retreat_exit'][key] for key in ('x', 'y', 'z')]
        segments = [('approach_to_exit', approach, exit_point)]
        segments.extend((f'exit_to_hold_{p["id"]}', exit_point, [p['x'], p['y'], cruise])
                        for p in mission['completed_holding'])
        for name, start, end in segments:
            hit = scene.corridor_hit(start, end, radius, half_height)
            retreat_routes.append({'segment': name, 'start_m': list(start), 'end_m': list(end), 'hit': hit})
            if hit:
                failures.append({'kind': 'retreat_exit', 'segment': name, 'hit': hit})
    separation = formation_separation(config)
    failures.extend({'kind': 'formation_separation', **row} for row in separation['segments'] if not row['passed'])
    landing = landing_waiting_separation(config)
    failures.extend({'kind': 'landing_waiting_separation', **row} for row in landing['segments'] if not row['passed'])
    return {'map': config['name'], 'scope': 'static_mesh_clearance', 'margin_m': margin,
                'triangle_count': len(triangles), 'homes': homes, 'transit_segments': routes,
                'formation_separation': separation,
                'landing_waiting_separation': landing,
                'landing_alignment_segments': alignment_routes,
                'retreat_exit_segments': retreat_routes,
                'target_hit': target_hit, 'failures': failures, 'passed': not failures}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, default=Path(__file__).resolve().parent / 'maps/rmuc_2027.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = check_map(args.map)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'passed': report['passed'], 'failures': report['failures']}, ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
