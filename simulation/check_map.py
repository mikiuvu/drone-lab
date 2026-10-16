"""检查静态地图中的起降净空、编队航路与装甲目标坐标。"""

import argparse
import json
from pathlib import Path

import numpy as np


class TriangleScene:
    """按线段包围盒筛选候选三角形，再用重心坐标求交。"""

    def __init__(self, triangles):
        self.triangles = np.asarray(triangles, dtype=float)
        self.low, self.high = self.triangles.min(axis=1), self.triangles.max(axis=1)

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
        return dict(triangle=int(indices[index]), distance_m=float(depth[index]), point_m=(start + ray * depth[index]).tolist())

    def corridor_hit(self, start, end, radius, half_height):
        """沿机体中心、上下表面和圆周采样线检查航段。"""
        offsets = [[0, 0, 0]]
        for z in (-half_height, half_height):
            offsets.append([0, 0, z])
            offsets.extend([[radius * np.cos(a), radius * np.sin(a), z] for a in np.linspace(0, 2 * np.pi, 12, endpoint=False)])
        for offset in offsets:
            hit = self.segment_hit(np.asarray(start) + offset, np.asarray(end) + offset)
            if hit:
                return hit
        return None


def check_map(path, margin=.02):
    path = Path(path)
    config = json.loads(path.read_text(encoding='utf-8'))
    triangles = np.fromfile(path.parent / config['terrain_triangles_file'], dtype='<f4').reshape(-1, 3, 3)
    scene = TriangleScene(triangles)
    mission = config['team_mission']
    radius = mission['contact_radius_m'] + margin
    half_height = mission['body_clearance_m'] + margin
    cruise = mission['cruise_z_m']
    failures, homes, routes = [], [], []
    for spawn in config['spawns']:
        i = spawn['id']
        start = np.array([spawn[k] for k in ('x', 'y', 'z')])
        ground = scene.segment_hit(start, start - [0, 0, 10])
        result = dict(id=i, ground_hit=ground)
        if not ground or abs(ground['point_m'][2] - mission['home_ground_z_m']) > .005:
            failures.append(dict(kind='home_ground', **result))
        up = start.copy()
        up[2] = cruise
        hit = scene.corridor_hit(start, up, radius, half_height)
        result['takeoff_hit'] = hit
        if hit:
            failures.append(dict(kind='takeoff', id=i, hit=hit))
        homes.append(result)
        offset = next(p for p in mission['formation_offsets'] if p['id'] == i)
        points = [up]
        points += [[p['x'] + offset['x'], p['y'] + offset['y'], p.get('z', cruise)] for p in mission['transit_waypoints']]
        assembly = next(p for p in mission['assembly'] if p['id'] == i)
        points.append([assembly['x'], assembly['y'], assembly.get('z', cruise)])
        for number, (a, b) in enumerate(zip(points, points[1:])):
            hit = scene.corridor_hit(a, b, radius, half_height)
            routes.append(dict(id=i, segment=number, start_m=list(a), end_m=list(b), hit=hit))
            if hit:
                failures.append(dict(kind='transit', id=i, segment=number, hit=hit))
    target = np.array([mission['target'][k] for k in ('x', 'y', 'z')])
    approach = np.array([mission['approach'][k] for k in ('x', 'y', 'z')])
    direction = target - approach
    direction /= np.linalg.norm(direction)
    target_hit = scene.segment_hit(approach, target + direction * .03)
    if not target_hit or np.linalg.norm(np.asarray(target_hit['point_m']) - target) > .01:
        failures.append(dict(kind='target_surface', hit=target_hit))
    return dict(map=config['name'], scope='static_mesh_clearance', margin_m=margin,
                triangle_count=len(triangles), homes=homes, transit_segments=routes,
                target_hit=target_hit, failures=failures, passed=not failures)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, default=Path(__file__).resolve().parent / 'maps/rmuc_2027.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = check_map(args.map)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(passed=report['passed'], failures=report['failures']), ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
