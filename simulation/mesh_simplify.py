"""按 CAD 部件压缩曲面网格，保留起降平台与装甲压力面的原始顶点。"""

import itertools

import numpy as np


def surface_distances(points, triangles):
    """最近顶点给出距离上界，再按三角形包围盒完整筛选求最近表面。"""
    from rtree import index
    from scipy.spatial import cKDTree
    properties = index.Property()
    properties.dimension = 3
    low, high = triangles.min(axis=1), triangles.max(axis=1)
    boxes = index.Index(((i, (*a, *b), None) for i, (a, b) in enumerate(zip(low, high))), properties=properties)
    limits, _ = cKDTree(triangles.reshape(-1, 3)).query(points)
    point_ids, triangle_ids = [], []
    for i, (point, distance) in enumerate(zip(points, limits)):
        radius = distance + 1e-7
        candidates = list(boxes.intersection((*tuple(point - radius), *tuple(point + radius))))
        point_ids.extend([i] * len(candidates))
        triangle_ids.extend(candidates)
    result = np.full(len(points), np.inf)
    for start in range(0, len(point_ids), 50000):
        selected = np.asarray(point_ids[start:start + 50000])
        p = points[selected]
        tri = triangles[triangle_ids[start:start + 50000]]
        a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
        e1, e2, w = b - a, c - a, p - a
        def dot(x, y):
            return np.einsum('...i,...i->...', x, y)
        d00, d01, d11 = dot(e1, e1), dot(e1, e2), dot(e2, e2)
        d20, d21 = dot(w, e1), dot(w, e2)
        determinant = d00 * d11 - d01 * d01
        valid = determinant > 1e-24
        inv = np.zeros_like(determinant)
        inv[valid] = 1 / determinant[valid]
        u, v = (d11 * d20 - d01 * d21) * inv, (d00 * d21 - d01 * d20) * inv
        squared = dot(w - u[..., None] * e1 - v[..., None] * e2,
                      w - u[..., None] * e1 - v[..., None] * e2)
        squared[~(valid & (u >= 0) & (v >= 0) & (u + v <= 1))] = np.inf
        for x, y in ((a, b), (b, c), (c, a)):
            edge, offset = y - x, p - x
            t = np.clip(dot(offset, edge) / np.maximum(dot(edge, edge), 1e-24), 0, 1)
            delta = offset - t[..., None] * edge
            squared = np.minimum(squared, dot(delta, delta))
        np.minimum.at(result, selected, np.sqrt(squared))
    boxes.close()
    return result


def sampled_error(source, result, seed=2027, samples=250):
    rng = np.random.default_rng(seed)
    maxima, means = [], []
    for first, second in ((source, result), (result, source)):
        indices = rng.choice(len(first), size=min(samples, len(first)), replace=False)
        points = np.concatenate([first[indices].mean(axis=1), first[indices, 0]])
        distance = surface_distances(points, second)
        maxima.append(float(distance.max()))
        means.append(float(distance.mean()))
    return max(maxima), max(means)


def connected_meshes(vertices, faces):
    """分离不连通的壳体，避免全局压缩把细小独立零件整体删除。"""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    rows = np.concatenate([faces[:, 0], faces[:, 1]])
    cols = np.concatenate([faces[:, 1], faces[:, 2]])
    graph = coo_matrix((np.ones(len(rows), dtype=bool), (rows, cols)), shape=(len(vertices), len(vertices))).tocsr()
    _, labels = connected_components(graph, directed=False)
    groups = labels[faces[:, 0]]
    order = np.argsort(groups, kind='stable')
    boundaries = np.r_[0, np.flatnonzero(np.diff(groups[order])) + 1, len(order)]
    result = []
    for low, high in itertools.pairwise(boundaries):
        selected = faces[order[low:high]]
        used, inverse = np.unique(selected, return_inverse=True)
        result.append((vertices[used], inverse.reshape(-1, 3).astype(np.int32)))
    return result


def simplify_components(meshes, fraction):
    import fast_simplification as fs
    result = []
    for vertices, faces in meshes:
        count = max(8, int(len(faces) * fraction))
        if count < len(faces):
            v, f = fs.simplify(vertices, faces, target_count=count, agg=5., preserve_border=False)
            result.append(v[f] if len(f) else vertices[faces])
        else:
            result.append(vertices[faces])
    return np.concatenate(result)


def simplify_field(triangles, part_ids, specification):
    import fast_simplification as fs
    settings = specification['simplification']
    fraction = settings['retained_fraction']
    tolerance = settings['sampled_error_tolerance_m']
    preserve = set(settings.get('preserve_part_ids', []))
    chunks, ids, reports = [], [], []
    low, high = np.asarray(specification['target_bounds_m'])
    protected = ((triangles.min(axis=1) >= low - 1e-6).all(axis=1)
                 & (triangles.max(axis=1) <= high + 1e-6).all(axis=1))
    protected &= np.isin(part_ids, specification['moving_part_ids'])
    for part in np.unique(part_ids):
        mask = part_ids == part
        original = triangles[mask].astype(float)
        fixed, active = original[protected[mask]], original[~protected[mask]]
        result, maximum, mean = active, 0., 0.
        if part not in preserve and len(active) > settings.get('minimum_source_faces', 10000):
            weld = settings.get('weld_spacing_m', .000001)
            welded = np.round(active.reshape(-1, 3) / weld) * weld
            vertices, inverse = np.unique(welded, axis=0, return_inverse=True)
            faces = inverse.reshape(-1, 3).astype(np.int32)
            faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 0] != faces[:, 2]) & (faces[:, 1] != faces[:, 2])]
            meshes = connected_meshes(vertices, faces)
            count = max(32, int(len(active) * fraction))
            while count < len(active):
                result = simplify_components(meshes, count / len(active))
                maximum, mean = sampled_error(active, result, seed=2027 + int(part))
                print(f'  Part {part}: candidate {len(result)}, error {maximum * 1000:.2f} mm', flush=True)
                if maximum <= tolerance and mean <= settings.get('sampled_mean_error_tolerance_m', tolerance):
                    break
                count = min(len(active), count * 2)
            if count >= len(active):
                result, maximum, mean = active, 0., 0.
        output = np.concatenate([fixed, result]).astype('<f4')
        chunks.append(output)
        ids.append(np.full(len(output), part, dtype='<i4'))
        report = {'part_id': int(part), 'source_faces': len(original), 'result_faces': len(output),
                      'protected_faces': len(fixed), 'sampled_max_error_m': maximum, 'sampled_mean_error_m': mean}
        reports.append(report)
        print(f'Part {part}: {len(original)} -> {len(output)}; sampled error {maximum * 1000:.2f} mm', flush=True)
    report = {'method': 'quadric edge collapse per connected CAD surface', 'library_version': fs.__version__,
                  'source_faces': len(triangles), 'result_faces': sum(map(len, chunks)),
                  'sampled_error_tolerance_m': tolerance,
                  'sampled_mean_error_tolerance_m': settings.get('sampled_mean_error_tolerance_m', tolerance),
                  'weld_spacing_m': settings.get('weld_spacing_m', .000001),
                  'error_sampling': 'bidirectional nearest-surface distance; up to 250 face centroids and 250 vertices per part, fixed seed',
                  'sampled_max_error_m': max(p['sampled_max_error_m'] for p in reports), 'parts': reports}
    return np.concatenate(chunks), np.concatenate(ids), report
